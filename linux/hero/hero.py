#!/usr/bin/env python3
"""Hero: web control for the two L298N motors on the FRDM-IMX93.

Motors go through motor_link: over RPMsg (/dev/ttyRPMSG30) to the Cortex-M33
firmware, which makes the PWM and stops the motors itself if Linux goes quiet;
or, when that firmware isn't running, straight to the GPIO pins as before.

  motor A (left):  IN1 pin 3, IN2 pin 5, ENA pin 32 (PWM from the M33); forward = IN2 high
  motor B (right): IN3 pin 7, IN4 pin 8, ENB pin 33 (PWM from the M33); forward = IN4 high

GET  /            the app
GET  /api/state   current motion and recent log
POST /api/move    {"action": "forward|reverse|left|right|stop", "motor": "A|B|both", "seconds": 3, "speed": 60}
POST /api/say     {"text": "go forward for 3 seconds"}  -> Hero parses it and moves
POST /api/listen  raw 16 kHz mono int16 PCM            -> {"text": "..."} (offline speech-to-text, Vosk)
GET  /camera.mjpg live view (USB camera's own MJPEG frames, passed through without re-encoding)
GET  /camera.jpg  one frame
GET  /api/vision  latest detections (SSD-MobileNet COCO on the Ethos-U65 NPU) and the VFH plan
POST /api/vision  {"detect": bool, "mode": "off|guard|follow", "rot": 0|90|180|270}

Simple commands are matched by rules (instant). Anything else, and sequences ("forward then spin"),
go to a small local language model served by llama-server on 127.0.0.1:8081. Both work offline.
"""
import json, os, re, signal, threading, time, urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import follow, vfh
from motor_link import make_motors

PORT = int(os.environ.get('HERO_PORT', 8080))
MAX_SECONDS = 30                                   # nothing runs longer than this without a new command
HERE = os.path.dirname(os.path.abspath(__file__))
LLM_URL = os.environ.get('HERO_LLM', 'http://127.0.0.1:8081')
VOSK_DIR = os.path.join(HERE, 'models', 'vosk-model-small-en-us-0.15')

# per-motor direction -> sign of the duty sent to the motor backend
SIGN = {'forward': 1, 'reverse': -1, 'stop': 0}


class Motors:
    def __init__(self, link=None):
        self.lock = threading.RLock()
        self.timer = None
        self.state = {'A': 'stop', 'B': 'stop', 'until': None}
        self.log = []
        self.speed = max(20, min(100, int(os.environ.get('HERO_SPEED', 60))))
        self.link = link or make_motors()
        self.link.on_event = self._link_event          # 'wd' and 'err' lines from the M33
        self.note('motors via ' + ('Cortex-M33 over RPMsg' if self.link.name == 'rpmsg' else 'Linux GPIO (no speed control)'))
        self._apply('stop', 'stop')

    def note(self, text):
        self.log.append({'t': time.strftime('%H:%M:%S'), 'text': text})
        del self.log[:-40]

    def _link_event(self, kind, info):
        with self.lock:
            if kind == 'watchdog':                     # the M33 already stopped the wheels
                if self.timer:
                    self.timer.cancel(); self.timer = None
                self.state.update(A='stop', B='stop', until=None)
            self.note(info)

    def _apply(self, a, b):
        self.link.drive(SIGN[a] * self.speed, SIGN[b] * self.speed)
        self.state.update(A=a, B=b)

    def drive(self, left, right, why=None):
        """Raw duty per wheel (-100..100), used by follow mode. No timer: the caller keeps it fresh."""
        with self.lock:
            if self.timer:
                self.timer.cancel(); self.timer = None
            self.link.drive(left, right)
            st = lambda v: 'forward' if v > 0 else 'reverse' if v < 0 else 'stop'
            self.state.update(A=st(left), B=st(right), until=None)
            if why:
                self.note(why)

    def set_speed(self, speed):
        with self.lock:
            self.speed = max(20, min(100, int(speed)))
            if self.state['A'] != 'stop' or self.state['B'] != 'stop':
                self._apply(self.state['A'], self.state['B'])

    def set(self, a, b, seconds=None, why=''):
        with self.lock:
            if self.timer:
                self.timer.cancel(); self.timer = None
            # reversing a spinning motor straight away is hard on the driver: coast briefly first.
            # The M33 does this itself (ramp to 0, 100 ms pause), so only the GPIO path needs it here.
            if self.link.name == 'gpio' and any(self.state[m] not in ('stop', new) and new != 'stop' for m, new in (('A', a), ('B', b))):
                self._apply('stop', 'stop'); time.sleep(0.3)
            self._apply(a, b)
            if a == b == 'stop':
                self.state['until'] = None
                self.note(why or 'stopped')
                return
            secs = min(float(seconds or MAX_SECONDS), MAX_SECONDS)
            self.state['until'] = time.time() + secs
            self.note(f'{why or f"A {a}, B {b}"} for {secs:g} s' + (f' at {self.speed}%' if self.link.name == 'rpmsg' else ''))
            self.timer = threading.Timer(secs, self._timeout)
            self.timer.daemon = True
            self.timer.start()

    def _timeout(self):
        with self.lock:
            self.timer = None
            self._apply('stop', 'stop')
            self.state['until'] = None
            self.note('done, stopped')

    def snapshot(self):
        s = dict(self.state)
        until = s.pop('until')
        s['remaining'] = max(0, round(until - time.time(), 1)) if until else 0
        return {'motors': s, 'speed': self.speed, 'link': self.link.status(), 'log': self.log[-15:]}


M = Motors()

ACTIONS = {  # whole-robot moves: (motor A, motor B)
    'forward': ('forward', 'forward'), 'reverse': ('reverse', 'reverse'),
    'left': ('reverse', 'forward'), 'right': ('forward', 'reverse'), 'stop': ('stop', 'stop'),
}


SEQ = {'gen': 0}   # bumped by every new command; a running sequence stops when it changes


def plan(action, motor='both'):
    if action not in ACTIONS:
        raise ValueError('action must be forward, reverse, left, right or stop')
    if motor in ('A', 'B') and action in ('forward', 'reverse', 'stop'):
        a = action if motor == 'A' else M.state['A']
        b = action if motor == 'B' else M.state['B']
        return a, b, f'motor {motor} {action}'
    return (*ACTIONS[action], action if action != 'stop' else 'stopped')


def move(action, motor='both', seconds=None, manual=False):
    a, b, why = plan(action, motor)
    if manual and VIS.mode == 'follow':
        VIS.mode = 'off'; M.note('follow off: manual control')
    SEQ['gen'] += 1
    M.set(a, b, seconds, why)


def run_steps(steps):
    """Run a list of {action, motor, seconds} one after another, in the background."""
    if VIS.mode == 'follow':
        VIS.mode = 'off'; M.note('follow off: running a plan')
    SEQ['gen'] += 1
    gen = SEQ['gen']

    def worker():
        for st in steps:
            if SEQ['gen'] != gen:
                return
            a, b, why = plan(st['action'], st['motor'])
            M.set(a, b, st['seconds'], why)
            if a == b == 'stop':
                continue
            end = time.time() + st['seconds'] + 0.35
            while time.time() < end:
                if SEQ['gen'] != gen:
                    return
                time.sleep(0.05)
    threading.Thread(target=worker, daemon=True).start()


# ---- "talk to Hero": plain rules today; an offline model can replace parse() later
WORDS = {
    'stop': 'stop', 'halt': 'stop', 'freeze': 'stop', 'brake': 'stop',
    'forward': 'forward', 'ahead': 'forward', 'front': 'forward', 'go': 'forward', 'straight': 'forward',
    'back': 'reverse', 'backward': 'reverse', 'backwards': 'reverse', 'reverse': 'reverse',
    'left': 'left', 'right': 'right', 'spin': 'left', 'turn': 'left',
}
NUMS = {'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5, 'six': 6, 'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10}


def parse(text):
    t = text.lower()
    words = re.findall(r'[a-z]+|\d+(?:\.\d+)?', t)
    if not words:
        return None
    if re.search(r'\b(hi|hello|hey)\b', t) and not any(w in WORDS for w in words):
        return {'reply': "Hi, I'm Hero. Tell me to go forward, back, left, right or stop, and for how long."}
    if re.search(r'\b(who|name)\b', t):
        return {'reply': "I'm Hero: an NXP i.MX93 robot. Linux does the thinking, the Cortex-M33 drives my motors."}
    if re.search(r'\b(status|doing|state)\b', t):
        snap = M.snapshot()
        s, link = snap['motors'], snap['link']
        path = f"the Cortex-M33 over RPMsg (link {link['link']})" if link['backend'] == 'rpmsg' else 'Linux GPIO'
        return {'reply': f"Motor A: {s['A']}, motor B: {s['B']}, speed {snap['speed']}%" + (f", {s['remaining']} s left" if s['remaining'] else '')
                         + f". Motors run through {path}."}
    found = {WORDS[w] for w in words if w in WORDS}
    if 'right' in words:                  # "turn right": 'turn' alone means left, an explicit side wins
        found.discard('left')
    # 'stop' wins, then turns, then back, then forward ("go back" is reverse, not forward)
    action = next((a for a in ('stop', 'right', 'left', 'reverse', 'forward') if a in found), None)
    if not action:
        return None
    motor = 'both'
    m = re.search(r'\bmotor\s*([ab])\b|\b([ab])\s*motor\b', t)
    if m:
        motor = (m.group(1) or m.group(2)).upper()
        if action in ('left', 'right'):
            action = 'forward'
    secs = None
    m = re.search(r'(\d+(?:\.\d+)?|' + '|'.join(NUMS) + r')\s*(s|sec|secs|second|seconds|m|min|minute|minutes)\b', t)
    if m:
        n = float(NUMS.get(m.group(1), m.group(1) if m.group(1)[0].isdigit() else 0))
        secs = n * 60 if m.group(2).startswith('m') else n
    if secs is None and re.search(r'\b(a little|a bit|slightly|bit)\b', t):
        secs = 1
    if re.search(r'\bspin\s*around\b|\bturn around\b', t) and secs is None:
        action, secs = 'right', 3
    explicit = secs is not None
    if action != 'stop' and secs is None:
        secs = 3
    return {'action': action, 'motor': motor, 'seconds': secs, 'explicit': explicit}


# ---- local language model (llama-server). On two A55 cores it writes ~5 tokens/s, so it answers in a
# compact grammar ("left 1, forward 3" or "say: ...") and Hero writes the reply sentence itself.
LLM_GRAMMAR = r'''
root   ::= plan | chat
plan   ::= step (", " step){0,5}
step   ::= action (" " motor)? " " num
action ::= "forward" | "reverse" | "left" | "right" | "stop"
motor  ::= "A" | "B"
num    ::= [0-9] | [1-3] [0-9] | [0-9] "." [05]
chat   ::= "say: " [^\n]{1,100}
'''
LLM_RULES = """You are Hero, a small robot with two motors, A and B. Turn the message into motor steps.
Format: action [A|B] seconds, steps separated by ", ". Actions: forward, reverse, left, right, stop.
Leave out A/B to use both motors. Use the seconds the user gives; otherwise 2. "a little" = 1. "spin around" = right 3.
If the message is not about moving, answer with "say: " and one short friendly sentence.

go forward then come back -> forward 2, reverse 2
turn left then go forward 3 seconds -> left 1, forward 3
do a little dance -> left 1, right 1, left 1, right 1
zigzag forward -> left 1, forward 1, right 1, forward 1, left 1, forward 1
run motor A backwards for 4 seconds then motor B forward -> reverse A 4, forward B 2
spin around twice -> right 3, right 3
how are you -> say: Wheels ready and feeling great!"""


def llm_ready():
    try:
        with urllib.request.urlopen(LLM_URL + '/health', timeout=1) as r:
            return r.status == 200
    except Exception:
        return False


# a message must name some kind of movement before the model may plan motor steps for it;
# anything else ("capture my pic") can only get a spoken answer
MOTION = re.compile(r'\b(move|moving|go|drive|run|roll|turn|spin|rotate|dance|wiggle|zig\w*|circle|square|forward|ahead|back\w*|reverse|left|right|around|motor|wheel\w*|come|stop|patrol|explore|shake)\b')
LLM_CHAT_GRAMMAR = 'root ::= "say: " [^\\n]{1,140}'
LLM_CHAT_RULES = """You are Hero, a cheerful little robot with two wheels and a camera, running on an NXP i.MX93 board.
You have no internet, so you can't check the weather, news or time. Answer in one or two short, friendly sentences.
Begin your answer with "say: "."""


def ask_llm(text):
    moving = bool(MOTION.search(text.lower()))
    body = {'messages': [{'role': 'system', 'content': LLM_RULES if moving else LLM_CHAT_RULES}, {'role': 'user', 'content': text}],
            'temperature': 0.1 if moving else 0.6, 'max_tokens': 60 if moving else 70,
            'grammar': LLM_GRAMMAR if moving else LLM_CHAT_GRAMMAR}
    req = urllib.request.Request(LLM_URL + '/v1/chat/completions', json.dumps(body).encode(),
                                 {'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=90) as r:
        out = json.load(r)['choices'][0]['message']['content'].strip()
    if out.startswith('say:'):
        return out[4:].strip()[:200] or 'OK.', []
    steps = []
    for part in out.split(','):
        w = part.split()
        if len(w) < 2 or w[0] not in ACTIONS:
            continue
        motor = w[1] if len(w) == 3 and w[1] in ('A', 'B') else 'both'
        steps.append({'action': w[0], 'motor': motor, 'seconds': max(0.5, min(float(w[-1]), MAX_SECONDS))})
    return describe(steps), steps[:6]


def describe(steps):
    if not steps:
        return 'OK.'
    words = {'forward': 'forward', 'reverse': 'back', 'left': 'turn left', 'right': 'turn right', 'stop': 'stop'}
    parts = [f"{'motor ' + st['motor'] + ' ' if st['motor'] != 'both' else ''}{words[st['action']]}"
             + ('' if st['action'] == 'stop' else f" {st['seconds']:g} s") for st in steps]
    text = ', then '.join(parts) + '.'
    return text[0].upper() + text[1:]


def parse_sequence(text):
    """'forward 2 s, then turn left and back up for 3 seconds' -> steps, if every part is a plain command."""
    clauses = [c for c in re.split(r'\s*(?:,|;|\bthen\b|\band\b|\bafter that\b|\bafterwards\b)\s*', text.lower()) if c.strip()]
    steps = []
    for c in clauses:
        times = 2 if re.search(r'\btwice\b', c) else 3 if re.search(r'\bthree times\b', c) else 1
        p = parse(re.sub(r'\b(twice|three times)\b', '', c))
        if p is None or 'reply' in p:
            return None
        secs = p['seconds'] if p['explicit'] or p['action'] == 'stop' else 1 if p['action'] in ('left', 'right') else 2
        steps += [{'action': p['action'], 'motor': p['motor'], 'seconds': min(float(secs or 0), MAX_SECONDS)}] * times
    return steps[:6] if len(steps) > 1 else None


def warm_llm():
    """Process the long instructions once at start-up so the first real command is quick."""
    for _ in range(60):
        if llm_ready():
            for warm in ('hello', 'go forward then turn left'):   # one chat prompt, one planning prompt
                try: ask_llm(warm)
                except Exception: pass
            return
        time.sleep(5)


SEQUENCE = re.compile(r'\b(then|after that|afterwards|and back|twice|dance|wiggle|zig|zigzag|zig-zag|circle|around|square|figure)\b')


def say(text):
    t = text.lower()
    if re.search(r'\b(weather|forecast|news|temperature outside|stock|score)\b', t):
        return "I'm offline, so I can't check that. I only know what my camera sees."
    if re.search(r'\bwhat(\W?s| is)? the time\b|\btime is it\b', t):
        return time.strftime("It's %H:%M (%Z) on my clock.")
    if re.search(r'\b(what(\W?s| is)? the date|what day is it|today\W?s date)\b', t):
        return time.strftime('Today is %A, %d %B %Y.')
    if PHOTO.search(t):
        try:
            return {'reply': 'Got it, here you are.', 'photo': take_photo()}
        except Exception as e:
            return f"I couldn't take the photo: {e}"
    if re.search(r'\b(what|who)\b.*\b(see|front|there|look)', t) or re.search(r'\blook around\b|\bdo you see\b', t):
        return VIS.describe()
    if re.search(r'\bfollow (me|him|her|them|the person)\b', t):
        VIS.configure(detect=True, mode='follow')
        return "Following you. Say \"stop following\" when you're done."
    if re.search(r'\b(stop|quit|end) following\b', t):
        VIS.configure(mode='off')
        return 'OK, I stopped following.'
    m = re.search(r'\bspeed\s*(?:to\s*)?(\d{2,3})\b|\b(\d{2,3})\s*(?:%|percent)', t)
    if m:
        M.set_speed(int(m.group(1) or m.group(2)))
        return f'Speed set to {M.speed}%.' + ('' if M.link.name == 'rpmsg' else ' (Speed control needs the M33 firmware; on GPIO the motors run full on.)')
    if re.search(r'\b(faster|speed up)\b', t):
        M.set_speed(M.speed + 15); return f'Faster: {M.speed}%.'
    if re.search(r'\b(slower|slow down)\b', t):
        M.set_speed(M.speed - 15); return f'Slower: {M.speed}%.'
    if re.search(r'\b(guard|watch out|be careful)\b', t):
        VIS.configure(detect=True, mode='guard')
        return "Guard is on: I'll stop if something is right in front of me."
    seq = parse_sequence(text)
    if seq:
        run_steps(seq)
        return describe(seq)
    p = None if SEQUENCE.search(t) else parse(text)
    if p is None:
        if not llm_ready():
            return "I didn't get that, and my AI brain is offline right now. Try \"forward 3 seconds\", \"turn left\" or \"stop\"."
        try:
            reply, steps = ask_llm(text)
        except Exception as e:
            return f"My AI brain had trouble with that ({type(e).__name__}). Try a simpler command."
        if steps:
            run_steps(steps)
            M.note('AI plan: ' + ', '.join(f"{s['action']}{'' if s['motor'] == 'both' else ' ' + s['motor']} {s['seconds']:g}s" for s in steps))
        return reply
    if 'reply' in p:
        return p['reply']
    move(p['action'], p['motor'], p['seconds'], manual=True)
    if p['action'] == 'stop':
        return 'Stopped.'
    secs = min(p['seconds'], MAX_SECONDS)
    who = 'both motors' if p['motor'] == 'both' else f"motor {p['motor']}"
    if p['action'] in ('left', 'right'):
        return f"Turning {p['action']} for {secs:g} s."
    return f"Running {who} {p['action']} for {secs:g} s." + (' (capped at 30 s)' if p['seconds'] > MAX_SECONDS else '')


class Camera:
    """Reads the USB camera only while someone is watching; frames stay JPEG from the camera."""
    W, H, FPS, IDLE = 640, 480, 15, 5

    def __init__(self):
        self.cond = threading.Condition()
        self.frame, self.seq, self.last_want, self.thread, self.error = None, 0, 0, None, None

    @staticmethod
    def device():
        # the capture node of the first USB (UVC) camera; its sibling node is metadata only
        for d in sorted(os.listdir('/sys/class/video4linux'), key=lambda x: int(x[5:])):
            name = open(f'/sys/class/video4linux/{d}/name').read()
            if 'mxc_isi' not in name and open(f'/sys/class/video4linux/{d}/index').read().strip() == '0':
                return f'/dev/{d}'
        return None

    def _run(self):
        import cv2
        dev = self.device()
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2) if dev else None
        if not cap or not cap.isOpened():
            with self.cond:
                self.error, self.thread = 'no USB camera found', None
                self.cond.notify_all()
            return
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)
        cap.set(cv2.CAP_PROP_FPS, self.FPS)
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)              # hand back the camera's JPEG bytes as they are
        self.error = None
        try:
            while time.time() - self.last_want < self.IDLE:
                ok, f = cap.read()
                if not ok:
                    self.error = 'camera stopped sending frames'; break
                with self.cond:
                    self.frame, self.seq = f.tobytes(), self.seq + 1
                    self.cond.notify_all()
        finally:
            cap.release()
            with self.cond:
                self.thread = None
                self.cond.notify_all()

    def get(self, after=0, timeout=3):
        """Next frame newer than `after`: (seq, jpeg) or (after, None) on timeout."""
        with self.cond:
            self.last_want = time.time()
            if not self.thread:
                self.thread = threading.Thread(target=self._run, daemon=True)
                self.thread.start()
            self.cond.wait_for(lambda: self.seq > after or (self.error and not self.thread), timeout)
            return (self.seq, self.frame) if self.seq > after else (after, None)


CAM = Camera()
DET_DIR = os.path.join(HERE, 'models', 'detect')


class Vision:
    """Object detection on the NPU, plus two behaviours that use it: guard and follow."""
    MIN_SCORE = 0.45
    CLOSE = 0.30            # a box covering this much of the view counts as "right in front"

    def __init__(self):
        self.detect, self.mode, self.rot = False, 'off', 0
        self.result = {'objects': [], 'fps': 0, 'ms': 0, 't': 0}
        self.thread, self.error, self.lock = None, None, threading.Lock()
        self.last_cmd = 0
        self.follower = follow.Follower(speed=M.speed)
        self.plan, self.last_reason = None, None

    def ready(self):
        return os.path.exists(os.path.join(DET_DIR, 'detect_vela.tflite'))

    def configure(self, detect=None, mode=None, rot=None):
        with self.lock:
            if rot in (0, 90, 180, 270): self.rot = rot
            if mode in ('off', 'guard', 'follow'):
                if self.mode == 'follow' and mode != 'follow':
                    move('stop')
                if mode == 'follow' and self.mode != 'follow':
                    self.follower = follow.Follower(speed=M.speed)
                self.mode = mode
                self.plan = None
                M.note(f'vision mode: {mode}')
            if detect is not None: self.detect = bool(detect)
            if (self.detect or self.mode != 'off') and not self.thread:
                self.thread = threading.Thread(target=self._run, daemon=True)
                self.thread.start()

    def _run(self):
        import numpy as np, cv2, warnings
        warnings.filterwarnings('ignore')
        import tflite_runtime.interpreter as tfl
        try:
            it = tfl.Interpreter(model_path=os.path.join(DET_DIR, 'detect_vela.tflite'),
                                 experimental_delegates=[tfl.load_delegate('/usr/lib/libethosu_delegate.so')])
            it.allocate_tensors()
            labels = [l.strip() for l in open(os.path.join(DET_DIR, 'labelmap.txt'))]
            inp = it.get_input_details()[0]; h, w = inp['shape'][1:3]
            outs = it.get_output_details()
            self.error = None
        except Exception as e:
            self.error, self.thread = f'detector failed to load: {e}', None
            return
        seq, times = 0, []
        while self.detect or self.mode != 'off':
            seq, jpg = CAM.get(seq)
            if jpg is None:
                if CAM.error: self.error = CAM.error; time.sleep(1)
                continue
            t0 = time.time()
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)   # mono camera: grey copied to 3 channels
            # detect on the upright picture: the model only knows people standing up, not lying sideways
            turn = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}.get(self.rot)
            if turn is not None:
                img = cv2.rotate(img, turn)
            it.set_tensor(inp['index'], cv2.resize(img, (w, h))[None])
            it.invoke()
            boxes, cls, score = (it.get_tensor(o['index'])[0] for o in outs[:3])
            objs = []
            for b, c, sc in zip(boxes, cls, score):
                if sc < self.MIN_SCORE: continue
                y0, x0, y1, x1 = (min(max(float(v), 0.0), 1.0) for v in b)
                objs.append({'label': labels[int(c) + 1], 'score': round(float(sc), 2), 'box': [round(v, 3) for v in (x0, y0, x1, y1)]})
            now = time.time()
            times = [t for t in times if now - t < 2] + [now]
            self.result = {'objects': objs, 'fps': round(len(times) / 2, 1), 'ms': round((now - t0) * 1000), 't': now}
            self._behave(objs)
        self.thread = None

    def _behave(self, objs):
        area = lambda o: (o['box'][2] - o['box'][0]) * (o['box'][3] - o['box'][1])
        dets = [vfh.Detection(o['label'], o['score'], *o['box']) for o in objs]
        if self.mode == 'guard':
            l, r = SIGN[M.state['A']] * M.speed, SIGN[M.state['B']] * M.speed
            p = follow.guard(dets, l, r)
            self.plan = p
            if p:
                move('stop')
                M.note(p.reason)
        elif self.mode == 'follow' and M.link.name == 'rpmsg':
            # the M33 gives real speed control: steer continuously with VFH around obstacles
            self.follower.speed = M.speed
            p = self.follower.plan(dets)
            self.plan = p
            M.drive(p.left, p.right, why=None if p.reason == self.last_reason else 'follow: ' + p.reason)
            self.last_reason = p.reason
        elif self.mode == 'follow' and time.time() - self.last_cmd > 0.4:
            people = sorted((o for o in objs if o['label'] == 'person'), key=area, reverse=True)
            if not people:
                if M.state['A'] != 'stop' or M.state['B'] != 'stop': M.set('stop', 'stop', why='follow: lost the person')
                return
            o = people[0]
            cx = (o['box'][0] + o['box'][2]) / 2
            self.last_cmd = time.time()
            # short bursts: the L298N inputs are on/off, so speed comes from how long each pulse is
            if cx < 0.38:   M.set(*ACTIONS['left'], 0.25, 'follow: turn left')
            elif cx > 0.62: M.set(*ACTIONS['right'], 0.25, 'follow: turn right')
            elif area(o) < 0.25: M.set(*ACTIONS['forward'], 0.4, 'follow: closer')
            elif M.state['A'] != 'stop' or M.state['B'] != 'stop': M.set('stop', 'stop', why='follow: close enough')

    def snapshot(self):
        fresh = time.time() - self.result['t'] < 2
        return {'ready': self.ready(), 'detect': self.detect, 'mode': self.mode, 'rot': self.rot, 'error': self.error,
                'objects': self.result['objects'] if fresh else [], 'fps': self.result['fps'] if fresh else 0, 'ms': self.result['ms'],
                'plan': plan_json(self.plan) if fresh else None}


def plan_json(p):
    """VFH plan for the UI: the 15 direction sectors, where the person is, and the path picked."""
    if p is None or p.hist is None:
        return None
    return {'left': p.left, 'right': p.right, 'reason': p.reason, 'goal_deg': p.goal_deg, 'chosen_deg': p.chosen_deg,
            'sectors': [{'deg': round(c, 1), 'blocked': b} for c, b in zip(p.hist.centers, p.hist.blocked)]}

    def describe(self):
        if not self.ready():
            return "My eyes aren't set up yet."
        if not self.thread:
            self.configure(detect=True)
            time.sleep(1.5)
        objs = self.snapshot()['objects']
        if not objs:
            return "I don't see anything I recognise right now."
        names = {}
        for o in objs: names[o['label']] = names.get(o['label'], 0) + 1
        parts = [f'{n} {k}s' if n > 1 else f'a {k}' for k, n in names.items()]
        return 'I see ' + (', '.join(parts[:-1]) + ' and ' + parts[-1] if len(parts) > 1 else parts[0]) + '.'


VIS = Vision()
PHOTO_DIR = os.path.join(HERE, 'photos')
PHOTO = re.compile(r'\b(capture|take|snap|click|grab|shoot)\b.*\b(pic|pics|picture|photo|selfie|snapshot|image|shot)\b|\b(selfie|snapshot)\b|\b(photo|picture|pic) of\b')


def take_photo():
    """Save one frame, turned upright the way the viewer rotates the live view. Returns its URL path."""
    import numpy as np, cv2
    _, jpg = CAM.get()
    if not jpg:
        raise RuntimeError(CAM.error or 'no camera frame')
    turn = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}.get(VIS.rot)
    if turn is not None:
        jpg = cv2.imencode('.jpg', cv2.rotate(cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR), turn),
                           [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tobytes()
    os.makedirs(PHOTO_DIR, exist_ok=True)
    name = time.strftime('hero-%Y%m%d-%H%M%S.jpg')
    with open(os.path.join(PHOTO_DIR, name), 'wb') as f:
        f.write(jpg)
    M.note(f'photo saved: {name}')
    return '/photos/' + name
VOSK = {'model': None, 'lock': threading.Lock()}


def voice_ready():
    return os.path.isdir(VOSK_DIR)


def transcribe(pcm):
    from vosk import Model, KaldiRecognizer, SetLogLevel
    with VOSK['lock']:
        if VOSK['model'] is None:
            SetLogLevel(-1)
            VOSK['model'] = Model(VOSK_DIR)
        rec = KaldiRecognizer(VOSK['model'], 16000)
        rec.AcceptWaveform(pcm)
        return json.loads(rec.FinalResult()).get('text', '')


class H(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype='application/json'):
        b = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(b)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            with open(os.path.join(HERE, 'index.html'), 'rb') as f:
                return self._send(200, f.read(), 'text/html; charset=utf-8')
        if self.path.startswith('/camera.jpg'):
            _, jpg = CAM.get()
            return self._send(200, jpg, 'image/jpeg') if jpg else self._send(503, {'error': CAM.error or 'no frame'})
        if self.path.startswith('/photos/'):
            name = os.path.basename(self.path.split('?')[0])
            path = os.path.join(PHOTO_DIR, name)
            if re.fullmatch(r'hero-[\d-]+\.jpg', name) and os.path.isfile(path):
                with open(path, 'rb') as f:
                    return self._send(200, f.read(), 'image/jpeg')
            return self._send(404, {'error': 'no such photo'})
        if self.path.startswith('/camera.mjpg'):
            return self._stream()
        if self.path == '/api/state':
            return self._send(200, {**M.snapshot(), 'ai': {'llm': llm_ready(), 'voice': voice_ready()},
                                    'camera': {'device': Camera.device(), 'error': CAM.error}, 'vision': VIS.snapshot()})
        if self.path == '/api/vision':
            return self._send(200, VIS.snapshot())
        self._send(404, {'error': 'not found'})

    def do_POST(self):
        try:
            n = int(self.headers.get('Content-Length') or 0)
            if self.path == '/api/listen':
                if not voice_ready():
                    return self._send(503, {'error': 'speech model not installed'})
                if n > 16000 * 2 * 20:
                    return self._send(413, {'error': 'keep voice commands under 20 seconds'})
                return self._send(200, {'text': transcribe(self.rfile.read(n))})
            body = json.loads(self.rfile.read(n) or b'{}')
            if self.path == '/api/move':
                if body.get('speed') is not None:
                    M.set_speed(body['speed'])
                if body.get('action'):
                    move(str(body['action']), str(body.get('motor', 'both')), body.get('seconds'), manual=True)
                return self._send(200, M.snapshot())
            if self.path == '/api/vision':
                rot = body.get('rot')
                VIS.configure(body.get('detect'), body.get('mode'), int(rot) if rot is not None else None)
                return self._send(200, VIS.snapshot())
            if self.path == '/api/say':
                out = say(str(body.get('text', ''))[:300])
                out = out if isinstance(out, dict) else {'reply': out}
                return self._send(200, {**out, **M.snapshot()})
            self._send(404, {'error': 'not found'})
        except (ValueError, TypeError) as e:
            self._send(400, {'error': str(e)})

    def _stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        seq = 0
        try:
            while True:
                seq, jpg = CAM.get(seq)
                if jpg is None:
                    if CAM.error: break
                    continue
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(jpg) + jpg + b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *a):
        pass


def shutdown(*_):
    M.set('stop', 'stop', why='server stopping')
    M.link.close()
    os._exit(0)


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    print(f'Hero listening on :{PORT}', flush=True)
    threading.Thread(target=warm_llm, daemon=True).start()
    ThreadingHTTPServer(('0.0.0.0', PORT), H).serve_forever()
