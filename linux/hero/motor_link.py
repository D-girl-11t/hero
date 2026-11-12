"""Motor backends for Hero.

RpmsgMotors   talks to the M33 firmware over /dev/ttyRPMSG30 (RPMsg). The M33
              makes the PWM, ramps speed, and stops the motors by itself if
              Linux goes quiet for 500 ms, so this class re-sends the current
              command every 200 ms while moving.
GpioMotors    the original setup: Linux drives IN1-IN4 with gpioset. No speed
              control (ENA/ENB jumpered), no watchdog. Kept as a fallback.

Both take duty in percent, -100..100 per side (left = motor A, right = motor B).
"""
import glob
import os
import select
import subprocess
import threading
import time
import tty

KEEPALIVE_S = 0.2          # well inside the M33's 500 ms watchdog
STATUS_EVERY_S = 1.0       # 'st' poll while stopped, to keep link status fresh
LINK_TIMEOUT_S = 2.5       # no reply for this long -> link reported down


def clamp_duty(v):
    return max(-100, min(100, int(round(v))))


def find_rpmsg_dev():
    devs = sorted(glob.glob('/dev/ttyRPMSG*'))
    return devs[0] if devs else None


class RpmsgMotors:
    name = 'rpmsg'

    def __init__(self, dev=None, on_event=None):
        self.dev = dev or os.environ.get('HERO_RPMSG_DEV') or find_rpmsg_dev()
        if not self.dev:
            raise FileNotFoundError('no /dev/ttyRPMSG* device: is the M33 firmware running? (cat /sys/class/remoteproc/remoteproc0/state)')
        self.fd = os.open(self.dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            tty.setraw(self.fd)
        except Exception:
            pass                                    # not every rpmsg tty supports termios
        self.on_event = on_event or (lambda kind, info: None)
        self.lock = threading.Lock()
        self.target = (0, 0)
        self.applied = (0, 0)                       # last duty the M33 reported on the pins
        self.version = None
        self.trips = 0
        self.last_rx = 0.0
        self.last_error = None
        self.sent = 0
        self.received = 0
        self._closed = threading.Event()
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx.start()
        self._tx = threading.Thread(target=self._keepalive_loop, daemon=True)
        self._tx.start()
        self._send('ping')

    # ---- commands
    def drive(self, left, right):
        l, r = clamp_duty(left), clamp_duty(right)
        with self.lock:
            self.target = (l, r)
            self._send('stop' if (l, r) == (0, 0) else f'drv {l} {r}')

    def stop(self):
        self.drive(0, 0)

    def status(self):
        with self.lock:
            up = bool(self.last_rx) and time.monotonic() - self.last_rx < LINK_TIMEOUT_S
            return {
                'backend': self.name, 'device': self.dev, 'link': 'up' if up else 'down',
                'firmware': self.version, 'target': list(self.target), 'applied': list(self.applied),
                'watchdog_trips': self.trips, 'last_error': self.last_error,
                'messages': {'sent': self.sent, 'received': self.received},
            }

    def close(self):
        try:
            self.stop()
        finally:
            self._closed.set()
            os.close(self.fd)

    # ---- plumbing
    def _send(self, line):
        try:
            os.write(self.fd, (line + '\n').encode())
            self.sent += 1
        except OSError as e:
            self.last_error = f'write: {e.strerror}'

    def _keepalive_loop(self):
        last_status = 0.0
        while not self._closed.wait(KEEPALIVE_S):
            with self.lock:
                moving = self.target != (0, 0) or self.applied != (0, 0)
                if self.target != (0, 0):
                    self._send('drv %d %d' % self.target)
                # applied duty: every tick while the wheels turn (ramps show up live), else once a second
                if moving or time.monotonic() - last_status >= STATUS_EVERY_S:
                    last_status = time.monotonic()
                    self._send('st')

    def _rx_loop(self):
        buf = b''
        while not self._closed.is_set():
            try:
                ready, _, _ = select.select([self.fd], [], [], 0.2)
                if not ready:
                    continue
                chunk = os.read(self.fd, 512)
            except (OSError, ValueError):
                if self._closed.is_set():
                    return
                time.sleep(0.2)
                continue
            if not chunk:
                time.sleep(0.05)
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self._handle(line.decode(errors='replace').strip())

    def _handle(self, line):
        if not line:
            return
        parts = line.split()
        event = None
        with self.lock:
            self.last_rx = time.monotonic()
            self.received += 1
            kind = parts[0]
            if kind == 'ok' and len(parts) == 3:
                pass                                            # target accepted; 'st' reports what's on the pins
            elif kind == 'st' and len(parts) == 5:
                self.applied = (int(parts[1]), int(parts[2]))
                self.trips = int(parts[3])
            elif kind == 'pong':
                self.version = line[5:]
            elif kind == 'wd':
                # the M33 stopped the motors because we went quiet: don't restart them on our own
                self.target = (0, 0)
                self.applied = (0, 0)
                self.trips += 1
                event = ('watchdog', 'M33 stopped the motors: no command for 500 ms')
            elif kind == 'err':
                self.last_error = line[4:]
                event = ('error', self.last_error)
        if event:
            self.on_event(*event)


class GpioMotors:
    """Linux drives the L298N direction pins directly. Speed is ignored (full on)."""
    name = 'gpio'
    CHIP, LINES = 'gpiochip0', (2, 3, 4, 14)               # IN1, IN2, IN3, IN4

    def __init__(self, on_event=None):
        self.proc = None
        self.target = (0, 0)
        subprocess.run(['pkill', '-x', 'gpioset'], stderr=subprocess.DEVNULL)
        self.drive(0, 0)

    @staticmethod
    def _pins(duty):                                         # forward = INb high (Hero's wiring)
        return (0, 1) if duty > 0 else (1, 0) if duty < 0 else (0, 0)

    def drive(self, left, right):
        l, r = clamp_duty(left), clamp_duty(right)
        if self.proc:
            self.proc.terminate()
            try:
                self.proc.wait(1)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        levels = self._pins(l) + self._pins(r)
        self.proc = subprocess.Popen(['gpioset', '-c', self.CHIP] + [f'{n}={v}' for n, v in zip(self.LINES, levels)])
        self.target = (l, r)

    def stop(self):
        self.drive(0, 0)

    def status(self):
        return {'backend': self.name, 'link': 'local', 'target': list(self.target), 'applied': list(self.target),
                'note': 'Linux drives the pins; no PWM speed control or M33 watchdog'}

    def close(self):
        self.stop()
        if self.proc:
            self.proc.terminate()


def make_motors(on_event=None):
    """HERO_MOTOR_BACKEND = auto (default) | rpmsg | gpio."""
    choice = os.environ.get('HERO_MOTOR_BACKEND', 'auto')
    if choice == 'rpmsg' or (choice == 'auto' and (os.environ.get('HERO_RPMSG_DEV') or find_rpmsg_dev())):
        return RpmsgMotors(on_event=on_event)
    return GpioMotors(on_event=on_event)
