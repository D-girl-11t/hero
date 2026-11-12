"""End to end without the board: the app talks RPMsg protocol to m33_sim,
which runs the M33 firmware's own protocol.c and control.c, through a pty.

Build the simulator first:  make -C firmware/m33/test
"""
import json
import os
import subprocess
import sys
import threading
import time
import unittest
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'hero'))
SIM = os.path.join(HERE, '..', '..', 'firmware', 'm33', 'test', 'm33_sim')

from motor_link import RpmsgMotors  # noqa: E402


def wait_for(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


@unittest.skipUnless(os.path.exists(SIM), 'run `make -C firmware/m33/test` first')
class M33SimCase(unittest.TestCase):
    def setUp(self):
        master, slave = os.openpty()
        import tty
        tty.setraw(slave)
        self.sim = subprocess.Popen([SIM, '--fd', str(master)], pass_fds=(master,))
        os.close(master)
        self.dev = os.ttyname(slave)
        self.slave = slave                                   # keep the pty open for the test
        self.events = []
        self.link = RpmsgMotors(dev=self.dev, on_event=lambda k, i: self.events.append(k))

    def tearDown(self):
        self.link.close()
        self.sim.kill()
        self.sim.wait()
        os.close(self.slave)

    def status(self):
        return self.link.status()


class LinkTests(M33SimCase):
    def test_ping_and_link_up(self):
        self.assertTrue(wait_for(lambda: self.status()['firmware']))
        self.assertIn('hero-m33', self.status()['firmware'])
        self.assertTrue(wait_for(lambda: self.status()['link'] == 'up'))

    def test_drive_ramps_and_keepalive_holds_speed(self):
        self.link.drive(60, -40)
        self.assertTrue(wait_for(lambda: self.status()['applied'] == [60, -40], 3))
        time.sleep(1.2)                                      # longer than the 500 ms watchdog
        self.assertEqual(self.status()['applied'], [60, -40])
        self.assertEqual(self.status()['watchdog_trips'], 0)
        self.link.stop()
        self.assertTrue(wait_for(lambda: self.status()['applied'] == [0, 0], 3))

    def test_watchdog_stops_motors_when_linux_goes_quiet(self):
        self.link.drive(50, 50)
        self.assertTrue(wait_for(lambda: self.status()['applied'] == [50, 50], 3))
        self.link._closed.set()                              # simulate a hung Linux: keepalive stops
        self.link._rx.join(1)
        # read the raw reply ourselves now that the link's reader is stopped
        deadline, got = time.monotonic() + 2, b''
        os.set_blocking(self.link.fd, False)
        while time.monotonic() < deadline and b'wd\n' not in got:
            try:
                got += os.read(self.link.fd, 256)
            except BlockingIOError:
                time.sleep(0.02)
        self.assertIn(b'wd\n', got)
        self.link._closed.clear()                            # let tearDown close normally

    def test_bad_command_reports_error(self):
        self.link._send('drv 500 0')
        self.assertTrue(wait_for(lambda: self.status()['last_error']))
        self.assertIn('-100..100', self.status()['last_error'])


class AppTests(unittest.TestCase):
    """The board's app (hero.py) with motors going to m33_sim over the pty."""

    @classmethod
    def setUpClass(cls):
        master, slave = os.openpty()
        import tty
        tty.setraw(slave)
        cls.sim = subprocess.Popen([SIM, '--fd', str(master)], pass_fds=(master,))
        os.close(master)
        cls.slave = slave
        os.environ.update(HERO_MOTOR_BACKEND='rpmsg', HERO_RPMSG_DEV=os.ttyname(slave), HERO_SPEED='60')
        import importlib
        if 'hero' in sys.modules:                # already imported: rebuild M with the env above
            app = importlib.reload(sys.modules['hero'])
        else:                                    # first import builds M with the env above
            import hero as app
        cls.app = app
        from http.server import ThreadingHTTPServer
        cls.srv = ThreadingHTTPServer(('127.0.0.1', 0), app.H)
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()
        cls.base = f'http://127.0.0.1:{cls.srv.server_address[1]}'

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.app.M.link.close()
        cls.sim.kill()
        cls.sim.wait()
        os.close(cls.slave)

    def setUp(self):
        self.app.VIS.mode = 'off'
        self.post('/api/move', {'action': 'stop', 'speed': 60})
        self.assertTrue(wait_for(lambda: self.applied() == [0, 0], 3))

    def post(self, path, body):
        req = urllib.request.Request(self.base + path, json.dumps(body).encode(), {'Content-Type': 'application/json'})
        with urllib.request.urlopen(req) as r:
            return json.load(r)

    def get(self, path):
        with urllib.request.urlopen(self.base + path) as r:
            return json.load(r)

    def applied(self):
        return self.get('/api/state')['link']['applied']

    def test_link_reported(self):
        s = self.get('/api/state')
        self.assertEqual(s['link']['backend'], 'rpmsg')
        self.assertTrue(wait_for(lambda: self.get('/api/state')['link']['link'] == 'up'))
        self.assertTrue(wait_for(lambda: self.get('/api/state')['link']['firmware']))
        self.assertIn('hero-m33', self.get('/api/state')['link']['firmware'])

    def test_move_and_speed_reach_m33(self):
        s = self.post('/api/move', {'action': 'forward', 'seconds': 5, 'speed': 70})
        self.assertEqual(s['motors']['A'], 'forward')
        self.assertTrue(wait_for(lambda: self.applied() == [70, 70], 3))
        self.post('/api/move', {'speed': 40})                 # speed change while moving
        self.assertTrue(wait_for(lambda: self.applied() == [40, 40], 3))
        self.post('/api/move', {'action': 'left', 'seconds': 5})
        self.assertTrue(wait_for(lambda: self.applied() == [-40, 40], 3))

    def test_timed_move_stops_by_itself(self):
        self.post('/api/move', {'action': 'forward', 'seconds': 1, 'speed': 40})
        self.assertTrue(wait_for(lambda: self.applied() == [40, 40], 3))
        self.assertTrue(wait_for(lambda: self.applied() == [0, 0], 3))
        self.assertEqual(self.get('/api/state')['link']['watchdog_trips'], 0)

    def test_say_speed_and_move(self):
        r = self.post('/api/say', {'text': 'speed 80'})
        self.assertEqual(r['speed'], 80)
        r = self.post('/api/say', {'text': 'go back for 2 seconds'})
        self.assertTrue(wait_for(lambda: self.applied() == [-80, -80], 3))
        r = self.post('/api/say', {'text': 'status'})
        self.assertIn('Cortex-M33 over RPMsg', r['reply'])

    def test_follow_steers_with_vfh(self):
        vis = self.app.VIS
        vis.mode = 'follow'
        vis.follower = __import__('follow').Follower(speed=60)
        person = {'label': 'person', 'score': 0.9, 'box': [0.45, 0.6, 0.55, 0.95]}
        chair = {'label': 'chair', 'score': 0.8, 'box': [0.35, 0.0, 0.62, 1.0]}   # fills the frame height: close
        vis._behave([person])
        self.assertEqual(vis.plan.left, vis.plan.right)
        self.assertTrue(wait_for(lambda: self.applied() == [vis.plan.left, vis.plan.right], 3))
        vis._behave([person, chair])                          # chair in the way: VFH goes around
        self.assertIn('around obstacle', vis.plan.reason)
        self.assertNotEqual(vis.plan.left, vis.plan.right)
        want = [vis.plan.left, vis.plan.right]
        self.assertTrue(wait_for(lambda: self.applied() == want, 3))
        vis.mode = 'off'

    def test_guard_stops_forward_motion(self):
        vis = self.app.VIS
        self.post('/api/move', {'action': 'forward', 'seconds': 5, 'speed': 50})
        self.assertTrue(wait_for(lambda: self.applied() == [50, 50], 3))
        vis.mode = 'guard'
        vis._behave([{'label': 'chair', 'score': 0.8, 'box': [0.3, 0.0, 0.7, 1.0]}])
        self.assertTrue(wait_for(lambda: self.applied() == [0, 0], 3))
        self.assertIn('guard', self.get('/api/state')['log'][-1]['text'])
        vis.mode = 'off'


if __name__ == '__main__':
    unittest.main()
