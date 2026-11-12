import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'hero'))

import follow  # noqa: E402
from vfh import Detection, bearing, choose, distance, histogram, steer  # noqa: E402


def box(label, cx, h, w=None, bottom=0.95, score=0.9):
    """Detection centred at cx (0..1) with height h (fraction of image)."""
    w = w if w is not None else h * 0.4
    return Detection(label, score, cx - w / 2, bottom - h, cx + w / 2, bottom)


class Geometry(unittest.TestCase):
    def test_bearing(self):
        c, hw = bearing(box('person', 0.5, 0.5))
        self.assertAlmostEqual(c, 0.0)
        c, _ = bearing(box('person', 0.75, 0.5))
        self.assertAlmostEqual(c, 15.0)            # quarter of a 60° view to the right

    def test_distance_shrinks_with_box_height(self):
        far, near = distance(box('person', .5, .3)), distance(box('person', .5, .8))
        self.assertGreater(far, near)
        self.assertAlmostEqual(distance(box('person', .5, .5)), 1.7 / (2 * 0.1989), places=1)  # tan(11.25°)


class Histogram(unittest.TestCase):
    def test_far_obstacle_ignored(self):
        h = histogram([box('object', 0.5, 0.2, w=0.2)])     # small box = far away (~3 m)
        self.assertFalse(any(h.blocked))

    def test_close_obstacle_blocks_its_sectors_plus_clearance(self):
        d = box('object', 0.5, 0.8, w=0.2)                   # 0.5 m object, ~0.8 m ahead
        h = histogram([d])
        mid = len(h.blocked) // 2
        self.assertTrue(h.blocked[mid])
        self.assertFalse(h.blocked[0] and h.blocked[-1])
        covered = sum(h.blocked)
        self.assertGreater(covered, round(0.2 * 15))           # wider than the box itself

    def test_choose_prefers_goal_side_and_returns_none_when_all_blocked(self):
        h = histogram([box('object', 0.4, 0.8, w=0.25)])       # obstacle slightly left of centre
        chosen = choose(h, 0.0)
        self.assertIsNotNone(chosen)
        self.assertGreater(chosen, 0)                           # go around on the right
        wall = histogram([box('object', 0.5, 0.95, w=1.0)])
        self.assertIsNone(choose(wall, 0.0))

    def test_steer(self):
        self.assertEqual(steer(0, 50), (50, 50))
        l, r = steer(10, 50)
        self.assertGreater(l, r)                                # right turn: left wheel faster
        l, r = steer(-40, 50)
        self.assertLess(l, 0)                                   # sharp left: spin on the spot
        self.assertTrue(all(-100 <= v <= 100 for v in steer(90, 100)))


class Follow(unittest.TestCase):
    def test_follows_person_straight_ahead(self):
        p = follow.Follower(speed=60).plan([box('person', 0.5, 0.35)])
        self.assertEqual(p.left, p.right)
        self.assertGreater(p.left, 0)

    def test_stops_when_close(self):
        p = follow.Follower().plan([box('person', 0.5, 1.0, bottom=1.0)])   # fills the frame
        self.assertTrue(p.is_stop)

    def test_steers_around_obstacle_toward_person(self):
        person = box('person', 0.5, 0.3)
        chair = box('object', 0.47, 0.85, w=0.3)
        p = follow.Follower(speed=60).plan([person, chair])
        self.assertIsNotNone(p.chosen_deg)
        self.assertNotAlmostEqual(p.chosen_deg, p.goal_deg)
        self.assertIn('around obstacle', p.reason)
        self.assertNotEqual(p.left, p.right)

    def test_keeps_same_person(self):
        f = follow.Follower()
        a, b = box('person', 0.3, 0.4), box('person', 0.75, 0.5)   # b is bigger
        f.last_target = a
        self.assertIs(f.pick([a, b]), a)                         # sticks with a, not the largest
        moved = box('person', 0.33, 0.4)
        self.assertIs(f.pick([moved, b]), moved)

    def test_lost_person_waits_then_searches(self):
        f = follow.Follower()
        f.plan([box('person', 0.5, 0.4)], now=0)
        self.assertTrue(f.plan([], now=0.1).is_stop)                         # just lost: wait
        self.assertTrue(f.plan([], now=1.0).is_stop)
        late = f.plan([], now=0.1 + follow.LOST_WAIT_S + 0.1)
        self.assertEqual((late.left, late.right), (-25, 25))                 # then turn to search

    def test_guard(self):
        close = [box('object', 0.5, 0.9, w=0.3)]
        self.assertIsNotNone(follow.guard(close, 60, 60))
        self.assertIsNone(follow.guard(close, -60, -60))         # backing away is fine
        self.assertIsNone(follow.guard([box('object', 0.5, 0.2)], 60, 60))


if __name__ == '__main__':
    unittest.main()
