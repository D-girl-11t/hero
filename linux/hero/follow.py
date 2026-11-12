"""Person following and collision guard, built on vfh.py.

Follower.plan() takes one frame's detections and returns the wheel command:
  - picks the person to follow (the one it followed last frame if still there,
    otherwise the largest), so it doesn't jump between people
  - treats every other detection as an obstacle
  - steers toward the person through the free sector VFH picks
  - stops at a comfortable distance, turns in place to search when the person is lost
"""
import time
from dataclasses import dataclass, field

from vfh import ahead_blocked, bearing, choose, distance, histogram, steer

STOP_AT_M = 0.8          # follow distance
SLOW_AT_M = 1.5
LOST_WAIT_S = 1.5        # person out of view this long -> turn on the spot to look for them
FORGET_S = 5.0           # after this, any person may become the new target
MIN_SCORE = 0.5


@dataclass
class Plan:
    left: int
    right: int
    reason: str
    target: object = None
    goal_deg: float = None
    chosen_deg: float = None
    hist: object = None
    obstacles: list = field(default_factory=list)

    @property
    def is_stop(self):
        return self.left == 0 and self.right == 0


def _iou(a, b):
    ix = max(0.0, min(a.x1, b.x1) - max(a.x0, b.x0))
    iy = max(0.0, min(a.y1, b.y1) - max(a.y0, b.y0))
    inter = ix * iy
    union = a.area + b.area - inter
    return inter / union if union > 0 else 0.0


class Follower:
    def __init__(self, speed=55):
        self.speed = speed
        self.last_target = None
        self.lost_since = None

    def pick(self, persons):
        if not persons:
            return None
        if self.last_target is not None:
            best = max(persons, key=lambda p: _iou(p, self.last_target))
            if _iou(best, self.last_target) > 0.1:
                return best
            near = min(persons, key=lambda p: abs(p.cx - self.last_target.cx))
            if abs(near.cx - self.last_target.cx) < 0.15:
                return near
        return max(persons, key=lambda p: p.area)

    def plan(self, detections, now=None):
        now = time.monotonic() if now is None else now
        dets = [d for d in detections if d.score >= MIN_SCORE]
        target = self.pick([d for d in dets if d.label == 'person'])
        obstacles = [d for d in dets if d is not target]
        h = histogram(obstacles)

        if target is None:
            if self.lost_since is None:
                self.lost_since = now
            gone = now - self.lost_since
            if gone > FORGET_S:
                self.last_target = None
            if gone < LOST_WAIT_S:
                return Plan(0, 0, 'person lost: waiting', hist=h, obstacles=obstacles)
            return Plan(-25, 25, 'person lost: turning to search', hist=h, obstacles=obstacles)

        self.lost_since = None
        self.last_target = target
        goal, _ = bearing(target)
        dist = distance(target)

        if dist <= STOP_AT_M:
            if abs(goal) > 10:
                l, r = steer(goal, 0, gain=1.0, max_turn=30)
                return Plan(l, r, f'close ({dist:.1f} m): turning to face', target, goal, goal, h, obstacles)
            return Plan(0, 0, f'close enough ({dist:.1f} m)', target, goal, None, h, obstacles)

        chosen = choose(h, goal)
        if chosen is None:
            return Plan(0, 0, 'every direction blocked: stopped', target, goal, None, h, obstacles)
        base = self.speed * (0.6 if dist < SLOW_AT_M else 1.0)
        l, r = steer(chosen, base)
        why = f'person {dist:.1f} m at {goal:+.0f}°'
        if abs(chosen - goal) > h.width:
            why += f', steering around obstacle via {chosen:+.0f}°'
        return Plan(l, r, why, target, goal, chosen, h, obstacles)


def guard(detections, left, right):
    """For manual driving: stop forward motion when something is close ahead."""
    if left + right <= 0:
        return None
    h = histogram([d for d in detections if d.score >= MIN_SCORE])
    if ahead_blocked(h):
        near = min(n for c, n in zip(h.centers, h.nearest_m) if abs(c) <= 12)
        return Plan(0, 0, f'guard: obstacle {near:.1f} m ahead', hist=h)
    return None
