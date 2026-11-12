"""VFH (Vector Field Histogram) steering from camera detections.

The camera's horizontal view is split into angle sectors. Each detected
obstacle closer than BLOCK_M marks the sectors it covers as blocked, widened by
the robot's half-width so Hero doesn't clip edges. Hero then steers toward the
free sector closest to where it wants to go.

Angles are degrees, 0 = straight ahead, positive = to the right.
Boxes are normalised image coordinates (0..1) after any camera rotation.
"""
import math
from dataclasses import dataclass, field

HFOV_DEG = 60.0          # camera horizontal field of view (after rotation)
VFOV_DEG = 45.0
SECTORS = 15             # 4 degrees each
BLOCK_M = 1.2            # obstacles nearer than this block their sectors
HALF_WIDTH_M = 0.13      # half of Hero's width plus a little margin

# typical real heights used to turn box height into distance
CLASS_HEIGHT_M = {'person': 1.7, 'chair': 0.9, 'bottle': 0.25, 'cup': 0.1, 'dog': 0.5, 'cat': 0.3, 'potted plant': 0.5}
DEFAULT_HEIGHT_M = 0.5


@dataclass
class Detection:
    label: str
    score: float
    x0: float
    y0: float
    x1: float
    y1: float

    @property
    def area(self):
        return max(0.0, self.x1 - self.x0) * max(0.0, self.y1 - self.y0)

    @property
    def cx(self):
        return (self.x0 + self.x1) / 2


def bearing(d, hfov=HFOV_DEG):
    """(centre angle, half angular width) of a box, in degrees."""
    return (d.cx - 0.5) * hfov, (d.x1 - d.x0) * hfov / 2


CUT_OFF_M = 0.6         # a box that spans the whole frame height is cut off: treat it as this close


def distance(d, vfov=VFOV_DEG):
    """Distance in metres from apparent height (pinhole model)."""
    real = CLASS_HEIGHT_M.get(d.label, DEFAULT_HEIGHT_M)
    ang = max(1e-3, (d.y1 - d.y0) * math.radians(vfov))
    est = real / (2 * math.tan(ang / 2))
    if d.y0 <= 0.02 and d.y1 >= 0.98:      # top and bottom both cut off: closer than the estimate says
        return min(est, CUT_OFF_M)
    return est


@dataclass
class Histogram:
    centers: list
    blocked: list
    nearest_m: list = field(default_factory=list)

    @property
    def width(self):
        return self.centers[1] - self.centers[0] if len(self.centers) > 1 else 0


def histogram(obstacles, hfov=HFOV_DEG, sectors=SECTORS, block_m=BLOCK_M, half_width_m=HALF_WIDTH_M):
    w = hfov / sectors
    centers = [-hfov / 2 + w * (i + 0.5) for i in range(sectors)]
    blocked = [False] * sectors
    nearest = [math.inf] * sectors
    for d in obstacles:
        dist = distance(d)
        if dist > block_m:
            continue
        c, hw = bearing(d, hfov)
        hw += math.degrees(math.atan2(half_width_m, max(dist, 0.05)))   # robot clearance
        for i, s in enumerate(centers):
            if abs(s - c) <= hw + w / 2:
                blocked[i] = True
                nearest[i] = min(nearest[i], dist)
    return Histogram(centers, blocked, nearest)


def choose(h, goal_deg):
    """Free sector closest to the goal; on a tie, the one inside the wider gap. None if all blocked."""
    gaps = _gap_sizes(h.blocked)
    best, key = None, None
    for i, (c, b) in enumerate(zip(h.centers, h.blocked)):
        if b:
            continue
        k = (round(abs(c - goal_deg), 6), -gaps[i])
        if key is None or k < key:
            best, key = c, k
    return best


def _gap_sizes(blocked):
    sizes, i = [0] * len(blocked), 0
    while i < len(blocked):
        if blocked[i]:
            i += 1
            continue
        j = i
        while j < len(blocked) and not blocked[j]:
            j += 1
        for k in range(i, j):
            sizes[k] = j - i
        i = j
    return sizes


def steer(angle_deg, base, gain=1.6, max_turn=45):
    """Wheel duties that head toward angle_deg. Positive angle = turn right = left wheel faster.
    Slows down for sharp turns so Hero turns on the spot instead of swinging wide."""
    turn = max(-max_turn, min(max_turn, gain * angle_deg))
    if abs(angle_deg) > 20:
        base *= 0.4
    clamp = lambda v: max(-100, min(100, int(round(v))))
    return clamp(base + turn), clamp(base - turn)


def ahead_blocked(h, cone_deg=12):
    """True when any sector within cone_deg of straight ahead is blocked."""
    return any(b for c, b in zip(h.centers, h.blocked) if abs(c) <= cone_deg)
