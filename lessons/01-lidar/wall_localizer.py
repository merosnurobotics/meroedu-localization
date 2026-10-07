"""Small known-rectangle range matcher, adapted from Team 14 (MIT).
No ROS, maps, logs or sensor driver required. Units: meters and radians.
"""
import math
from dataclasses import dataclass

@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float

BOUNDS = (-2.0, 2.0, -2.0, 2.0)

def wall_range(x, y, angle, bounds=BOUNDS):
    xmin, xmax, ymin, ymax = bounds
    if not (xmin < x < xmax and ymin < y < ymax):
        return math.inf
    c, s = math.cos(angle), math.sin(angle)
    distances = []
    if abs(c) > 1e-9:
        distances.append(((xmax if c > 0 else xmin) - x) / c)
    if abs(s) > 1e-9:
        distances.append(((ymax if s > 0 else ymin) - y) / s)
    return min(distances)

def score(pose, beams, bounds=BOUNDS):
    """Count short returns too: assumes obstacles are BELOW the scan plane."""
    residuals = [min(abs(r - wall_range(pose.x, pose.y, pose.yaw + a, bounds)), .35)
                 for a, r in beams if math.isfinite(r) and .05 < r < 10]
    if len(residuals) < 8:
        return math.inf
    residuals.sort()
    keep = max(8, math.ceil(len(residuals) * .7))
    return sum(residuals[:keep]) / keep

def grid_search(beams, yaw, bounds=BOUNDS, step=.2):
    """Yaw is known at initialization; search x/y then refine."""
    if step <= 0:
        raise ValueError('step must be positive')
    xmin, xmax, ymin, ymax = bounds
    candidates = []
    x = xmin + step / 2
    while x < xmax:
        y = ymin + step / 2
        while y < ymax:
            p = Pose(x, y, yaw)
            candidates.append((score(p, beams, bounds), p))
            y += step
        x += step
    best_score, best = min(candidates, key=lambda v: v[0])
    if not math.isfinite(best_score):
        raise ValueError('at least 8 usable beams required')
    for scale in [step / 4, step / 16, step / 64]:
        options = [(score(p, beams, bounds), p)
                   for dx in range(-4, 5) for dy in range(-4, 5)
                   if xmin < (p := Pose(best.x + dx*scale, best.y + dy*scale, yaw)).x < xmax
                   and ymin < p.y < ymax]
        best_score, best = min(options, key=lambda v: v[0])
    return best, best_score

def scan_to_points(pose, beams):
    return [(pose.x+r*math.cos(pose.yaw+a), pose.y+r*math.sin(pose.yaw+a))
            for a, r in beams if math.isfinite(r) and r > 0]
