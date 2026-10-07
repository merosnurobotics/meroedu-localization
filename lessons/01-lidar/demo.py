"""Generate a tiny deterministic SYNTHETIC scan and recover its pose."""
import argparse, csv, json, math, random
from pathlib import Path
from wall_localizer import Pose, wall_range, grid_search

parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, default=Path('output'))
args = parser.parse_args()
rng = random.Random(14)
truth = Pose(.63, -.47, .35)
beams = [(a, wall_range(truth.x, truth.y, truth.yaw+a) + rng.gauss(0,.008))
         for a in [i*2*math.pi/48 for i in range(48)]]
# Deliberate bad measurements, not actual obstacle geometry.
beams[5] = (beams[5][0], .2)
beams[23] = (beams[23][0], .3)
estimate, residual = grid_search(beams, truth.yaw)
args.output.mkdir(parents=True, exist_ok=True)
with (args.output/'scan.csv').open('w') as f:
    writer = csv.writer(f); writer.writerow(['angle_rad','range_m']); writer.writerows(beams)
result = {'kind':'synthetic educational example','truth':vars(truth),
          'estimate':vars(estimate),'score_m':residual,
          'position_error_m':math.hypot(estimate.x-truth.x, estimate.y-truth.y)}
(args.output/'result.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
