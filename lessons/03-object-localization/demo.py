"""Run a tiny synthetic example, or process a captured, aligned JSON frame."""
import argparse
import json
import math
from pathlib import Path
from object_localizer import Intrinsics, Mount, RobotPose, locate_object


def synthetic_frame():
    # One original pixel grid, not a stitched image. These are teaching values.
    mask = [[int(9 <= u <= 13 and 6 <= v <= 10) for u in range(17)] for v in range(13)]
    depth = [[1500 if mask[v][u] else 3500 for u in range(17)] for v in range(13)]
    depth[10][11] = 0  # Missing return: excluded instead of treated as zero meters.
    return {'kind': 'synthetic educational input', 'label': 'apple_cube', 'mask': mask,
            'depth': depth, 'depth_scale_m': 0.001,
            'intrinsics': {'fx': 80.0, 'fy': 80.0, 'cx': 8.0, 'cy': 6.0},
            'mount': {'forward_m': 0.2, 'left_m': 0.0, 'height_m': 0.6,
                      'pitch_down_rad': math.radians(15)},
            'robot_pose': {'x': 0.5, 'y': -0.5, 'yaw': math.pi/2}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, help='Aligned mask/depth and calibrated pose JSON')
    parser.add_argument('--output', type=Path, default=Path('output'))
    args = parser.parse_args()
    frame = json.loads(args.input.read_text()) if args.input else synthetic_frame()
    result = locate_object(frame['mask'], frame['depth'], Intrinsics(**frame['intrinsics']),
                           Mount(**frame['mount']), RobotPose(**frame['robot_pose']),
                           frame['depth_scale_m'])
    result['label'] = frame.get('label', 'object')
    result['input_kind'] = frame.get('kind', 'user-provided input; not independently verified')
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'input.json').write_text(json.dumps(frame, indent=2))
    (args.output/'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
