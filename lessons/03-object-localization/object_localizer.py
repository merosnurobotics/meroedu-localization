"""Mask + aligned depth -> visible surface point -> robot -> map.
Adapted from ddonggae's depth path (Team 14, MIT), using ROS axis names.
Pinhole model: input pixels must be rectified, with matching intrinsics.
"""
import math
from dataclasses import dataclass
from statistics import median


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class Mount:
    forward_m: float = 0.0
    left_m: float = 0.0
    height_m: float = 0.0
    pitch_down_rad: float = 0.0


@dataclass(frozen=True)
class RobotPose:
    x: float
    y: float
    yaw: float


def image_shape(image):
    if not image or not image[0] or any(len(row) != len(image[0]) for row in image):
        raise ValueError('Expected a nonempty rectangular image')
    return len(image), len(image[0])


def mask_bottom_pixel(mask):
    """Mean u in the bottom two mask rows, and the lowest v, like ddonggae."""
    image_shape(mask)
    pixels = [(u, v) for v, row in enumerate(mask) for u, value in enumerate(row) if value]
    if not pixels:
        raise ValueError('Empty object mask')
    bottom = max(v for _, v in pixels)
    us = [u for u, v in pixels if v >= bottom - 1]
    return sum(us) / len(us), float(bottom)


def masked_depth_median(depth, mask, u, v, scale_m=0.001, radius=2,
                        min_m=0.1, max_m=5.0, min_samples=3):
    """Exclude invalid pixels AND background outside the object mask."""
    h, w = image_shape(depth)
    if image_shape(mask) != (h, w):
        raise ValueError('Mask and aligned depth must share the original pixel grid')
    if not math.isfinite(scale_m) or scale_m <= 0:
        raise ValueError('Depth scale must be positive meters per stored unit')
    if radius < 0 or min_samples < 1 or not (0 < min_m < max_m):
        raise ValueError('Invalid depth sampling settings')
    u0, v0 = round(u), round(v)
    samples = []
    for y in range(max(0, v0-radius), min(h, v0+radius+1)):
        for x in range(max(0, u0-radius), min(w, u0+radius+1)):
            value = float(depth[y][x]) * scale_m
            if mask[y][x] and math.isfinite(value) and min_m < value < max_m:
                samples.append(value)
    if len(samples) < min_samples:
        raise ValueError('Not enough valid masked depth; do not invent a position')
    return median(samples)


def back_project(u, v, depth_m, intr):
    values = (u, v, depth_m, intr.fx, intr.fy, intr.cx, intr.cy)
    if not all(math.isfinite(value) for value in values) or min(intr.fx, intr.fy, depth_m) <= 0:
        raise ValueError('Invalid depth or intrinsics')
    # Optical axes: x right, y down, z forward. Depth is z, not Euclidean range.
    return ((u-intr.cx)/intr.fx * depth_m, (v-intr.cy)/intr.fy * depth_m, depth_m)


def camera_to_robot(point, mount):
    """Aligned yaw/roll, pitch only; robot axes: x forward, y left, z up."""
    xc, yc, zc = point
    c, s = math.cos(mount.pitch_down_rad), math.sin(mount.pitch_down_rad)
    values = (*point, mount.forward_m, mount.left_m, mount.height_m, mount.pitch_down_rad)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Nonfinite camera point or mount')
    return (mount.forward_m + c*zc - s*yc,
            mount.left_m - xc,
            mount.height_m - s*zc - c*yc)


def robot_to_map(point, pose):
    if not all(math.isfinite(value) for value in (*point, pose.x, pose.y, pose.yaw)):
        raise ValueError('Nonfinite point or robot pose')
    x, y, z = point
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    # Robot is on a level floor, map z=0 at robot base origin.
    return (pose.x + c*x - s*y, pose.y + s*x + c*y, z)


def locate_object(mask, depth, intr, mount, pose, depth_scale_m=0.001):
    if image_shape(mask) != image_shape(depth):
        raise ValueError('Mask and depth pixel grids must match')
    u, v = mask_bottom_pixel(mask)
    z = masked_depth_median(depth, mask, u, v, depth_scale_m)
    camera = back_project(u, v, z, intr)
    robot = camera_to_robot(camera, mount)
    world = robot_to_map(robot, pose)
    return {'pixel_uv': [u, v], 'depth_m': z, 'camera_xyz_m': list(camera),
            'robot_xyz_m': list(robot), 'map_xyz_m': list(world),
            'point_kind': 'visible lower surface estimate, not object center or ground truth'}
