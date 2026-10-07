import math
import unittest
from demo import synthetic_frame
from object_localizer import (Intrinsics, Mount, RobotPose, back_project, camera_to_robot,
                              robot_to_map, masked_depth_median, locate_object)


class GeometryTest(unittest.TestCase):
    def test_optical_and_robot_axes(self):
        point = back_project(420, 240, 2, Intrinsics(500, 500, 320, 240))
        self.assertEqual(point, (0.4, 0, 2))
        self.assertEqual(camera_to_robot(point, Mount()), (2, -0.4, 0))

    def test_map_rotation_and_translation(self):
        x, y, z = robot_to_map((2, -0.4, 0.3), RobotPose(1, 2, math.pi/2))
        self.assertAlmostEqual(x, 1.4)
        self.assertAlmostEqual(y, 4)
        self.assertAlmostEqual(z, 0.3)

    def test_pitch_and_height(self):
        point = camera_to_robot((0, 0, 2), Mount(height_m=1, pitch_down_rad=math.pi/6))
        self.assertAlmostEqual(point[0], math.sqrt(3))
        self.assertAlmostEqual(point[2], 0)

    def test_exclude_background_holes_and_outlier(self):
        depth = [[9000, 0, 9000], [9000, 1500, 9000], [9000, 1500, 2000]]
        mask = [[0, 1, 0], [0, 1, 0], [0, 1, 1]]
        self.assertEqual(masked_depth_median(depth, mask, 1, 2), 1.5)
        with self.assertRaises(ValueError):
            masked_depth_median([[0]*3 for _ in range(3)], mask, 1, 2)

    def test_grid_and_intrinsics_validation(self):
        with self.assertRaises(ValueError):
            masked_depth_median([[1000]], [[1, 1]], 0, 0)
        with self.assertRaises(ValueError):
            back_project(1, 1, 1, Intrinsics(0, 1, 0, 0))

    def test_full_pipeline(self):
        f = synthetic_frame()
        r = locate_object(f['mask'], f['depth'], Intrinsics(**f['intrinsics']),
                          Mount(**f['mount']), RobotPose(**f['robot_pose']))
        self.assertEqual(r['pixel_uv'], [11, 10])
        self.assertEqual(r['depth_m'], 1.5)
        self.assertAlmostEqual(r['map_xyz_m'][0], 0.55625)
        self.assertAlmostEqual(r['map_xyz_m'][1], 1.129477311050913, places=6)


if __name__ == '__main__':
    unittest.main()
