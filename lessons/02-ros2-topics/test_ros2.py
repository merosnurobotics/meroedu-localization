"""ROS 2 integration checks. Run after sourcing /opt/ros/humble/setup.bash."""
import math
import time
import unittest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from demo_scan import DemoScan
from localization_node import LocalizationNode


class TopicTest(unittest.TestCase):
    def test_scan_to_pose_and_stale_rejection(self):
        rclpy.init()
        source, localizer, observer = DemoScan(), LocalizationNode(), Node('meroedu_test_observer')
        poses, scores = [], []
        observer.create_subscription(PoseStamped, '/meroedu/pose', poses.append, 10)
        observer.create_subscription(Float64, '/meroedu/match_score', scores.append, 10)
        executor = SingleThreadedExecutor()
        for node in (source, localizer, observer):
            executor.add_node(node)
        try:
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline and not (poses and scores):
                executor.spin_once(timeout_sec=0.1)
            self.assertTrue(poses and scores, 'No real ROS messages received')
            pose = poses[-1]
            self.assertEqual(pose.header.frame_id, 'map')
            self.assertGreater(pose.header.stamp.sec, 0)
            self.assertAlmostEqual(pose.pose.position.x, 0.63, delta=0.03)
            self.assertAlmostEqual(pose.pose.position.y, -0.47, delta=0.03)
            self.assertAlmostEqual(pose.pose.orientation.z, math.sin(0.35 / 2), places=6)
            self.assertLess(scores[-1].data, 0.03)
            # Stop new scans and drain pending callbacks before checking no repeats.
            source.timer.cancel()
            for _ in range(15):
                executor.spin_once(timeout_sec=0.05)
            count = len(poses)
            stale = LaserScan()
            stale.header.stamp.sec = 1
            localizer.receive_scan(stale)
            localizer.process_latest_scan()
            for _ in range(15):
                executor.spin_once(timeout_sec=0.05)
            self.assertEqual(len(poses), count, 'Stale scan must not publish an old pose')
        finally:
            executor.shutdown()
            for node in (source, localizer, observer):
                node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
