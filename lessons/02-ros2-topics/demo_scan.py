"""SYNTHETIC, stationary scan for learning ROS topics. Not real LiDAR data."""
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '01-lidar'))
from wall_localizer import wall_range
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class DemoScan(Node):
    def __init__(self):
        super().__init__('meroedu_demo_scan')
        self.publisher = self.create_publisher(LaserScan, '/meroedu/scan', qos_profile_sensor_data)
        self.rng = random.Random(14)
        self.timer = self.create_timer(1.0, self.publish_scan)

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        msg.angle_min = -math.pi
        msg.angle_increment = 2 * math.pi / 48
        msg.angle_max = msg.angle_min + 47 * msg.angle_increment
        msg.scan_time = 1.0
        msg.range_min, msg.range_max = 0.05, 10.0
        msg.ranges = [wall_range(0.63, -0.47, 0.35 + msg.angle_min + i * msg.angle_increment)
                      + self.rng.gauss(0, 0.008) for i in range(48)]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = DemoScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
