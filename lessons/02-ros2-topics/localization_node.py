"""Publish the lesson-01 estimate; no motor commands, IMU or TF integration."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '01-lidar'))
from wall_localizer import grid_search
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('meroedu_localization')
        self.declare_parameter('scan_topic', '/meroedu/scan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('known_yaw', 0.35)
        self.declare_parameter('max_scan_age_sec', 1.0)
        self.declare_parameter('max_score_m', 0.15)
        self.latest_scan = None
        # Keep the newest scan instead of building a backlog.
        scan_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
        self.scan_sub = self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.receive_scan, scan_qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/meroedu/pose', 10)
        self.score_pub = self.create_publisher(Float64, '/meroedu/match_score', 10)
        self.timer = self.create_timer(0.5, self.process_latest_scan)

    def receive_scan(self, msg):
        self.latest_scan = msg

    def process_latest_scan(self):
        scan, self.latest_scan = self.latest_scan, None
        if scan is None:
            return
        stamp_ns = scan.header.stamp.sec * 10**9 + scan.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 10**9
        if stamp_ns <= 0 or age < -0.1 or age > self.get_parameter('max_scan_age_sec').value:
            self.get_logger().warning('Ignored scan: missing, future or stale timestamp')
            return
        yaw = float(self.get_parameter('known_yaw').value)
        if (not all(math.isfinite(value) for value in (yaw, scan.angle_min, scan.angle_increment,
                                                    scan.range_min, scan.range_max))
                or scan.angle_increment <= 0 or not (0 <= scan.range_min < scan.range_max)):
            self.get_logger().warning('Ignored scan: invalid yaw or angle increment')
            return
        beams = [(scan.angle_min + i * scan.angle_increment, float(distance))
                 for i, distance in enumerate(scan.ranges)
                 if math.isfinite(distance) and scan.range_min < distance < scan.range_max]
        # Bound the cost: preserve angular coverage, at most 48 beams.
        if len(beams) > 48:
            beams = [beams[round(i * (len(beams) - 1) / 47)] for i in range(48)]
        try:
            pose, score_m = grid_search(beams, yaw)
        except ValueError:
            self.get_logger().warning('Ignored scan: fewer than 8 usable beams')
            return
        if score_m > self.get_parameter('max_score_m').value:
            self.get_logger().warning('Ignored estimate: matching residual too large')
            return
        msg = PoseStamped()
        msg.header.stamp = scan.header.stamp  # Observation time, not computation finish time.
        msg.header.frame_id = self.get_parameter('map_frame').value
        msg.pose.position.x, msg.pose.position.y = pose.x, pose.y
        msg.pose.orientation.z = math.sin(pose.yaw / 2)
        msg.pose.orientation.w = math.cos(pose.yaw / 2)
        self.pose_pub.publish(msg)
        score = Float64()
        score.data = score_m
        self.score_pub.publish(score)


def main():
    rclpy.init()
    node = LocalizationNode()
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
