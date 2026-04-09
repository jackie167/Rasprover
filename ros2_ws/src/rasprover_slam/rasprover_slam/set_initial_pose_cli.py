import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('set_initial_pose_cli')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def publish_initial_pose(self, x, y, yaw, frame_id='map'):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qz, qw = quaternion_from_yaw(yaw)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068

        # Give discovery a moment so AMCL subscribes before we publish.
        for _ in range(20):
            if self.pub.get_subscription_count() > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        # Publish repeatedly for a few seconds to survive late discovery.
        for _ in range(30):
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(
            'published initial pose frame=%s x=%.3f y=%.3f yaw=%.3f rad subs=%d'
            % (frame_id, x, y, yaw, self.pub.get_subscription_count())
        )


def parse_args(argv):
    parser = argparse.ArgumentParser(description='Publish AMCL initial pose.')
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw', type=float, default=0.0, help='Yaw in radians')
    parser.add_argument('--yaw-deg', type=float, default=None, help='Yaw in degrees')
    parser.add_argument('--frame-id', type=str, default='map')
    return parser.parse_args(argv)


def main(args=None):
    cli_args = parse_args(sys.argv[1:] if args is None else args)
    yaw = math.radians(cli_args.yaw_deg) if cli_args.yaw_deg is not None else cli_args.yaw
    rclpy.init(args=None)
    node = InitialPosePublisher()
    try:
        node.publish_initial_pose(cli_args.x, cli_args.y, yaw, cli_args.frame_id)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
