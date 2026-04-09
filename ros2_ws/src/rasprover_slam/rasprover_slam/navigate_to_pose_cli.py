import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


class NavigateToPoseClient(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_cli')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def send_goal(self, x, y, yaw, frame_id='map'):
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose action server not available')
            return 2

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = frame_id
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        qz, qw = quaternion_from_yaw(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            'sending goal frame=%s x=%.3f y=%.3f yaw=%.3f rad'
            % (frame_id, x, y, yaw)
        )
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('goal rejected')
            return 3

        self.get_logger().info('goal accepted, waiting for result')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            self.get_logger().error('goal finished without result')
            return 4

        status = int(result.status)
        self.get_logger().info('goal finished with status=%d' % status)
        return 0 if status == 4 else 5


def parse_args(argv):
    parser = argparse.ArgumentParser(description='Send a NavigateToPose goal.')
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
    node = NavigateToPoseClient()
    try:
        code = node.send_goal(cli_args.x, cli_args.y, yaw, cli_args.frame_id)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
