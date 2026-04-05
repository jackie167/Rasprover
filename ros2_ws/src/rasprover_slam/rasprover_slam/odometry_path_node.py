from collections import deque
from math import hypot

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.node import Node


class OdometryPathNode(Node):
    def __init__(self):
        super().__init__('odometry_path_node')
        self.odom_topic = self.declare_parameter('odom_topic', '/odometry/filtered').value
        self.path_topic = self.declare_parameter('path_topic', '/odom_path').value
        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        self.min_translation = float(self.declare_parameter('min_translation', 0.02).value)
        self.min_rotation = float(self.declare_parameter('min_rotation', 0.03).value)
        self.max_poses = int(self.declare_parameter('max_poses', 5000).value)

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._on_odom, 20)

        self._poses = deque(maxlen=self.max_poses)
        self._last_position = None
        self._last_orientation = None

        self.get_logger().info(
            f'odometry_path_node odom_topic={self.odom_topic} path_topic={self.path_topic} '
            f'frame_id={self.frame_id} max_poses={self.max_poses}'
        )

    def _orientation_delta(self, orientation):
        if self._last_orientation is None:
            return None
        return abs(orientation.z - self._last_orientation.z) + abs(orientation.w - self._last_orientation.w)

    def _should_append(self, pose):
        position = pose.position
        orientation = pose.orientation
        if self._last_position is None:
            return True

        distance = hypot(position.x - self._last_position.x, position.y - self._last_position.y)
        rotation_delta = self._orientation_delta(orientation)
        return distance >= self.min_translation or (rotation_delta is not None and rotation_delta >= self.min_rotation)

    def _on_odom(self, msg):
        pose = msg.pose.pose
        if not self._should_append(pose):
            return

        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = self.frame_id or msg.header.frame_id or 'odom'
        pose_msg.pose = pose

        self._poses.append(pose_msg)
        self._last_position = pose.position
        self._last_orientation = pose.orientation

        path = Path()
        path.header.stamp = msg.header.stamp
        path.header.frame_id = pose_msg.header.frame_id
        path.poses = list(self._poses)
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPathNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
