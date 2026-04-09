import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from rasprover_msgs.msg import CvControlIntent


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class NavCmdVelBridgeNode(Node):
    """Bridge Nav2 cmd_vel into the existing typed motion-intent lane."""

    def __init__(self):
        super().__init__('nav_cmd_vel_bridge_node')

        self.declare_parameter('input_topic', '/cmd_vel_nav')
        self.declare_parameter('output_topic', '/cv/control_intent')
        self.declare_parameter('source', 'nav2')
        self.declare_parameter('mode', 'nav')
        self.declare_parameter('timeout_ms', 250)
        self.declare_parameter('max_linear', 0.10)
        self.declare_parameter('max_angular', 0.30)
        self.declare_parameter('publish_zero_on_idle', True)
        self.declare_parameter('deadband_linear', 0.001)
        self.declare_parameter('deadband_angular', 0.001)
        self.declare_parameter('min_linear_floor', 0.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.source_name = self.get_parameter('source').value
        self.mode_name = self.get_parameter('mode').value
        self.timeout_ms = int(self.get_parameter('timeout_ms').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.publish_zero_on_idle = bool(self.get_parameter('publish_zero_on_idle').value)
        self.deadband_linear = float(self.get_parameter('deadband_linear').value)
        self.deadband_angular = float(self.get_parameter('deadband_angular').value)
        self.min_linear_floor = float(self.get_parameter('min_linear_floor').value)

        self.cmd_sub = self.create_subscription(Twist, self.input_topic, self.handle_cmd_vel, 20)
        self.intent_pub = self.create_publisher(CvControlIntent, self.output_topic, 20)

        self.get_logger().info(
            'nav_cmd_vel_bridge_node input=%s output=%s max_linear=%.3f max_angular=%.3f timeout_ms=%d'
            % (
                self.input_topic,
                self.output_topic,
                self.max_linear,
                self.max_angular,
                self.timeout_ms,
            )
        )

    def handle_cmd_vel(self, msg):
        linear = clamp(float(msg.linear.x), -self.max_linear, self.max_linear)
        angular = clamp(float(msg.angular.z), -self.max_angular, self.max_angular)

        if math.fabs(linear) < self.deadband_linear:
            linear = 0.0
        if math.fabs(angular) < self.deadband_angular:
            angular = 0.0
        if linear != 0.0 and math.fabs(linear) < self.min_linear_floor:
            linear = math.copysign(self.min_linear_floor, linear)

        if not self.publish_zero_on_idle and linear == 0.0 and angular == 0.0:
            return

        intent = CvControlIntent()
        intent.stamp = self.get_clock().now().to_msg()
        intent.source = self.source_name
        intent.kind = 'motion'
        intent.mode = self.mode_name
        intent.target_present = True
        intent.linear = linear
        intent.angular = angular
        intent.timeout_ms = self.timeout_ms
        self.intent_pub.publish(intent)


def main(args=None):
    rclpy.init(args=args)
    node = NavCmdVelBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
