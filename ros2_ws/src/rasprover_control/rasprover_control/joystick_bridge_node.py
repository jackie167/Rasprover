import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from rasprover_msgs.msg import GimbalCommand
from rasprover_msgs.msg import LightCommand
from rasprover_msgs.msg import MotionCommand


def apply_deadzone(value, deadzone):
    if abs(value) < deadzone:
        return 0.0
    return value


class JoystickTeleopNode(Node):
    """Maps joystick input from /joy to motion and gimbal UI command topics."""

    def __init__(self):
        super().__init__('joystick_bridge_node')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('motion_topic', '/ui/cmd/motion')
        self.declare_parameter('gimbal_topic', '/ui/cmd/gimbal')
        self.declare_parameter('lights_topic', '/ui/cmd/lights')
        self.declare_parameter('source', 'joystick')
        self.declare_parameter('deadman_button', -1)
        self.declare_parameter('estop_button', -1)
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 0)
        self.declare_parameter('pan_axis', 2)
        self.declare_parameter('tilt_axis', 3)
        self.declare_parameter('max_linear', 0.8)
        self.declare_parameter('max_angular', 0.8)
        self.declare_parameter('max_pan', 90.0)
        self.declare_parameter('max_tilt', 60.0)
        self.declare_parameter('pan_step_deg', 3.0)
        self.declare_parameter('tilt_step_deg', 3.0)
        self.declare_parameter('drive_scale_step', 0.1)
        self.declare_parameter('gimbal_scale_step', 0.2)
        self.declare_parameter('min_drive_scale', 0.3)
        self.declare_parameter('max_drive_scale', 1.5)
        self.declare_parameter('min_gimbal_scale', 0.4)
        self.declare_parameter('max_gimbal_scale', 3.0)
        self.declare_parameter('drive_faster_button', 6)
        self.declare_parameter('drive_slower_button', 8)
        self.declare_parameter('drive_slower_axis', 5)
        self.declare_parameter('gimbal_faster_button', 7)
        self.declare_parameter('gimbal_slower_button', 9)
        self.declare_parameter('gimbal_slower_axis', 4)
        self.declare_parameter('gimbal_speed', 200.0)
        self.declare_parameter('gimbal_accel', 10.0)
        self.declare_parameter('headlight_toggle_button', 3)
        self.declare_parameter('headlight_pwm', 255)
        self.declare_parameter('lights_timeout_ms', 86400000)
        self.declare_parameter('motion_timeout_ms', 250)
        self.declare_parameter('gimbal_timeout_ms', 250)
        self.declare_parameter('deadzone', 0.12)
        self.declare_parameter('invert_linear', True)
        self.declare_parameter('invert_angular', True)
        self.declare_parameter('invert_pan', False)
        self.declare_parameter('invert_tilt', True)

        self.source = self.get_parameter('source').get_parameter_value().string_value
        self.deadman_button = self.get_parameter('deadman_button').get_parameter_value().integer_value
        self.estop_button = self.get_parameter('estop_button').get_parameter_value().integer_value
        self.linear_axis = self.get_parameter('linear_axis').get_parameter_value().integer_value
        self.angular_axis = self.get_parameter('angular_axis').get_parameter_value().integer_value
        self.pan_axis = self.get_parameter('pan_axis').get_parameter_value().integer_value
        self.tilt_axis = self.get_parameter('tilt_axis').get_parameter_value().integer_value
        self.max_linear = self.get_parameter('max_linear').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular').get_parameter_value().double_value
        self.max_pan = self.get_parameter('max_pan').get_parameter_value().double_value
        self.max_tilt = self.get_parameter('max_tilt').get_parameter_value().double_value
        self.pan_step_deg = self.get_parameter('pan_step_deg').get_parameter_value().double_value
        self.tilt_step_deg = self.get_parameter('tilt_step_deg').get_parameter_value().double_value
        self.drive_scale_step = self.get_parameter('drive_scale_step').get_parameter_value().double_value
        self.gimbal_scale_step = self.get_parameter('gimbal_scale_step').get_parameter_value().double_value
        self.min_drive_scale = self.get_parameter('min_drive_scale').get_parameter_value().double_value
        self.max_drive_scale = self.get_parameter('max_drive_scale').get_parameter_value().double_value
        self.min_gimbal_scale = self.get_parameter('min_gimbal_scale').get_parameter_value().double_value
        self.max_gimbal_scale = self.get_parameter('max_gimbal_scale').get_parameter_value().double_value
        self.drive_faster_button = self.get_parameter('drive_faster_button').get_parameter_value().integer_value
        self.drive_slower_button = self.get_parameter('drive_slower_button').get_parameter_value().integer_value
        self.drive_slower_axis = self.get_parameter('drive_slower_axis').get_parameter_value().integer_value
        self.gimbal_faster_button = self.get_parameter('gimbal_faster_button').get_parameter_value().integer_value
        self.gimbal_slower_button = self.get_parameter('gimbal_slower_button').get_parameter_value().integer_value
        self.gimbal_slower_axis = self.get_parameter('gimbal_slower_axis').get_parameter_value().integer_value
        self.gimbal_speed = self.get_parameter('gimbal_speed').get_parameter_value().double_value
        self.gimbal_accel = self.get_parameter('gimbal_accel').get_parameter_value().double_value
        self.headlight_toggle_button = self.get_parameter('headlight_toggle_button').get_parameter_value().integer_value
        self.headlight_pwm = self.get_parameter('headlight_pwm').get_parameter_value().integer_value
        self.lights_timeout_ms = self.get_parameter('lights_timeout_ms').get_parameter_value().integer_value
        self.motion_timeout_ms = self.get_parameter('motion_timeout_ms').get_parameter_value().integer_value
        self.gimbal_timeout_ms = self.get_parameter('gimbal_timeout_ms').get_parameter_value().integer_value
        self.deadzone = self.get_parameter('deadzone').get_parameter_value().double_value
        self.invert_linear = self.get_parameter('invert_linear').get_parameter_value().bool_value
        self.invert_angular = self.get_parameter('invert_angular').get_parameter_value().bool_value
        self.invert_pan = self.get_parameter('invert_pan').get_parameter_value().bool_value
        self.invert_tilt = self.get_parameter('invert_tilt').get_parameter_value().bool_value

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        motion_topic = self.get_parameter('motion_topic').get_parameter_value().string_value
        gimbal_topic = self.get_parameter('gimbal_topic').get_parameter_value().string_value
        lights_topic = self.get_parameter('lights_topic').get_parameter_value().string_value

        self.motion_pub = self.create_publisher(MotionCommand, motion_topic, 10)
        self.gimbal_pub = self.create_publisher(GimbalCommand, gimbal_topic, 10)
        self.lights_pub = self.create_publisher(LightCommand, lights_topic, 10)
        self.joy_sub = self.create_subscription(Joy, joy_topic, self.handle_joy, 10)

        self.deadman_active = False
        self.last_motion = (0.0, 0.0)
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.drive_scale = 1.0
        self.gimbal_scale = 1.0
        self.last_drive_slower_pressed = False
        self.last_drive_faster_pressed = False
        self.last_gimbal_slower_pressed = False
        self.last_gimbal_faster_pressed = False
        self.last_headlight_toggle_pressed = False
        self.headlight_on = False

        self.get_logger().info(
            'joystick_bridge_node listening on %s -> motion=%s gimbal=%s deadman_button=%d'
            % (joy_topic, motion_topic, gimbal_topic, self.deadman_button)
        )

    @staticmethod
    def axis_value(msg, index):
        if 0 <= index < len(msg.axes):
            return float(msg.axes[index])
        return 0.0

    @staticmethod
    def button_pressed(msg, index):
        if index < 0:
            return False
        if 0 <= index < len(msg.buttons):
            return bool(msg.buttons[index])
        return False

    def trigger_pressed(self, msg, index):
        if index < 0 or index >= len(msg.axes):
            return False
        return float(msg.axes[index]) > 0.5

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def publish_motion(self, linear, angular, mode='joystick'):
        msg = MotionCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.source = self.source
        msg.linear = float(linear)
        msg.angular = float(angular)
        msg.priority = 100
        msg.mode = mode
        msg.timeout_ms = self.motion_timeout_ms
        msg.frame_id = 'base_link'
        self.motion_pub.publish(msg)

    def publish_gimbal(self, pan, tilt, mode='joystick'):
        msg = GimbalCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.source = self.source
        msg.pan = float(pan)
        msg.tilt = float(tilt)
        msg.speed = self.gimbal_speed
        msg.accel = self.gimbal_accel
        msg.mode = mode
        msg.timeout_ms = self.gimbal_timeout_ms
        self.gimbal_pub.publish(msg)

    def publish_lights(self, base_pwm, head_pwm, mode='joystick'):
        msg = LightCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.source = self.source
        msg.mode = mode
        msg.base_pwm = int(base_pwm)
        msg.head_pwm = int(head_pwm)
        msg.timeout_ms = self.lights_timeout_ms
        self.lights_pub.publish(msg)

    def handle_joy(self, msg):
        drive_faster_pressed = self.button_pressed(msg, self.drive_faster_button)
        drive_slower_pressed = self.button_pressed(msg, self.drive_slower_button) or self.trigger_pressed(
            msg, self.drive_slower_axis
        )
        gimbal_faster_pressed = self.button_pressed(msg, self.gimbal_faster_button)
        gimbal_slower_pressed = self.button_pressed(msg, self.gimbal_slower_button) or self.trigger_pressed(
            msg, self.gimbal_slower_axis
        )
        headlight_toggle_pressed = self.button_pressed(msg, self.headlight_toggle_button)

        if drive_slower_pressed and not self.last_drive_slower_pressed:
            self.drive_scale = self.clamp(
                self.drive_scale - self.drive_scale_step,
                self.min_drive_scale,
                self.max_drive_scale,
            )
            self.get_logger().info('drive scale %.2f' % self.drive_scale)
        if drive_faster_pressed and not self.last_drive_faster_pressed:
            self.drive_scale = self.clamp(
                self.drive_scale + self.drive_scale_step,
                self.min_drive_scale,
                self.max_drive_scale,
            )
            self.get_logger().info('drive scale %.2f' % self.drive_scale)
        if gimbal_slower_pressed and not self.last_gimbal_slower_pressed:
            self.gimbal_scale = self.clamp(
                self.gimbal_scale - self.gimbal_scale_step,
                self.min_gimbal_scale,
                self.max_gimbal_scale,
            )
            self.get_logger().info('gimbal scale %.2f' % self.gimbal_scale)
        if gimbal_faster_pressed and not self.last_gimbal_faster_pressed:
            self.gimbal_scale = self.clamp(
                self.gimbal_scale + self.gimbal_scale_step,
                self.min_gimbal_scale,
                self.max_gimbal_scale,
            )
            self.get_logger().info('gimbal scale %.2f' % self.gimbal_scale)

        self.last_drive_slower_pressed = drive_slower_pressed
        self.last_drive_faster_pressed = drive_faster_pressed
        self.last_gimbal_slower_pressed = gimbal_slower_pressed
        self.last_gimbal_faster_pressed = gimbal_faster_pressed

        if headlight_toggle_pressed and not self.last_headlight_toggle_pressed:
            self.headlight_on = not self.headlight_on
            self.publish_lights(0, self.headlight_pwm if self.headlight_on else 0, mode='joystick_headlight_toggle')
            self.get_logger().info('headlight %s' % ('on' if self.headlight_on else 'off'))
        self.last_headlight_toggle_pressed = headlight_toggle_pressed

        if self.button_pressed(msg, self.estop_button):
            self.publish_motion(0.0, 0.0, mode='joystick_estop')
            self.publish_gimbal(0.0, 0.0, mode='joystick_estop')
            self.last_motion = (0.0, 0.0)
            self.current_pan = 0.0
            self.current_tilt = 0.0
            self.deadman_active = False
            self.get_logger().warning('joystick estop button pressed')
            return

        deadman_pressed = True if self.deadman_button < 0 else self.button_pressed(msg, self.deadman_button)
        if not deadman_pressed:
            if self.deadman_active or any(abs(v) > 1e-3 for v in self.last_motion):
                self.publish_motion(0.0, 0.0, mode='joystick_release')
                self.last_motion = (0.0, 0.0)
            self.deadman_active = False
            return

        self.deadman_active = True

        linear_input = apply_deadzone(self.axis_value(msg, self.linear_axis), self.deadzone)
        angular_input = apply_deadzone(self.axis_value(msg, self.angular_axis), self.deadzone)
        pan_input = apply_deadzone(self.axis_value(msg, self.pan_axis), self.deadzone)
        tilt_input = apply_deadzone(self.axis_value(msg, self.tilt_axis), self.deadzone)

        if self.invert_linear:
            linear_input *= -1.0
        if self.invert_angular:
            angular_input *= -1.0
        if self.invert_pan:
            pan_input *= -1.0
        if self.invert_tilt:
            tilt_input *= -1.0

        linear_limit = self.max_linear * self.drive_scale
        angular_limit = self.max_angular * self.drive_scale
        linear = max(-linear_limit, min(linear_limit, linear_input * linear_limit))
        angular = max(-angular_limit, min(angular_limit, angular_input * angular_limit))

        motion = (linear, angular)
        if any(not math.isclose(a, b, abs_tol=1e-3) for a, b in zip(motion, self.last_motion)) or any(
            abs(v) > 1e-3 for v in motion
        ):
            self.publish_motion(linear, angular)
            self.last_motion = motion

        pan_changed = abs(pan_input) > 1e-3
        tilt_changed = abs(tilt_input) > 1e-3
        if pan_changed or tilt_changed:
            pan_step = self.pan_step_deg * self.gimbal_scale
            tilt_step = self.tilt_step_deg * self.gimbal_scale
            self.current_pan = max(-self.max_pan, min(self.max_pan, self.current_pan + pan_input * pan_step))
            self.current_tilt = max(
                -self.max_tilt,
                min(self.max_tilt, self.current_tilt + tilt_input * tilt_step),
            )
            self.publish_gimbal(self.current_pan, self.current_tilt)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
