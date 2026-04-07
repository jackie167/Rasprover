import math

import rclpy
from rclpy.node import Node

from rasprover_msgs.msg import GimbalCommand
from rasprover_msgs.msg import LightCommand
from rasprover_msgs.msg import MotionCommand
from rasprover_msgs.msg import RawRobotFeedback
from rasprover_msgs.msg import RobotFeedback
from rasprover_msgs.msg import ServoSetupCommand

from .adapters import RobotFeedbackAdapter
from .adapters import RobotHardwareAdapter


class RobotBaseNode(Node):
    """Hardware and protocol boundary for the robot base."""

    def __init__(self):
        super().__init__('robot_base_node')

        self.declare_parameter('serial_port', '')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('left_drive_scale', 1.0)
        self.declare_parameter('right_drive_scale', 1.0)
        self.declare_parameter('feedback_period_sec', 0.05)
        self.declare_parameter('feedback_wheel_separation_m', 0.52)
        self.declare_parameter('feedback_wheel_yaw_scale', 1.0)
        self.declare_parameter('feedback_gyro_z_scale', 1.0)
        self.declare_parameter('swap_feedback_wheels', False)
        self.declare_parameter('feedback_motion_epsilon', 1e-6)
        self.declare_parameter('straight_controller_enabled', True)
        self.declare_parameter('straight_controller_forward_only', True)
        self.declare_parameter('straight_controller_linear_min', 0.10)
        self.declare_parameter('straight_controller_angular_window', 0.05)
        self.declare_parameter('straight_controller_heading_gain', 0.9)
        self.declare_parameter('straight_controller_integral_gain', 0.12)
        self.declare_parameter('straight_controller_wheel_balance_gain', 0.8)
        self.declare_parameter('straight_controller_gyro_gain', 0.2)
        self.declare_parameter('straight_controller_integral_limit', 0.3)
        self.declare_parameter('straight_controller_max_correction', 0.12)
        self.declare_parameter('straight_controller_debug_enabled', True)
        self.declare_parameter('straight_controller_debug_period_sec', 0.3)
        self.declare_parameter('slam_debug_enabled', True)
        self.declare_parameter('slam_debug_period_sec', 0.2)
        self.declare_parameter('slam_debug_motion_threshold', 0.0005)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value or None
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        left_drive_scale = self.get_parameter('left_drive_scale').get_parameter_value().double_value
        right_drive_scale = self.get_parameter('right_drive_scale').get_parameter_value().double_value
        feedback_period = self.get_parameter('feedback_period_sec').get_parameter_value().double_value
        self.feedback_wheel_separation = (
            self.get_parameter('feedback_wheel_separation_m').get_parameter_value().double_value
        )
        self.feedback_wheel_yaw_scale = (
            self.get_parameter('feedback_wheel_yaw_scale').get_parameter_value().double_value
        )
        self.feedback_gyro_z_scale = self.get_parameter('feedback_gyro_z_scale').get_parameter_value().double_value
        self.swap_feedback_wheels = (
            self.get_parameter('swap_feedback_wheels').get_parameter_value().bool_value
        )
        self.feedback_motion_epsilon = (
            self.get_parameter('feedback_motion_epsilon').get_parameter_value().double_value
        )
        self.straight_controller_enabled = (
            self.get_parameter('straight_controller_enabled').get_parameter_value().bool_value
        )
        self.straight_controller_forward_only = (
            self.get_parameter('straight_controller_forward_only').get_parameter_value().bool_value
        )
        self.straight_linear_min = (
            self.get_parameter('straight_controller_linear_min').get_parameter_value().double_value
        )
        self.straight_angular_window = (
            self.get_parameter('straight_controller_angular_window').get_parameter_value().double_value
        )
        self.straight_heading_gain = (
            self.get_parameter('straight_controller_heading_gain').get_parameter_value().double_value
        )
        self.straight_integral_gain = (
            self.get_parameter('straight_controller_integral_gain').get_parameter_value().double_value
        )
        self.straight_wheel_balance_gain = (
            self.get_parameter('straight_controller_wheel_balance_gain').get_parameter_value().double_value
        )
        self.straight_gyro_gain = (
            self.get_parameter('straight_controller_gyro_gain').get_parameter_value().double_value
        )
        self.straight_integral_limit = (
            self.get_parameter('straight_controller_integral_limit').get_parameter_value().double_value
        )
        self.straight_max_correction = (
            self.get_parameter('straight_controller_max_correction').get_parameter_value().double_value
        )
        self.straight_debug_enabled = (
            self.get_parameter('straight_controller_debug_enabled').get_parameter_value().bool_value
        )
        self.straight_debug_period = (
            self.get_parameter('straight_controller_debug_period_sec').get_parameter_value().double_value
        )
        self.slam_debug_enabled = self.get_parameter('slam_debug_enabled').get_parameter_value().bool_value
        self.slam_debug_period = self.get_parameter('slam_debug_period_sec').get_parameter_value().double_value
        self.slam_debug_motion_threshold = (
            self.get_parameter('slam_debug_motion_threshold').get_parameter_value().double_value
        )

        self.hardware = RobotHardwareAdapter(
            port=serial_port,
            baud=baud,
            left_drive_scale=left_drive_scale,
            right_drive_scale=right_drive_scale,
        )
        self.feedback_adapter = RobotFeedbackAdapter()
        self.last_feedback_time = None
        self.last_odl = None
        self.last_odr = None
        self.last_slam_debug_time = 0.0
        self.last_feedback_dt = 0.0
        self.last_feedback_dleft = 0.0
        self.last_feedback_dright = 0.0
        self.feedback_encoder_heading = 0.0
        self.feedback_wheel_balance_error = 0.0
        self.feedback_gyro_z = 0.0
        self.feedback_ready = False
        self.straight_control_active = False
        self.straight_target_heading = 0.0
        self.straight_heading_integral = 0.0
        self.straight_motion_sign = 0
        self.straight_motion_source = ''
        self.straight_motion_mode = ''
        self.last_straight_debug_time = 0.0

        self.motion_sub = self.create_subscription(
            MotionCommand,
            '/robot/cmd/motion',
            self.handle_motion_command,
            10,
        )
        self.gimbal_sub = self.create_subscription(
            GimbalCommand,
            '/robot/cmd/gimbal',
            self.handle_gimbal_command,
            10,
        )
        self.light_sub = self.create_subscription(
            LightCommand,
            '/robot/cmd/lights',
            self.handle_light_command,
            10,
        )
        self.servo_setup_sub = self.create_subscription(
            ServoSetupCommand,
            '/robot/cmd/servo_setup',
            self.handle_servo_setup_command,
            10,
        )
        self.feedback_pub = self.create_publisher(RobotFeedback, '/robot/state/feedback', 10)
        self.feedback_raw_pub = self.create_publisher(RawRobotFeedback, '/robot/state/feedback_raw', 10)
        self.feedback_timer = self.create_timer(feedback_period, self.publish_feedback)

        active_port = serial_port or 'auto'
        self.get_logger().info(
            'robot_base_node started on serial_port=%s baud=%d left_drive_scale=%.3f right_drive_scale=%.3f '
            'straight_controller=%s wheel_sep=%.3f yaw_scale=%.3f swap_feedback_wheels=%s'
            % (
                active_port,
                baud,
                left_drive_scale,
                right_drive_scale,
                str(self.straight_controller_enabled).lower(),
                self.feedback_wheel_separation,
                self.feedback_wheel_yaw_scale,
                str(self.swap_feedback_wheels).lower(),
            )
        )

    @staticmethod
    def _feedback_float(feedback, key):
        try:
            return float(feedback[key])
        except (KeyError, TypeError, ValueError):
            return None

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def _normalize_angle(value):
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value

    def reset_straight_controller(self):
        self.straight_control_active = False
        self.straight_target_heading = self.feedback_encoder_heading
        self.straight_heading_integral = 0.0
        self.straight_motion_sign = 0
        self.straight_motion_source = ''
        self.straight_motion_mode = ''

    def should_apply_straight_controller(self, linear, angular):
        if not self.straight_controller_enabled or not self.feedback_ready:
            return False
        if self.straight_controller_forward_only and linear <= 0.0:
            return False
        if abs(linear) < self.straight_linear_min:
            return False
        if abs(angular) > self.straight_angular_window:
            return False
        return True

    def apply_straight_controller(self, msg):
        linear = float(msg.linear)
        angular = float(msg.angular)
        if not self.should_apply_straight_controller(linear, angular):
            self.reset_straight_controller()
            return angular, 0.0

        motion_sign = 1 if linear >= 0.0 else -1
        if (
            not self.straight_control_active
            or motion_sign != self.straight_motion_sign
            or msg.source != self.straight_motion_source
            or msg.mode != self.straight_motion_mode
        ):
            self.straight_control_active = True
            self.straight_target_heading = self.feedback_encoder_heading
            self.straight_heading_integral = 0.0
            self.straight_motion_sign = motion_sign
            self.straight_motion_source = msg.source
            self.straight_motion_mode = msg.mode

        heading_error = self._normalize_angle(self.feedback_encoder_heading - self.straight_target_heading)
        self.straight_heading_integral += heading_error * max(self.last_feedback_dt, 0.0)
        self.straight_heading_integral = self._clamp(
            self.straight_heading_integral,
            -self.straight_integral_limit,
            self.straight_integral_limit,
        )

        raw_correction = (
            self.straight_heading_gain * heading_error
            + self.straight_integral_gain * self.straight_heading_integral
            + self.straight_wheel_balance_gain * self.feedback_wheel_balance_error
            + self.straight_gyro_gain * self.feedback_gyro_z
        )
        correction = self._clamp(raw_correction, -self.straight_max_correction, self.straight_max_correction)
        corrected_angular = self._clamp(angular - correction, -1.0, 1.0)

        now_seconds = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if self.straight_debug_enabled and (now_seconds - self.last_straight_debug_time) >= self.straight_debug_period:
            self.last_straight_debug_time = now_seconds
            self.get_logger().info(
                'straight_ctrl heading_err=%.4f balance_err=%.4f gz=%.4f integral=%.4f correction=%.4f'
                % (
                    heading_error,
                    self.feedback_wheel_balance_error,
                    self.feedback_gyro_z,
                    self.straight_heading_integral,
                    correction,
                )
            )

        return corrected_angular, correction

    def handle_motion_command(self, msg):
        corrected_angular, correction = self.apply_straight_controller(msg)
        left, right = self.hardware.send_motion(msg.linear, corrected_angular)
        if abs(correction) > 1e-6:
            self.get_logger().info(
                'motion source=%s linear=%.3f angular_in=%.3f angular_out=%.3f correction=%.3f left=%.3f right=%.3f'
                % (msg.source, msg.linear, msg.angular, corrected_angular, correction, left, right)
            )
        else:
            self.get_logger().info(
                'motion source=%s linear=%.3f angular=%.3f left=%.3f right=%.3f'
                % (msg.source, msg.linear, corrected_angular, left, right)
            )

    def handle_gimbal_command(self, msg):
        pan, tilt, speed, accel = self.hardware.send_gimbal(msg.pan, msg.tilt, msg.speed, msg.accel)
        self.get_logger().info(
            f'gimbal source={msg.source} pan={pan:.3f} tilt={tilt:.3f} speed={speed:.3f} accel={accel:.3f}'
        )

    def handle_light_command(self, msg):
        base_pwm, head_pwm = self.hardware.send_lights(msg.base_pwm, msg.head_pwm)
        self.get_logger().info(
            f'lights source={msg.source} mode={msg.mode} base_pwm={base_pwm} head_pwm={head_pwm}'
        )

    def handle_servo_setup_command(self, msg):
        action = self.hardware.send_servo_setup(
            msg.action,
            old_id=msg.old_id,
            new_id=msg.new_id,
            servo_id=msg.servo_id,
            status=msg.status,
        )
        self.get_logger().info(
            f'servo_setup source={msg.source} action={action} old_id={msg.old_id} new_id={msg.new_id} servo_id={msg.servo_id} status={msg.status}'
        )

    def update_feedback_control_state(self, feedback):
        if not isinstance(feedback, dict) or feedback.get('T') != 1001:
            return

        odl = self._feedback_float(feedback, 'odl')
        odr = self._feedback_float(feedback, 'odr')
        if odl is None or odr is None:
            return

        gz = self._feedback_float(feedback, 'gz')
        if gz is None:
            gz = 0.0

        now_seconds = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if self.last_feedback_time is None or self.last_odl is None or self.last_odr is None:
            self.last_feedback_time = now_seconds
            self.last_odl = odl
            self.last_odr = odr
            self.feedback_gyro_z = gz * self.feedback_gyro_z_scale
            return

        dt = max(0.0, now_seconds - self.last_feedback_time)
        d_left = odl - self.last_odl
        d_right = odr - self.last_odr

        if self.swap_feedback_wheels:
            d_left, d_right = d_right, d_left

        self.last_feedback_time = now_seconds
        self.last_odl = odl
        self.last_odr = odr
        self.last_feedback_dt = dt
        self.last_feedback_dleft = d_left
        self.last_feedback_dright = d_right
        self.feedback_gyro_z = gz * self.feedback_gyro_z_scale

        if dt <= 0.0:
            return

        encoder_dtheta = (d_right - d_left) / max(self.feedback_wheel_separation, self.feedback_motion_epsilon)
        encoder_dtheta *= self.feedback_wheel_yaw_scale
        self.feedback_encoder_heading = self._normalize_angle(self.feedback_encoder_heading + encoder_dtheta)

        total_motion = abs(d_left) + abs(d_right)
        if total_motion > self.feedback_motion_epsilon:
            self.feedback_wheel_balance_error = (d_right - d_left) / total_motion
        else:
            self.feedback_wheel_balance_error = 0.0

        self.feedback_ready = True

    def publish_feedback(self):
        feedback = self.hardware.get_feedback()
        self.update_feedback_control_state(feedback)
        stamp = self.get_clock().now().to_msg()
        raw_msg = self.feedback_adapter.raw_feedback_msg(stamp, feedback)
        feedback_msg = self.feedback_adapter.robot_feedback_msg(stamp, feedback)
        self.feedback_raw_pub.publish(raw_msg)
        self.feedback_pub.publish(feedback_msg)
        self.publish_slam_debug(feedback)

    def publish_slam_debug(self, feedback):
        if not self.slam_debug_enabled or not isinstance(feedback, dict):
            return
        if feedback.get('T') != 1001:
            return

        odl = self._feedback_float(feedback, 'odl')
        odr = self._feedback_float(feedback, 'odr')
        gx = self._feedback_float(feedback, 'gx')
        gy = self._feedback_float(feedback, 'gy')
        gz = self._feedback_float(feedback, 'gz')
        ax = self._feedback_float(feedback, 'ax')
        ay = self._feedback_float(feedback, 'ay')
        az = self._feedback_float(feedback, 'az')
        mx = self._feedback_float(feedback, 'mx')
        my = self._feedback_float(feedback, 'my')
        mz = self._feedback_float(feedback, 'mz')

        required = (odl, odr, gx, gy, gz, ax, ay, az, mx, my, mz)
        if any(value is None for value in required):
            return

        dt = self.last_feedback_dt
        dodl = self.last_feedback_dleft
        dodr = self.last_feedback_dright
        moving = (
            abs(dodl) > self.slam_debug_motion_threshold
            or abs(dodr) > self.slam_debug_motion_threshold
            or abs(self.feedback_gyro_z) > 0.0
        )
        if not moving:
            return

        now_seconds = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if (now_seconds - self.last_slam_debug_time) < self.slam_debug_period:
            return

        self.last_slam_debug_time = now_seconds
        self.get_logger().info(
            'slam_debug dt=%.3f odl=%.6f odr=%.6f dodl=%.6f dodr=%.6f '
            'gyro=[%.4f %.4f %.4f] accel=[%.4f %.4f %.4f] mag=[%.2f %.2f %.2f]'
            % (dt, odl, odr, dodl, dodr, gx, gy, gz, ax, ay, az, mx, my, mz)
        )

    def destroy_node(self):
        try:
            self.hardware.stop()
        except Exception as exc:  # pragma: no cover
            self.get_logger().warning(f'failed to stop base on shutdown: {exc}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
