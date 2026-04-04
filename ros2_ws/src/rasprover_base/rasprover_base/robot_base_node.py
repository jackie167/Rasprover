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
        self.declare_parameter('feedback_period_sec', 0.05)
        self.declare_parameter('slam_debug_enabled', True)
        self.declare_parameter('slam_debug_period_sec', 0.2)
        self.declare_parameter('slam_debug_motion_threshold', 0.0005)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value or None
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        feedback_period = self.get_parameter('feedback_period_sec').get_parameter_value().double_value
        self.slam_debug_enabled = self.get_parameter('slam_debug_enabled').get_parameter_value().bool_value
        self.slam_debug_period = self.get_parameter('slam_debug_period_sec').get_parameter_value().double_value
        self.slam_debug_motion_threshold = (
            self.get_parameter('slam_debug_motion_threshold').get_parameter_value().double_value
        )

        self.hardware = RobotHardwareAdapter(port=serial_port, baud=baud)
        self.feedback_adapter = RobotFeedbackAdapter()
        self.last_feedback_time = None
        self.last_odl = None
        self.last_odr = None
        self.last_slam_debug_time = 0.0

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
        self.get_logger().info(f'robot_base_node started on serial_port={active_port} baud={baud}')

    def handle_motion_command(self, msg):
        left, right = self.hardware.send_motion(msg.linear, msg.angular)
        self.get_logger().info(
            f'motion source={msg.source} linear={msg.linear:.3f} angular={msg.angular:.3f} left={left:.3f} right={right:.3f}'
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

    def publish_feedback(self):
        feedback = self.hardware.get_feedback()
        stamp = self.get_clock().now().to_msg()
        raw_msg = self.feedback_adapter.raw_feedback_msg(stamp, feedback)
        feedback_msg = self.feedback_adapter.robot_feedback_msg(stamp, feedback)
        self.feedback_raw_pub.publish(raw_msg)
        self.feedback_pub.publish(feedback_msg)
        self.publish_slam_debug(feedback)

    @staticmethod
    def _feedback_float(feedback, key):
        try:
            return float(feedback[key])
        except (KeyError, TypeError, ValueError):
            return None

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

        now_seconds = self.get_clock().now().nanoseconds / 1_000_000_000.0
        dt = 0.0 if self.last_feedback_time is None else max(0.0, now_seconds - self.last_feedback_time)
        dodl = 0.0 if self.last_odl is None else odl - self.last_odl
        dodr = 0.0 if self.last_odr is None else odr - self.last_odr

        self.last_feedback_time = now_seconds
        self.last_odl = odl
        self.last_odr = odr

        moving = abs(dodl) > self.slam_debug_motion_threshold or abs(dodr) > self.slam_debug_motion_threshold
        if not moving:
            return
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
