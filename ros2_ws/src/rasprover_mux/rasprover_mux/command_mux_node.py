import copy

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from rasprover_msgs.msg import CvControlIntent
from rasprover_msgs.msg import GimbalCommand
from rasprover_msgs.msg import LightCommand
from rasprover_msgs.msg import MotionCommand
from rasprover_msgs.msg import ServoSetupCommand

from .arbiters import MotionArbiter
from .arbiters import TimedCommandArbiter


class CommandMuxNode(Node):
    """Decision and policy boundary for robot typed commands."""

    def __init__(self):
        super().__init__('command_mux_node')

        self.declare_parameter('tick_period_sec', 0.05)
        self.declare_parameter('stop_priority', 255)
        self.declare_parameter('default_timeout_ms', 500)

        self.tick_period = self.get_parameter('tick_period_sec').get_parameter_value().double_value
        self.stop_priority = self.get_parameter('stop_priority').get_parameter_value().integer_value
        self.default_timeout_ms = self.get_parameter('default_timeout_ms').get_parameter_value().integer_value

        self.motion_arbiter = MotionArbiter()
        self.gimbal_arbiter = TimedCommandArbiter(self.default_timeout_ms)
        self.light_arbiter = TimedCommandArbiter(self.default_timeout_ms)
        self.motion_estop = False

        self.motion_sub = self.create_subscription(
            MotionCommand,
            '/ui/cmd/motion',
            self.handle_ui_motion,
            10,
        )
        self.gimbal_sub = self.create_subscription(
            GimbalCommand,
            '/ui/cmd/gimbal',
            self.handle_ui_gimbal,
            10,
        )
        self.light_sub = self.create_subscription(
            LightCommand,
            '/ui/cmd/lights',
            self.handle_ui_lights,
            10,
        )
        self.stop_sub = self.create_subscription(
            Bool,
            '/system/cmd/stop',
            self.handle_stop,
            10,
        )
        self.servo_setup_sub = self.create_subscription(
            ServoSetupCommand,
            '/ui/cmd/servo_setup',
            self.handle_servo_setup,
            10,
        )
        self.cv_intent_sub = self.create_subscription(
            CvControlIntent,
            '/cv/control_intent',
            self.handle_cv_control_intent,
            10,
        )
        self.motion_pub = self.create_publisher(MotionCommand, '/robot/cmd/motion', 10)
        self.gimbal_pub = self.create_publisher(GimbalCommand, '/robot/cmd/gimbal', 10)
        self.light_pub = self.create_publisher(LightCommand, '/robot/cmd/lights', 10)
        self.servo_setup_pub = self.create_publisher(ServoSetupCommand, '/robot/cmd/servo_setup', 10)
        self.timer = self.create_timer(self.tick_period, self.on_tick)

        self.get_logger().info('command_mux_node started for motion, gimbal, and light lanes')

    def now_seconds(self):
        now = self.get_clock().now().nanoseconds
        return now / 1_000_000_000.0

    def zero_motion_command(self, source='system_stop', mode='stop'):
        msg = MotionCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.source = source
        msg.linear = 0.0
        msg.angular = 0.0
        msg.priority = self.stop_priority
        msg.mode = mode
        msg.timeout_ms = self.default_timeout_ms
        msg.frame_id = 'base_link'
        return msg

    def publish_motion(self, msg):
        msg.stamp = self.get_clock().now().to_msg()
        self.motion_pub.publish(msg)

    def publish_gimbal(self, msg):
        msg.stamp = self.get_clock().now().to_msg()
        self.gimbal_pub.publish(msg)

    def publish_lights(self, msg):
        msg.stamp = self.get_clock().now().to_msg()
        self.light_pub.publish(msg)

    def publish_servo_setup(self, msg):
        msg.stamp = self.get_clock().now().to_msg()
        self.servo_setup_pub.publish(msg)

    def handle_ui_motion(self, msg):
        if self.motion_estop:
            self.get_logger().info('ignoring UI motion while estop is active')
            return

        motion_msg = copy.deepcopy(msg)
        if int(motion_msg.timeout_ms or 0) <= 0:
            motion_msg.timeout_ms = self.default_timeout_ms

        active = self.motion_arbiter.set_command(motion_msg, self.now_seconds(), origin='ui')
        self.publish_motion(active.command)
        self.get_logger().info(
            'forwarded motion source=%s linear=%.3f angular=%.3f timeout_ms=%d'
            % (
                motion_msg.source,
                motion_msg.linear,
                motion_msg.angular,
                motion_msg.timeout_ms,
            )
        )

    def handle_ui_gimbal(self, msg):
        if self.motion_estop:
            self.get_logger().info('ignoring UI gimbal while estop is active')
            return

        gimbal_msg = copy.deepcopy(msg)
        if int(gimbal_msg.timeout_ms or 0) <= 0:
            gimbal_msg.timeout_ms = self.default_timeout_ms

        active = self.gimbal_arbiter.set_command(gimbal_msg, self.now_seconds(), origin='ui')
        self.publish_gimbal(active.command)
        self.get_logger().info(
            'forwarded gimbal source=%s pan=%.3f tilt=%.3f speed=%.3f accel=%.3f timeout_ms=%d'
            % (
                gimbal_msg.source,
                gimbal_msg.pan,
                gimbal_msg.tilt,
                gimbal_msg.speed,
                gimbal_msg.accel,
                gimbal_msg.timeout_ms,
            )
        )

    def handle_ui_lights(self, msg):
        light_msg = copy.deepcopy(msg)
        if int(light_msg.timeout_ms or 0) <= 0:
            light_msg.timeout_ms = self.default_timeout_ms

        active = self.light_arbiter.set_command(light_msg, self.now_seconds(), origin='ui')
        self.publish_lights(active.command)
        self.get_logger().info(
            'forwarded lights source=%s mode=%s base_pwm=%d head_pwm=%d timeout_ms=%d'
            % (
                light_msg.source,
                light_msg.mode,
                light_msg.base_pwm,
                light_msg.head_pwm,
                light_msg.timeout_ms,
            )
        )

    def handle_servo_setup(self, msg):
        setup_msg = copy.deepcopy(msg)
        self.publish_servo_setup(setup_msg)
        self.get_logger().info(
            'forwarded servo_setup source=%s action=%s old_id=%d new_id=%d servo_id=%d status=%d'
            % (
                setup_msg.source,
                setup_msg.action,
                setup_msg.old_id,
                setup_msg.new_id,
                setup_msg.servo_id,
                setup_msg.status,
            )
        )

    def handle_cv_control_intent(self, msg):
        now_seconds = self.now_seconds()
        if msg.kind == 'motion':
            if self.motion_arbiter.has_active(now_seconds) and self.motion_arbiter.active_origin() == 'ui':
                self.get_logger().debug('ignoring cv motion intent while ui motion command is active')
                return
            motion_msg = MotionCommand()
            motion_msg.stamp = msg.stamp
            motion_msg.source = msg.source or 'cv_intent'
            motion_msg.linear = msg.linear
            motion_msg.angular = msg.angular
            motion_msg.priority = 10
            motion_msg.mode = msg.mode or 'cv'
            motion_msg.timeout_ms = msg.timeout_ms or self.default_timeout_ms
            motion_msg.frame_id = 'base_link'
            active = self.motion_arbiter.set_command(motion_msg, now_seconds, origin='cv')
            self.publish_motion(active.command)
            self.get_logger().info(
                'forwarded cv motion source=%s mode=%s linear=%.3f angular=%.3f'
                % (motion_msg.source, motion_msg.mode, motion_msg.linear, motion_msg.angular)
            )
            return

        if msg.kind == 'gimbal':
            if self.gimbal_arbiter.has_active(now_seconds) and self.gimbal_arbiter.active_origin() == 'ui':
                self.get_logger().debug('ignoring cv gimbal intent while ui gimbal command is active')
                return
            gimbal_msg = GimbalCommand()
            gimbal_msg.stamp = msg.stamp
            gimbal_msg.source = msg.source or 'cv_intent'
            gimbal_msg.pan = msg.pan
            gimbal_msg.tilt = msg.tilt
            gimbal_msg.speed = msg.speed
            gimbal_msg.accel = msg.accel
            gimbal_msg.mode = msg.mode or 'cv'
            gimbal_msg.timeout_ms = msg.timeout_ms or self.default_timeout_ms
            active = self.gimbal_arbiter.set_command(gimbal_msg, now_seconds, origin='cv')
            self.publish_gimbal(active.command)
            self.get_logger().info(
                'forwarded cv gimbal source=%s mode=%s pan=%.3f tilt=%.3f'
                % (gimbal_msg.source, gimbal_msg.mode, gimbal_msg.pan, gimbal_msg.tilt)
            )
            return

        if msg.kind == 'lights':
            if self.light_arbiter.has_active(now_seconds) and self.light_arbiter.active_origin() == 'ui':
                self.get_logger().debug('ignoring cv light intent while ui light command is active')
                return
            light_msg = LightCommand()
            light_msg.stamp = msg.stamp
            light_msg.source = msg.source or 'cv_intent'
            light_msg.mode = msg.mode or 'cv'
            light_msg.base_pwm = msg.base_pwm
            light_msg.head_pwm = msg.head_pwm
            light_msg.timeout_ms = msg.timeout_ms or self.default_timeout_ms
            active = self.light_arbiter.set_command(light_msg, now_seconds, origin='cv')
            self.publish_lights(active.command)
            self.get_logger().info(
                'forwarded cv lights source=%s mode=%s base_pwm=%d head_pwm=%d'
                % (light_msg.source, light_msg.mode, light_msg.base_pwm, light_msg.head_pwm)
            )
            return

    def zero_gimbal_command(self, source='system_stop', mode='stop'):
        msg = GimbalCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.source = source
        msg.pan = 0.0
        msg.tilt = 0.0
        msg.speed = 0.0
        msg.accel = 0.0
        msg.mode = mode
        msg.timeout_ms = self.default_timeout_ms
        return msg

    def zero_light_command(self, source='system_stop', mode='stop'):
        msg = LightCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.source = source
        msg.mode = mode
        msg.base_pwm = 0
        msg.head_pwm = 0
        msg.timeout_ms = self.default_timeout_ms
        return msg

    def handle_stop(self, msg):
        self.motion_estop = bool(msg.data)
        if self.motion_estop:
            self.motion_arbiter.clear()
            self.gimbal_arbiter.clear()
            self.light_arbiter.clear()
            self.publish_motion(self.zero_motion_command())
            self.publish_gimbal(self.zero_gimbal_command())
            self.publish_lights(self.zero_light_command())
            self.get_logger().warning('system stop asserted, published zero motion')
        else:
            self.get_logger().info('system stop cleared')

    def on_tick(self):
        if self.motion_estop:
            return
        if self.motion_arbiter.expired(self.now_seconds()):
            self.motion_arbiter.clear()
            self.publish_motion(self.zero_motion_command(source='motion_timeout', mode='timeout'))
            self.get_logger().info('motion timeout expired, published zero motion')
        if self.gimbal_arbiter.expired(self.now_seconds()):
            self.gimbal_arbiter.clear()
            self.get_logger().info('gimbal timeout expired, cleared active gimbal command without recentering')
        if self.light_arbiter.expired(self.now_seconds()):
            self.light_arbiter.clear()
            self.publish_lights(self.zero_light_command(source='light_timeout', mode='timeout'))
            self.get_logger().info('light timeout expired, published zero lights')


def main(args=None):
    rclpy.init(args=args)
    node = CommandMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
