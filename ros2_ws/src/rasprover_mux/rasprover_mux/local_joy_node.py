import os

import pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class LocalJoyNode(Node):
    """Publishes /joy from a locally attached SDL / pygame joystick."""

    def __init__(self):
        super().__init__('local_joy_node')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('device_index', 0)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('autorepeat_rate_hz', 10.0)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('log_changes', False)

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        self.device_index = self.get_parameter('device_index').get_parameter_value().integer_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.autorepeat_rate_hz = self.get_parameter('autorepeat_rate_hz').get_parameter_value().double_value
        self.deadzone = self.get_parameter('deadzone').get_parameter_value().double_value
        self.log_changes = self.get_parameter('log_changes').get_parameter_value().bool_value

        self.publisher = self.create_publisher(Joy, joy_topic, 10)

        # Prefer the event/input path that pygame uses on Linux SBCs.
        os.environ.setdefault('SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS', '1')

        pygame.init()
        pygame.joystick.init()

        self.joystick = None
        self.last_axes = []
        self.last_buttons = []
        self.last_hats = []
        self.last_publish_time = 0.0

        timer_period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.on_timer)

        self.get_logger().info('local_joy_node started, publishing to %s' % joy_topic)

    def try_open_joystick(self):
        if not pygame.joystick.get_init():
            pygame.joystick.init()

        count = pygame.joystick.get_count()
        if count <= self.device_index:
            if self.joystick is not None:
                self.get_logger().warning('joystick disconnected')
            self.joystick = None
            return

        if self.joystick is not None:
            try:
                if self.joystick.get_init():
                    return
            except Exception:
                self.joystick = None

        joystick = pygame.joystick.Joystick(self.device_index)
        joystick.init()

        self.joystick = joystick
        self.last_axes = [0.0] * joystick.get_numaxes()
        self.last_buttons = [0] * joystick.get_numbuttons()
        self.last_hats = [(0, 0)] * joystick.get_numhats()
        self.get_logger().info(
            'connected joystick index=%d name=%s axes=%d buttons=%d hats=%d'
            % (
                self.device_index,
                joystick.get_name(),
                joystick.get_numaxes(),
                joystick.get_numbuttons(),
                joystick.get_numhats(),
            )
        )

    def current_state(self):
        axes = []
        buttons = []
        hats = []

        if self.joystick is None:
            return axes, buttons, hats

        for idx in range(self.joystick.get_numaxes()):
            value = float(self.joystick.get_axis(idx))
            if abs(value) < self.deadzone:
                value = 0.0
            axes.append(value)

        for idx in range(self.joystick.get_numbuttons()):
            buttons.append(int(self.joystick.get_button(idx)))

        for idx in range(self.joystick.get_numhats()):
            hats.append(tuple(int(v) for v in self.joystick.get_hat(idx)))

        return axes, buttons, hats

    @staticmethod
    def flatten_hats(hats):
        flattened = []
        for hat_x, hat_y in hats:
            flattened.extend([float(hat_x), float(hat_y)])
        return flattened

    def publish_state(self, axes, buttons, hats):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = list(axes) + self.flatten_hats(hats)
        msg.buttons = list(buttons)
        self.publisher.publish(msg)

    def log_detailed_changes(self, axes, buttons, hats):
        axis_changes = []
        button_changes = []
        hat_changes = []

        for idx, value in enumerate(axes):
            previous = self.last_axes[idx] if idx < len(self.last_axes) else None
            if previous is None or abs(value - previous) >= 1e-3:
                axis_changes.append('a%d=%.3f' % (idx, value))

        for idx, value in enumerate(buttons):
            previous = self.last_buttons[idx] if idx < len(self.last_buttons) else None
            if previous is None or value != previous:
                button_changes.append('b%d=%d' % (idx, value))

        for idx, value in enumerate(hats):
            previous = self.last_hats[idx] if idx < len(self.last_hats) else None
            if previous is None or value != previous:
                hat_changes.append('h%d=%s' % (idx, value))

        if axis_changes or button_changes or hat_changes:
            parts = axis_changes + button_changes + hat_changes
            self.get_logger().info('joy changes %s' % ' '.join(parts))

    def on_timer(self):
        pygame.event.pump()
        self.try_open_joystick()
        if self.joystick is None:
            return

        axes, buttons, hats = self.current_state()
        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
        changed = axes != self.last_axes or buttons != self.last_buttons or hats != self.last_hats
        repeat_due = (now - self.last_publish_time) >= (1.0 / max(self.autorepeat_rate_hz, 1.0))

        if changed or repeat_due:
            self.publish_state(axes, buttons, hats)
            if changed and self.log_changes:
                self.log_detailed_changes(axes, buttons, hats)
                self.get_logger().info(
                    'joy axes=%s buttons=%s hats=%s'
                    % (
                        [round(v, 3) for v in axes],
                        buttons,
                        hats,
                    )
                )
            self.last_axes = axes
            self.last_buttons = buttons
            self.last_hats = hats
            self.last_publish_time = now


def main(args=None):
    rclpy.init(args=args)
    node = LocalJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            pygame.joystick.quit()
            pygame.quit()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
