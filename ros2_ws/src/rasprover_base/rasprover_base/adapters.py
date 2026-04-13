import json

from rasprover_msgs.msg import RawRobotFeedback
from rasprover_msgs.msg import RobotFeedback

from .legacy_imports import BaseDriver
from .legacy_imports import StateStore


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class RobotHardwareAdapter:
    """Typed command adapter over the legacy base driver."""

    def __init__(self, port=None, baud=115200, left_drive_scale=1.0, right_drive_scale=1.0):
        self.state_store = StateStore()
        self.left_drive_scale = float(left_drive_scale)
        self.right_drive_scale = float(right_drive_scale)
        if port:
            self.base_driver = BaseDriver(port, baud)
        else:
            self.base_driver = BaseDriver(self._default_port(), baud)
        self.base_driver.attach_state_store(self.state_store)

    @staticmethod
    def _default_port():
        try:
            with open('/proc/cpuinfo', 'r', encoding='utf-8') as cpuinfo:
                for line in cpuinfo:
                    if 'Model' in line and 'Raspberry Pi 5' in line:
                        return '/dev/ttyAMA0'
        except OSError:
            pass
        return '/dev/serial0'

    def send_motion(self, linear, angular):
        left = clamp((linear - angular) * self.left_drive_scale, -1.0, 1.0)
        right = clamp((linear + angular) * self.right_drive_scale, -1.0, 1.0)
        self.base_driver.send_lr(left, right)
        return left, right

    def send_gimbal(self, pan, tilt, speed, accel):
        self.base_driver.gimbal(pan, tilt, speed, accel)
        return pan, tilt, speed, accel

    def send_lights(self, base_pwm, head_pwm):
        base_pwm = int(clamp(int(base_pwm), 0, 255))
        head_pwm = int(clamp(int(head_pwm), 0, 255))
        self.base_driver.set_lights(base_pwm, head_pwm)
        return base_pwm, head_pwm

    def send_servo_setup(self, action, old_id=0, new_id=0, servo_id=0, status=0):
        if action == 'set_pan_id':
            self.base_driver.set_servo_id(old_id or 255, new_id or 2)
        elif action == 'set_tilt_id':
            self.base_driver.set_servo_id(old_id or 255, new_id or 1)
        elif action == 'release':
            self.base_driver.set_servo_torque(servo_id or 255, status)
        elif action == 'middle_set':
            self.base_driver.set_servo_mid(servo_id or 255)
        else:
            raise ValueError(f'unsupported servo setup action: {action}')
        return action

    def get_feedback(self):
        return self.base_driver.get_feedback()

    def stop(self):
        self.base_driver.stop()

    def send_oled_line(self, line, text):
        self.base_driver.oled(int(line), str(text))
        return int(line), str(text)


class RobotFeedbackAdapter:
    """Converts legacy base feedback into ROS-native messages."""

    def raw_feedback_msg(self, stamp, feedback, source='base_driver'):
        msg = RawRobotFeedback()
        msg.stamp = stamp
        msg.source = source
        if isinstance(feedback, dict):
            msg.payload_json = json.dumps(feedback, ensure_ascii=True, separators=(',', ':'))
        else:
            msg.payload_json = ''
        return msg

    def robot_feedback_msg(self, stamp, feedback, source='base_driver'):
        msg = RobotFeedback()
        msg.stamp = stamp
        msg.source = source
        msg.raw_available = isinstance(feedback, dict)
        msg.motion_state = 'unknown'

        if not isinstance(feedback, dict):
            return msg

        msg.battery_voltage = float(feedback.get('v', 0.0) or 0.0)
        msg.gimbal_pan = float(feedback.get('pan', feedback.get('x', 0.0)) or 0.0)
        msg.gimbal_tilt = float(feedback.get('tilt', feedback.get('y', 0.0)) or 0.0)
        msg.base_light_pwm = int(feedback.get('base_light', feedback.get('io4', 0)) or 0)
        msg.head_light_pwm = int(feedback.get('head_light', feedback.get('io5', 0)) or 0)

        if any(abs(float(feedback.get(key, 0.0) or 0.0)) > 0.001 for key in ('L', 'R', 'l', 'r')):
            msg.motion_state = 'moving'
        elif msg.raw_available:
            msg.motion_state = 'idle'

        if feedback.get('fault'):
            msg.fault_flags.append(str(feedback['fault']))
        if feedback.get('err'):
            msg.fault_flags.append(str(feedback['err']))

        return msg
