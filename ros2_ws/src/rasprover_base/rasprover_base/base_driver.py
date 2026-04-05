from .base_ctrl import BaseController


class BaseDriver:
    def __init__(self, port, baud):
        self.base = BaseController(port, baud)
        self.state_store = None

    def attach_state_store(self, state_store):
        self.state_store = state_store

    def send_lr(self, left, right):
        cmd = {"T": 1, "L": left, "R": right}
        self.send_json(cmd)

    def stop(self):
        self.send_lr(0, 0)

    def send_json(self, data):
        self.base.base_json_ctrl(data)

    def set_lights(self, base_pwm, head_pwm=None):
        if head_pwm is None:
            head_pwm = self.base.head_light_status
        self.base.lights_ctrl(base_pwm, head_pwm)
        if self.state_store is not None:
            self.state_store.update_robot_state(
                base_light_status=self.base.base_light_status,
                head_light_status=self.base.head_light_status,
            )

    def set_light(self, val):
        self.set_lights(val)

    def gimbal(self, x, y, speed, acceleration):
        self.base.gimbal_ctrl(x, y, speed, acceleration)

    def oled(self, line, text):
        self.base.base_oled(line, text)

    def set_servo_id(self, old_id, new_id):
        self.base.bus_servo_id_set(old_id, new_id)

    def set_servo_torque(self, servo_id, status):
        self.base.bus_servo_torque_lock(servo_id, status)

    def set_servo_mid(self, servo_id):
        self.base.bus_servo_mid_set(servo_id)

    def get_feedback(self):
        feedback = self.base.feedback_data()
        if self.state_store is not None:
            self.state_store.update_base_feedback(feedback)
        return feedback
