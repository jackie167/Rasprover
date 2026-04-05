"""Legacy in-process command mux.

This class is retained for compatibility with the fallback app.py runtime.
The primary command policy runtime now lives in ROS command_mux_node.
"""

from .command_router import CommandRouter
from .command_service import CommandService


class CmdMux:
    def __init__(self, base_driver, app_config, state_store):
        self.base_driver = base_driver
        self.base = base_driver.base
        self.app_config = app_config
        self.state_store = state_store
        self.command_service = CommandService(self, app_config, state_store)
        self.command_router = CommandRouter(app_config)
        self.cmd_actions, self.cmd_feedback_actions = self.command_router.build_actions(self.command_service)

    def attach_cv_controller(self, cvf):
        self.command_service.attach_cv_controller(cvf)
        self.sync_cv_state()

    @property
    def rl(self):
        return self.base.rl

    @property
    def base_data(self):
        return self.base.base_data

    @property
    def use_lidar(self):
        return self.base.use_lidar

    @property
    def extra_sensor(self):
        return self.base.extra_sensor

    @property
    def base_light_status(self):
        return self.base.base_light_status

    @base_light_status.setter
    def base_light_status(self, value):
        self.base.base_light_status = value

    @property
    def head_light_status(self):
        return self.base.head_light_status

    @head_light_status.setter
    def head_light_status(self, value):
        self.base.head_light_status = value

    def sync_cv_state(self):
        self.command_service.sync_cv_state()

    def send_json(self, data):
        self.base_driver.send_json(data)

    # Compatibility alias so legacy call sites still route through mux.
    def base_json_ctrl(self, data):
        self.send_json(data)

    def stop(self):
        self.base_driver.stop()

    def breath_light(self, input_time):
        self.base.breath_light(input_time)

    def base_speed_ctrl(self, left, right):
        self.base_driver.send_lr(left, right)

    def gimbal_ctrl(self, x, y, speed, acceleration):
        self.base_driver.gimbal(x, y, speed, acceleration)

    def gimbal_base_ctrl(self, x, y, speed):
        self.send_json({"T": 141, "X": x, "Y": y, "SPD": speed})

    def base_oled(self, line, text):
        self.base_driver.oled(line, text)

    def bus_servo_id_set(self, old_id, new_id):
        self.base_driver.set_servo_id(old_id, new_id)

    def bus_servo_torque_lock(self, servo_id, status):
        self.base_driver.set_servo_torque(servo_id, status)

    def bus_servo_mid_set(self, servo_id):
        self.base_driver.set_servo_mid(servo_id)

    def lights_ctrl(self, base_pwm, head_pwm):
        self.base_driver.set_lights(base_pwm, head_pwm)
        self.state_store.update_robot_state(
            base_light_status=self.base.base_light_status,
            head_light_status=self.base.head_light_status,
        )

    # CV interface so cv_ctrl can use the same boundary in both legacy and ROS paths.
    def emit_cv_gimbal_intent(self, pan, tilt, speed, accel, mode=None, timeout_ms=None):
        self.gimbal_ctrl(pan, tilt, speed, accel)

    def emit_cv_light_intent(self, base_pwm, head_pwm, mode=None, timeout_ms=None):
        self.lights_ctrl(base_pwm, head_pwm)

    def emit_cv_motion_intent(self, linear, angular, mode=None, timeout_ms=None):
        self.send_json({"T": 13, "X": linear, "Z": angular})

    def emit_tracking_target(self, **kwargs):
        return

    def base_lights_ctrl(self):
        if self.base.base_light_status != 0:
            self.lights_ctrl(0, self.base.head_light_status)
        else:
            self.lights_ctrl(255, self.base.head_light_status)

    def feedback_data(self):
        return self.base_driver.get_feedback()

    def set_version(self, input_main, input_module):
        self.command_service.set_version(input_main, input_module, source="system")

    def handle_json_command(self, payload, source="web_json"):
        self.command_service.handle_json_command(payload, source=source)

    def handle_action(self, cmd_a, source="web_socket"):
        action = self.cmd_actions.get(cmd_a)
        if action:
            self.state_store.mark_command(source, cmd_a)
            self.command_service.execute_action(action)
        return cmd_a in self.cmd_feedback_actions

    def play_audio_file(self, audio_file):
        self.command_service.play_audio_file(audio_file)

    def stop_audio(self):
        self.command_service.stop_audio()

    def cmdline_ctrl(self, args_string, source="web_command"):
        if not args_string:
            return

        self.state_store.mark_command(source, args_string)
        self.command_service.show_command_info(args_string)
        self.command_service.handle_cmdline(args_string, source=source)
