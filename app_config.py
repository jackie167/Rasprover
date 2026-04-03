from config_loader import load_config, save_config


class AppConfig:
    def __init__(self, path):
        self.path = path
        self._data = load_config(path)

    @property
    def robot_name(self):
        return self._data["base_config"]["robot_name"]

    @property
    def sbc_version(self):
        return self._data["base_config"]["sbc_version"]

    @property
    def main_type(self):
        return self._data["base_config"]["main_type"]

    @property
    def module_type(self):
        return self._data["base_config"]["module_type"]

    @property
    def cmd_arm_ctrl_ui(self):
        return self._data["cmd_config"]["cmd_arm_ctrl_ui"]

    def base(self, name):
        return self._data["base_config"][name]

    def args(self, name):
        return self._data["args_config"][name]

    def cmd(self, name):
        return self._data["cmd_config"][name]

    def video(self, name):
        return self._data["video"][name]

    def cv(self, name):
        return self._data["cv"][name]

    @property
    def arm_default_pose(self):
        return {
            "E": self._data["args_config"]["arm_default_e"],
            "Z": self._data["args_config"]["arm_default_z"],
            "R": self._data["args_config"]["arm_default_r"],
        }

    def code(self, name):
        return self._data["code"][name]

    def fb(self, name):
        return self._data["fb"][name]

    def module_select_command(self):
        return f'base -c {{"T":4,"cmd":{self.module_type}}}'

    def set_robot_profile(self, main_type, module_type):
        if main_type == 1:
            robot_name = "RaspRover"
            max_speed = 0.65
            slow_speed = 0.3
        elif main_type == 2:
            robot_name = "UGV Rover"
            max_speed = 1.3
            slow_speed = 0.2
        elif main_type == 3:
            robot_name = "UGV Beast"
            max_speed = 1.0
            slow_speed = 0.2
        else:
            return

        self._data["base_config"]["robot_name"] = robot_name
        self._data["base_config"]["main_type"] = main_type
        self._data["base_config"]["module_type"] = module_type
        self._data["args_config"]["max_speed"] = max_speed
        self._data["args_config"]["slow_speed"] = slow_speed
        self.save()

    def save(self):
        save_config(self.path, self._data)
