import threading
import time


class StateStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._state = {
            "cv": {
                "mode": None,
                "detection_reaction_mode": None,
                "motion_lock": None,
                "led_mode": 0,
                "pan_angle": 0,
                "tilt_angle": 0,
                "video_fps": 0,
            },
            "command": {
                "source": None,
                "selected": None,
                "source_timestamps": {},
            },
            "robot": {
                "base_data": None,
                "base_voltage": 0,
                "base_light_status": 0,
                "head_light_status": 0,
            },
            "system": {
                "cpu_load": 0,
                "cpu_temp": 0,
                "ram_usage": 0,
                "wifi_rssi": 0,
                "wifi_mode": None,
                "wlan_ip": None,
                "eth0_ip": None,
                "picture_size": 0,
                "video_size": 0,
            },
        }

    def update_cv_state(self, **kwargs):
        with self._lock:
            self._state["cv"].update(kwargs)

    def update_command_state(self, **kwargs):
        with self._lock:
            self._state["command"].update(kwargs)

    def update_robot_state(self, **kwargs):
        with self._lock:
            self._state["robot"].update(kwargs)

    def update_system_state(self, **kwargs):
        with self._lock:
            self._state["system"].update(kwargs)

    def snapshot(self):
        with self._lock:
            return {
                "cv": dict(self._state["cv"]),
                "command": {
                    **self._state["command"],
                    "source_timestamps": dict(self._state["command"]["source_timestamps"]),
                },
                "robot": dict(self._state["robot"]),
                "system": dict(self._state["system"]),
            }

    def mark_command(self, source, command):
        with self._lock:
            self._state["command"]["source"] = source
            self._state["command"]["selected"] = command
            self._state["command"]["source_timestamps"][source] = time.time()

    def update_system_info(self, system_info):
        self.update_system_state(
            picture_size=system_info.pictures_size,
            video_size=system_info.videos_size,
            cpu_load=system_info.cpu_load,
            cpu_temp=system_info.cpu_temp,
            ram_usage=system_info.ram,
            wifi_rssi=system_info.wifi_rssi,
            wifi_mode=system_info.wifi_mode,
            wlan_ip=system_info.wlan_ip,
            eth0_ip=system_info.eth0_ip,
        )

    def update_base_feedback(self, base_data):
        if not isinstance(base_data, dict):
            return
        base_voltage = base_data.get('v', self.snapshot()["robot"]["base_voltage"])
        self.update_robot_state(base_data=base_data, base_voltage=base_voltage)
