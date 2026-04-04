"""Legacy runtime entrypoint.

This file is kept only as a fallback/dev path while the ROS stack becomes the
default runtime. The primary boot path is:
web_bridge_node -> command_mux_node -> robot_base_node -> cv_node
"""

import os
import threading
import time

import audio_ctrl
import cv_ctrl
import os_info
from app_config import AppConfig
from base_driver import BaseDriver
from cmd_mux import CmdMux
from state_store import StateStore
from web_ui import WebUI


def is_raspberry_pi5():
    with open('/proc/cpuinfo', 'r', encoding='utf-8') as file:
        for line in file:
            if 'Model' in line:
                return 'Raspberry Pi 5' in line
    return False


def create_base_driver():
    if is_raspberry_pi5():
        return BaseDriver('/dev/ttyAMA0', 115200)
    return BaseDriver('/dev/serial0', 115200)


curpath = os.path.realpath(__file__)
THIS_PATH = os.path.dirname(curpath)
CONFIG_PATH = os.path.join(THIS_PATH, 'config.yaml')


def create_runtime():
    app_config = AppConfig(CONFIG_PATH)
    state_store = StateStore()
    base_driver = create_base_driver()
    base_driver.attach_state_store(state_store)
    cmd_mux = CmdMux(base_driver, app_config, state_store)
    cvf = cv_ctrl.OpencvFuncs(THIS_PATH, cmd_mux)
    cmd_mux.attach_cv_controller(cvf)
    system_info = os_info.SystemInfo()
    web_ui = WebUI(THIS_PATH, CONFIG_PATH, app_config, cmd_mux, cvf, system_info, state_store)

    return {
        "app_config": app_config,
        "state_store": state_store,
        "base_driver": base_driver,
        "cmd_mux": cmd_mux,
        "cvf": cvf,
        "system_info": system_info,
        "web_ui": web_ui,
        "app": web_ui.app,
        "socketio": web_ui.socketio,
    }


def initialize_boot_display(runtime):
    app_config = runtime["app_config"]
    cmd_mux = runtime["cmd_mux"]
    threading.Thread(target=lambda: cmd_mux.breath_light(15), daemon=True).start()
    cmd_mux.base_oled(0, app_config.robot_name)
    cmd_mux.base_oled(1, f"sbc_version: {app_config.sbc_version}")
    cmd_mux.base_oled(2, f"{app_config.main_type}{app_config.module_type}")
    cmd_mux.base_oled(3, "Starting...")


def print_startup_banner(runtime):
    app_config = runtime["app_config"]
    print("[startup] robot is started")
    print(f"[startup] robot_name={app_config.robot_name}")
    print(f"[startup] main_type={app_config.main_type} module_type={app_config.module_type}")
    print(f"[startup] web_ui=http://0.0.0.0:5000")
    print(f"[startup] jupyter=http://0.0.0.0:8888")


def update_data_loop(runtime):
    cmd_mux = runtime["cmd_mux"]
    state_store = runtime["state_store"]
    system_info = runtime["system_info"]
    web_ui = runtime["web_ui"]
    cmd_mux.base_oled(2, "F/J:5000/8888")
    start_time = time.time()
    time.sleep(1)
    while True:
        state_store.update_system_info(system_info)
        web_ui.update_data_websocket_single()
        state = state_store.snapshot()
        system_state = state['system']
        eth0 = system_state['eth0_ip']
        wlan = system_state['wlan_ip']
        if eth0:
            cmd_mux.base_oled(0, f"E:{eth0}")
        else:
            cmd_mux.base_oled(0, "E: No Ethernet")
        if wlan:
            cmd_mux.base_oled(1, f"W:{wlan}")
        else:
            cmd_mux.base_oled(1, f"W: NO {system_info.net_interface}")
        elapsed_time = time.time() - start_time
        hours = int(elapsed_time // 3600)
        minutes = int((elapsed_time % 3600) // 60)
        seconds = int(elapsed_time % 60)
        cmd_mux.base_oled(3, f"{system_state['wifi_mode']} {hours:02d}:{minutes:02d}:{seconds:02d} {system_state['wifi_rssi']}dBm")
        time.sleep(5)


def base_data_loop(runtime):
    base_driver = runtime["base_driver"]
    cmd_mux = runtime["cmd_mux"]
    cvf = runtime["cvf"]
    sensor_interval = 1
    sensor_read_time = time.time()
    while True:
        base_feedback = base_driver.get_feedback()
        cvf.update_base_data(base_feedback)

        if cmd_mux.extra_sensor and time.time() - sensor_read_time > sensor_interval:
            cmd_mux.rl.read_sensor_data()
            sensor_read_time = time.time()

        if cmd_mux.use_lidar:
            cmd_mux.rl.lidar_data_recv()

        time.sleep(0.025)


def cmd_on_boot(runtime):
    app_config = runtime["app_config"]
    cmd_mux = runtime["cmd_mux"]
    cvf = runtime["cvf"]
    cmd_list = [
        'base -c {"T":142,"cmd":50}',
        'base -c {"T":131,"cmd":1}',
        'base -c {"T":143,"cmd":0}',
        app_config.module_select_command(),
        'base -c {"T":300,"mode":0,"mac":"EF:EF:EF:EF:EF:EF"}',
        'send -a -b',
    ]
    print(app_config.module_select_command())
    for command in cmd_list:
        cmd_mux.cmdline_ctrl(command, source="system_boot")
        cvf.info_update(command, (0, 255, 255), 0.36)
    cmd_mux.set_version(app_config.main_type, app_config.module_type)


def main():
    runtime = create_runtime()
    initialize_boot_display(runtime)

    app_config = runtime["app_config"]
    cmd_mux = runtime["cmd_mux"]
    system_info = runtime["system_info"]
    socketio = runtime["socketio"]
    app = runtime["app"]

    print_startup_banner(runtime)
    cmd_mux.lights_ctrl(255, 255)
    audio_ctrl.play_random_audio("robot_started", False)
    system_info.update_folder(THIS_PATH)

    if app_config.module_type == 1:
        arm_payload = {"T": app_config.cmd_arm_ctrl_ui, **app_config.arm_default_pose}
        cmd_mux.send_json(arm_payload)
    else:
        cmd_mux.gimbal_ctrl(0, 0, 200, 10)

    system_info.start()
    system_info.resume()
    threading.Thread(target=update_data_loop, args=(runtime,), daemon=True).start()
    threading.Thread(target=base_data_loop, args=(runtime,), daemon=True).start()

    cmd_mux.lights_ctrl(0, 0)
    cmd_on_boot(runtime)
    socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)


if __name__ == "__main__":
    main()
