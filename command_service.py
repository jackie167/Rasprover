import json
import shlex

import audio_ctrl


class CommandService:
    def __init__(self, mux, app_config, state_store):
        self.mux = mux
        self.app_config = app_config
        self.state_store = state_store
        self.cvf = None

    @property
    def head_light_status(self):
        return self.mux.head_light_status

    def attach_cv_controller(self, cvf):
        self.cvf = cvf

    def sync_cv_state(self):
        if self.cvf is None:
            return
        self.state_store.update_cv_state(
            mode=self.cvf.cv_mode,
            detection_reaction_mode=self.cvf.detection_reaction_mode,
            motion_lock=self.cvf.cv_movtion_lock,
            led_mode=self.cvf.cv_light_mode,
            pan_angle=self.cvf.pan_angle,
            tilt_angle=self.cvf.tilt_angle,
            video_fps=self.cvf.video_fps,
        )

    def picture_capture(self):
        if self.cvf is not None:
            self.cvf.picture_capture()

    def video_record(self, enabled):
        if self.cvf is not None:
            self.cvf.video_record(enabled)

    def scale_ctrl(self, scale):
        if self.cvf is not None:
            self.cvf.scale_ctrl(scale)

    def set_cv_mode(self, mode):
        if self.cvf is not None:
            self.cvf.set_cv_mode(mode)

    def set_detection_reaction(self, reaction):
        if self.cvf is not None:
            self.cvf.set_detection_reaction(reaction)

    def set_movtion_lock(self, enabled):
        if self.cvf is not None:
            self.cvf.set_movtion_lock(enabled)

    def head_light_ctrl(self, mode):
        if self.cvf is not None:
            self.cvf.head_light_ctrl(mode)

    def bus_servo_torque_lock(self, servo_id, status):
        self.mux.bus_servo_torque_lock(servo_id, status)

    def bus_servo_id_set(self, old_id, new_id):
        self.mux.bus_servo_id_set(old_id, new_id)

    def bus_servo_mid_set(self, servo_id):
        self.mux.bus_servo_mid_set(servo_id)

    def lights_ctrl(self, base_pwm, head_pwm):
        self.mux.lights_ctrl(base_pwm, head_pwm)

    def base_lights_ctrl(self):
        self.mux.base_lights_ctrl()

    def set_version(self, input_main, input_module, source="system"):
        self.state_store.mark_command(source, f"set_version:{input_main}{input_module}")
        self.mux.send_json({"T": 900, "main": input_main, "module": input_module})
        if self.cvf is None:
            return
        if input_main == 1:
            self.cvf.info_update("RaspRover", (0, 255, 255), 0.36)
        elif input_main == 2:
            self.cvf.info_update("UGV Rover", (0, 255, 255), 0.36)
        elif input_main == 3:
            self.cvf.info_update("UGV Beast", (0, 255, 255), 0.36)
        if input_module == 0:
            self.cvf.info_update("No Module", (0, 255, 255), 0.36)
        elif input_module == 1:
            self.cvf.info_update("ARM", (0, 255, 255), 0.36)
        elif input_module == 2:
            self.cvf.info_update("PT", (0, 255, 255), 0.36)

    def play_audio_file(self, audio_file):
        audio_ctrl.play_audio_thread(audio_file)

    def stop_audio(self):
        audio_ctrl.stop()

    def execute_action(self, action):
        if action is None:
            return
        action()
        self.sync_cv_state()

    def show_command_info(self, command_text):
        if self.cvf is not None:
            self.cvf.info_update(f"CMD:{command_text}", (0, 255, 255), 0.36)

    def handle_json_command(self, payload, source="web_json"):
        self.state_store.mark_command(source, payload)
        self.mux.send_json(payload)

    def handle_cmdline(self, args_string, source="web_command"):
        if not args_string:
            return

        try:
            args = shlex.split(args_string)
        except ValueError:
            return
        if not args:
            return
        if len(args) < 2 and args[0] not in {'p', 's', 'test'}:
            return
        if args[0] == 'base':
            if args[1] == '-c' or args[1] == '--cmd':
                json_str = args_string.partition(args[1])[2].strip()
                if json_str.startswith('--cmd'):
                    json_str = json_str.partition('--cmd')[2].strip()
                elif json_str.startswith('-c'):
                    json_str = json_str.partition('-c')[2].strip()
                try:
                    self.mux.send_json(json.loads(json_str))
                except json.JSONDecodeError:
                    return
            elif args[1] == '-r' or args[1] == '--recv':
                if self.cvf is None:
                    return
                self.cvf.show_recv_info(args[2] == 'on')

        elif args[0] == 'audio':
            if args[1] == '-s' or args[1] == '--say':
                if len(args) < 3:
                    return
                audio_ctrl.play_speech_thread(' '.join(args[2:]))
            elif args[1] == '-v' or args[1] == '--volume':
                if len(args) < 3:
                    return
                audio_ctrl.set_audio_volume(args[2])
            elif args[1] == '-p' or args[1] == '--play_file':
                if len(args) < 3:
                    return
                audio_ctrl.play_file(args[2])

        elif args[0] == 'send':
            if len(args) < 2:
                return
            if args[1] == '-a' or args[1] == '--add':
                if len(args) < 3:
                    return
                if args[2] == '-b' or args[2] == '--broadcast':
                    self.mux.send_json({"T": 303, "mac": "FF:FF:FF:FF:FF:FF"})
                else:
                    self.mux.send_json({"T": 303, "mac": args[2]})
            elif args[1] == '-rm' or args[1] == '--remove':
                if len(args) < 3:
                    return
                if args[2] == '-b' or args[2] == '--broadcast':
                    self.mux.send_json({"T": 304, "mac": "FF:FF:FF:FF:FF:FF"})
                else:
                    self.mux.send_json({"T": 304, "mac": args[2]})
            elif args[1] == '-b' or args[1] == '--broadcast':
                self.mux.send_json({"T": 306, "mac": "FF:FF:FF:FF:FF:FF", "dev": 0, "b": 0, "s": 0, "e": 0, "h": 0, "cmd": 3, "megs": ' '.join(args[2:])})
            elif args[1] == '-g' or args[1] == '--group':
                self.mux.send_json({"T": 305, "dev": 0, "b": 0, "s": 0, "e": 0, "h": 0, "cmd": 3, "megs": ' '.join(args[2:])})
            else:
                self.mux.send_json({"T": 306, "mac": args[1], "dev": 0, "b": 0, "s": 0, "e": 0, "h": 0, "cmd": 3, "megs": ' '.join(args[2:])})

        elif self.cvf is None:
            return

        elif args[0] == 'cv':
            if args[1] == '-r' or args[1] == '--range':
                if len(args) < 4:
                    return
                try:
                    lower_nums = [int(lower_num) for lower_num in args[2].strip("[]").split(",")]
                    upper_nums = [int(upper_num) for upper_num in args[3].strip("[]").split(",")]
                    if not all(0 <= num <= 255 for num in lower_nums + upper_nums):
                        return
                except Exception:
                    return
                self.cvf.change_target_color(lower_nums, upper_nums)
            elif args[1] == '-s' or args[1] == '--select':
                if len(args) < 3:
                    return
                self.cvf.selet_target_color(args[2])

        elif args[0] == 'video' or args[0] == 'v':
            if len(args) < 3:
                return
            try:
                self.cvf.set_video_quality(int(args[2]))
            except Exception:
                return

        elif args[0] == 'line':
            if args[1] == '-r' or args[1] == '--range':
                if len(args) < 4:
                    return
                try:
                    lower_nums = [int(lower_num) for lower_num in args[2].strip("[]").split(",")]
                    upper_nums = [int(upper_num) for upper_num in args[3].strip("[]").split(",")]
                    if not all(0 <= num <= 255 for num in lower_nums + upper_nums):
                        return
                except Exception:
                    return
                self.cvf.change_line_color(lower_nums, upper_nums)
            elif args[1] == '-s' or args[1] == '--set':
                if len(args) != 9:
                    return
                try:
                    values = [float(args[i]) for i in range(2, 9)]
                except Exception:
                    return
                self.cvf.set_line_track_args(*values)

        elif args[0] == 'track':
            if len(args) < 3:
                return
            self.cvf.set_pt_track_args(args[1], args[2])

        elif args[0] == 'timelapse':
            if args[1] == '-s' or args[1] == '--start':
                if len(args) != 6:
                    return
                try:
                    move_speed = float(args[2])
                    move_time = float(args[3])
                    t_interval = float(args[4])
                    loop_times = int(args[5])
                except Exception:
                    return
                self.cvf.timelapse(move_speed, move_time, t_interval, loop_times)
            elif args[1] == '-e' or args[1] == '--end' or args[1] == '--stop':
                self.cvf.mission_stop()

        elif args[0] == 'p':
            if len(args) < 2:
                return
            main_type = int(args[1][0])
            module_type = int(args[1][1])
            self.set_version(main_type, module_type, source=source)

        elif args[0] == 's':
            if len(args) < 2:
                return
            main_type = int(args[1][0])
            module_type = int(args[1][1])
            self.app_config.set_robot_profile(main_type, module_type)
            self.set_version(main_type, module_type, source=source)

        elif args[0] == 'test':
            self.cvf.update_base_data({"T": 1003, "mac": 1111, "megs": "helllo aaaaaaaa"})

        self.sync_cv_state()
