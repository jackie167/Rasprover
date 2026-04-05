class CommandRouter:
    def __init__(self, app_config):
        self.app_config = app_config

    def build_actions(self, command_service):
        code = self.app_config.code
        actions = {
            code('zoom_x1'): lambda: command_service.scale_ctrl(1),
            code('zoom_x2'): lambda: command_service.scale_ctrl(2),
            code('zoom_x4'): lambda: command_service.scale_ctrl(4),
            code('pic_cap'): command_service.picture_capture,
            code('vid_sta'): lambda: command_service.video_record(True),
            code('vid_end'): lambda: command_service.video_record(False),
            code('cv_none'): lambda: command_service.set_cv_mode(code('cv_none')),
            code('cv_moti'): lambda: command_service.set_cv_mode(code('cv_moti')),
            code('cv_face'): lambda: command_service.set_cv_mode(code('cv_face')),
            code('cv_objs'): lambda: command_service.set_cv_mode(code('cv_objs')),
            code('cv_clor'): lambda: command_service.set_cv_mode(code('cv_clor')),
            code('mp_hand'): lambda: command_service.set_cv_mode(code('mp_hand')),
            code('cv_auto'): lambda: command_service.set_cv_mode(code('cv_auto')),
            code('mp_face'): lambda: command_service.set_cv_mode(code('mp_face')),
            code('mp_pose'): lambda: command_service.set_cv_mode(code('mp_pose')),
            code('re_none'): lambda: command_service.set_detection_reaction(code('re_none')),
            code('re_capt'): lambda: command_service.set_detection_reaction(code('re_capt')),
            code('re_reco'): lambda: command_service.set_detection_reaction(code('re_reco')),
            code('mc_lock'): lambda: command_service.set_movtion_lock(True),
            code('mc_unlo'): lambda: command_service.set_movtion_lock(False),
            code('led_off'): lambda: command_service.head_light_ctrl(0),
            code('led_aut'): lambda: command_service.head_light_ctrl(1),
            code('led_ton'): lambda: command_service.head_light_ctrl(2),
            code('release'): lambda: command_service.bus_servo_torque_lock(255, 0),
            code('s_panid'): lambda: command_service.bus_servo_id_set(255, 2),
            code('s_tilid'): lambda: command_service.bus_servo_id_set(255, 1),
            code('set_mid'): lambda: command_service.bus_servo_mid_set(255),
            code('base_of'): lambda: command_service.lights_ctrl(0, command_service.head_light_status),
            code('base_on'): lambda: command_service.lights_ctrl(255, command_service.head_light_status),
            code('head_ct'): lambda: command_service.head_light_ctrl(3),
            code('base_ct'): command_service.base_lights_ctrl,
        }
        feedback_actions = {
            code('cv_none'), code('cv_moti'), code('cv_face'), code('cv_objs'),
            code('cv_clor'), code('mp_hand'), code('cv_auto'), code('mp_face'),
            code('mp_pose'), code('re_none'), code('re_capt'), code('re_reco'),
            code('mc_lock'), code('mc_unlo'), code('led_off'), code('led_aut'),
            code('led_ton'), code('base_of'), code('base_on'), code('head_ct'),
            code('base_ct'),
        }
        return actions, feedback_actions
