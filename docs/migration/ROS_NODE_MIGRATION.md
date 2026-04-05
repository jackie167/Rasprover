# ROS Node Migration Status

Current target runtime:

`web_bridge_node -> command_mux_node -> robot_base_node -> base_driver/base_ctrl`

Current full-stack runtime on the Pi:

`web_bridge_node -> command_mux_node -> robot_base_node`

and in parallel:

`cv_node`

Current status:

- `robot_base_node`
  - active replacement for the legacy hardware boundary
  - owns `/robot/cmd/motion`
  - owns `/robot/state/feedback`
  - owns `/robot/state/feedback_raw`
- `command_mux_node`
  - active replacement target for the legacy `cmd_mux.py` motion lane
  - owns `/ui/cmd/motion -> /robot/cmd/motion`
  - applies timeout and stop policy
- `web_bridge_node`
  - active replacement target for the legacy `web_ui.py` non-CV web runtime
  - now exposes:
    - motion/gimbal/lights/settings actions
    - state/config/media pages and APIs
    - replacement pages:
      - `/`
      - `/photo`
      - `/video`
      - `/settings`
- `cv_node`
  - initial ROS replacement entrypoint for the legacy `cv_ctrl.py` runtime
  - currently owns:
    - `GET http://<pi>:5051/video_feed`
    - `GET http://<pi>:5051/cv/status`
    - `POST http://<pi>:5051/cv/mode`
    - publish `/cv/status`
    - publish `/cv/control_intent`
  - currently runs CV and camera lifecycle inside ROS
  - currently publishes typed CV intent for the first migrated slice:
    - lights intent
    - gimbal intent wiring

Legacy code status:

- `base_driver.py`, `base_ctrl.py`
  - remain as the old backend
  - are intentionally reused by `robot_base_node`
- `cmd_mux.py`
  - legacy runtime path
  - should no longer be the primary motion runtime once the ROS stack is adopted
- `web_ui.py`
  - legacy runtime path
  - should no longer be the primary non-CV web runtime once the ROS stack is adopted
- `cv_ctrl.py`
  - now wrapped by `cv_node` for camera/CV status/MJPEG serving
  - still needs deeper migration for typed CV intent/output separation

Current replacement scope:

- motion lane
- gimbal lane
- lights lane
- settings servo setup lane
- non-CV browser pages:
  - home
  - photo
  - video
  - settings
- initial CV runtime:
  - MJPEG feed
  - CV status
  - CV control intent endpoint and topic

Operational scripts:

- `./start_ros_motion_stack.sh`
- `./stop_ros_motion_stack.sh`
- `./status_ros_motion_stack.sh`
- `./start_ros_full_stack.sh`
- `./stop_ros_full_stack.sh`
- `./status_ros_full_stack.sh`
- legacy fallback:
  - `./start_legacy_app.sh`
  - `./stop_legacy_app.sh`
  - `./status_legacy_app.sh`

Default autorun target:

- `./start_ros_full_stack.sh`
