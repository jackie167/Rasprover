# ROS Migration Checklist

## Goal

Move this project from a monolithic Flask-first robot app toward a ROS-based architecture with clear node boundaries, message contracts, and hardware isolation.

## Current Readiness

### Ready or nearly ready

- `app.py`
  - Good bootstrap/lifecycle entrypoint.
  - Can be replaced later by a ROS launch file or supervisor entrypoint.

- `web_ui.py`
  - Already separated as a web adapter layer.
  - Good candidate to become a ROS-aware bridge node or a separate UI gateway process.

- `cmd_mux.py`
  - Already acts as a facade for command intake and source tracking.
  - Good place to map web/manual/CV intent into ROS command publishers.

- `command_router.py`
  - Web action-code mapping is isolated.
  - Easy to keep as a UI-side command map.

- `command_service.py`
  - Action execution logic is separated from the web layer.
  - Good candidate to split into ROS command clients/publishers.

- `app_config.py`
  - Central config boundary already exists.
  - Easy to replace with ROS params gradually.

- `state_store.py`
  - Shared runtime state already grouped.
  - Good source for future ROS topic/status aggregation.

- `base_driver.py`
  - Clean adapter between app logic and hardware control.
  - Best place to evolve into a ROS hardware interface wrapper.

### Needs more work before ROS

- `base_ctrl.py`
  - Still mixes serial transport, packet parsing, sensor access, and hardware helpers.
  - Should eventually split into transport + protocol + hardware adapter layers.

- `cv_ctrl.py`
  - Still contains both perception logic and direct robot actuation intent.
  - Should be split into perception outputs vs control decisions.

- `audio_ctrl.py`
  - Functional, but still side-effect driven and tightly local.
  - Should be exposed as a simple ROS service/action if needed.

## Suggested ROS Node Split

### 1. `robot_base_node`

Responsibility:
- Own UART connection to ESP32/base
- Publish parsed feedback
- Accept low-level motion/light/servo commands

Likely extracted from:
- `base_driver.py`
- `base_ctrl.py`

ROS interfaces:
- Subscribe: `/robot/cmd/motion`
- Subscribe: `/robot/cmd/gimbal`
- Subscribe: `/robot/cmd/lights`
- Subscribe: `/robot/cmd/raw_json` optional/debug only
- Publish: `/robot/state/base_feedback`
- Publish: `/robot/state/extra_sensor`
- Publish: `/robot/state/lidar`

Internal split:
- ROS-facing typed command API
- base protocol adapter for UART/JSON framing

### 2. `cv_node`

Responsibility:
- Own camera capture
- Run vision modes
- Publish perception results
- Optionally publish rendered debug image

Likely extracted from:
- `cv_ctrl.py`

ROS interfaces:
- Publish: `/cv/image_raw`
- Publish: `/cv/image_debug`
- Publish: `/cv/detections`
- Publish: `/cv/tracking_target`
- Publish: `/cv/status`
- Publish: `/cv/control_intent`

Rule:
- `cv_node` publishes perception and control intent
- `cv_node` does not publish final robot motion/gimbal/light commands

### 3. `command_mux_node`

Responsibility:
- Source tracking
- Priority/policy/manual override
- Convert high-level intent into robot commands
- Timeout policy
- Emergency stop
- Command dedupe/rate limiting

Likely extracted from:
- `cmd_mux.py`
- `command_service.py`

ROS interfaces:
- Subscribe: `/ui/cmd/action`
- Subscribe: `/ui/cmd/json`
- Subscribe: `/cv/control_intent`
- Subscribe: `/system/cmd`
- Publish: `/robot/cmd/motion`
- Publish: `/robot/cmd/gimbal`
- Publish: `/robot/cmd/lights`
- Publish: `/robot/cmd/raw_json` only as compatibility/debug fallback
- Publish: `/system/state/command_source`

Rule:
- this node is the single writer for official motion/gimbal/light topics

### 4. `system_info_node`

Responsibility:
- CPU/RAM/Wi-Fi/IP/media size status

Likely extracted from:
- `os_info.py`

ROS interfaces:
- Publish: `/system/state/host`

Rule:
- host telemetry only
- do not move application business state into this node

### 5. `web_bridge_node`

Responsibility:
- Keep Flask/Socket.IO or replace later
- Translate browser events into ROS messages
- Read ROS state and emit websocket updates

Rule:
- keep this node as stateless as possible
- avoid business-critical state here

Likely extracted from:
- `web_ui.py`

ROS interfaces:
- Publish: `/ui/cmd/action`
- Publish: `/ui/cmd/json`
- Publish: `/ui/cmd/text`
- Subscribe: `/system/state/host`
- Subscribe: `/robot/state/base_feedback`
- Subscribe: `/cv/status`

### 6. `audio_node`

Responsibility:
- TTS and file playback

Likely extracted from:
- `audio_ctrl.py`

ROS interfaces:
- Subscribe: `/audio/play_file`
- Subscribe: `/audio/say`
- Subscribe: `/audio/stop`

### 7. `state_aggregator_node`

Responsibility:
- Aggregate robot, CV, and host state into UI/dashboard-friendly summaries

ROS interfaces:
- Subscribe: `/robot/state/*`
- Subscribe: `/cv/status`
- Subscribe: `/system/state/host`
- Publish: `/system/state/summary`

## Message/Contract Work Needed

- Define a stable motion command schema
  - linear, angular, source, timestamp, timeout/policy fields

- Define a stable gimbal command schema
  - pan, tilt, speed, accel, source

- Define a stable light command schema
  - base_pwm, head_pwm, mode, source

- Define a stable CV status schema
  - mode, reaction, motion_lock, pan_angle, tilt_angle, fps

- Define a stable robot feedback schema
  - normalized feedback and raw payload separately

Reference:
- [ROS_CONTRACTS.md](/home/ws/ugv_rpi/ROS_CONTRACTS.md)

## Migration Order

### Phase 1

- Keep current app running
- Freeze message contracts first
- Remove direct hardware access outside `cmd_mux`

### Phase 2

- Build `robot_base_node`
- Put `base_driver/base_ctrl` behind typed ROS command topics
- Keep raw JSON as compatibility/debug only

### Phase 3

- Split `cv_ctrl` into:
  - perception publisher
  - control-intent publisher
- remove direct actuation from CV logic

### Phase 4

- Convert `web_ui` into a ROS bridge node
- Replace direct in-process state aggregation with ROS topic subscriptions

### Phase 5

- Replace `app.py` runtime boot flow with ROS launch files or supervisor

## Main Risks

- `cv_ctrl.py` still mixes perception and control intent.
- Raw JSON commands can become long-term debt if kept as a first-class API.
- Current web update payloads still depend on numeric feedback codes from `config.yaml`.
- Serial protocol parsing in `base_ctrl.py` is tolerant, but still tightly coupled to runtime behavior.

## Short Recommendation

Use the current architecture as the staging layer before ROS:

- keep `cmd_mux` as the policy boundary
- move hardware I/O outward first
- move perception outward second
- move web bridge outward last

That order will preserve working robot behavior while steadily replacing the runtime underneath.
