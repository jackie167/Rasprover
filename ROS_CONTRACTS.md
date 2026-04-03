# ROS Message Contracts

## Goal

Freeze the first set of typed contracts before moving hardware, CV, and UI into ROS nodes.

Design rules:

- Typed topics are the primary contract.
- `/robot/cmd/raw_json` is compatibility/debug only.
- Only `command_mux_node` publishes official robot motion/gimbal/light commands.
- UI and CV publish intent, never final robot actuation.
- Feedback must expose both normalized state and raw payload separately.

## Topic Ownership

### Final robot actuation topics

Published only by:
- `command_mux_node`

Topics:
- `/robot/cmd/motion`
- `/robot/cmd/gimbal`
- `/robot/cmd/lights`

### Intent topics

Published by:
- `web_bridge_node`
- `cv_node`
- `system` automation logic

Topics:
- `/ui/cmd/action`
- `/ui/cmd/text`
- `/ui/cmd/json`
- `/cv/control_intent`
- `/system/cmd`

### Compatibility topic

Optional, temporary, debug/fallback only:
- `/robot/cmd/raw_json`

## Contract 1: Motion Command

Topic:
- `/robot/cmd/motion`

Purpose:
- Chassis linear/angular command in normalized robot-space.

Required fields:
- `stamp`
- `source`
- `linear`
- `angular`

Recommended fields:
- `priority`
- `mode`
- `timeout_ms`
- `frame_id`

Schema:

```yaml
stamp: builtin_interfaces/Time
source: string
linear: float32
angular: float32
priority: uint8
mode: string
timeout_ms: uint32
frame_id: string
```

Semantics:
- `linear`: forward/backward command, normalized or SI-based once chosen.
- `angular`: yaw/turn command, normalized or SI-based once chosen.
- `source`: `web`, `cv`, `system`, `safety`, etc.
- `timeout_ms`: command validity window.

Current mapping:
- current raw protocol often uses:
  - `{"T":13,"X":linear,"Z":angular}`
  - or left/right motor packets in lower layers

Notes:
- Choose one canonical scale early:
  - normalized `[-1.0, 1.0]`, or
  - SI units
- Do not expose `T=13`, `L`, `R`, `X`, `Z` outside `robot_base_node`.

## Contract 2: Gimbal Command

Topic:
- `/robot/cmd/gimbal`

Purpose:
- Command pan/tilt target plus motion parameters.

Required fields:
- `stamp`
- `source`
- `pan`
- `tilt`

Recommended fields:
- `speed`
- `accel`
- `mode`
- `timeout_ms`

Schema:

```yaml
stamp: builtin_interfaces/Time
source: string
pan: float32
tilt: float32
speed: float32
accel: float32
mode: string
timeout_ms: uint32
```

Semantics:
- `pan`: target pan angle
- `tilt`: target tilt angle
- `speed`: desired motion speed
- `accel`: desired motion acceleration

Current mapping:
- current raw protocol uses:
  - `{"T":133,"X":pan,"Y":tilt,"SPD":speed,"ACC":accel}`
  - and sometimes `{"T":141,...}` for base-relative gimbal control

Notes:
- Keep `mode` extensible for future stabilization/tracking/manual modes.

## Contract 3: Light Command

Topic:
- `/robot/cmd/lights`

Purpose:
- Unified head/base light control.

Required fields:
- `stamp`
- `source`
- `mode`

Recommended fields:
- `base_pwm`
- `head_pwm`
- `timeout_ms`

Schema:

```yaml
stamp: builtin_interfaces/Time
source: string
mode: string
base_pwm: uint16
head_pwm: uint16
timeout_ms: uint32
```

Semantics:
- `mode`: examples:
  - `off`
  - `manual_pwm`
  - `head_auto`
  - `head_on`
  - `toggle_base`
- `base_pwm` and `head_pwm` used when `mode == manual_pwm`

Current mapping:
- current raw protocol commonly maps to:
  - `{"T":132,"IO4":base_pwm,"IO5":head_pwm}`

Notes:
- This contract should replace scattered light semantics leaking through UI/CV.

## Contract 4: CV Status

Topic:
- `/cv/status`

Purpose:
- Report current CV runtime state for UI, mux, and logging.

Required fields:
- `stamp`
- `mode`
- `motion_lock`
- `fps`
- `target_present`

Recommended fields:
- `reaction_mode`
- `tracking_state`
- `pan_angle`
- `tilt_angle`
- `led_mode`

Schema:

```yaml
stamp: builtin_interfaces/Time
mode: string
reaction_mode: string
motion_lock: bool
fps: float32
target_present: bool
tracking_state: string
pan_angle: float32
tilt_angle: float32
led_mode: string
```

Semantics:
- `mode`: `none`, `motion`, `face`, `objects`, `color`, `hand`, `auto_drive`, `mp_face`, `mp_pose`
- `reaction_mode`: `none`, `capture`, `record`
- `target_present`: whether the current mode has a valid target/subject
- `tracking_state`: free text or enum like `idle`, `tracking`, `lost`

Current mapping:
- today this state lives across:
  - `cv_ctrl.py`
  - `state_store.py`
  - websocket fb numeric map in config

Notes:
- UI should eventually consume this typed status instead of firmware-derived numeric codes.

## Contract 5: Robot Feedback

Topics:
- `/robot/state/feedback_raw`
- `/robot/state/feedback`

Purpose:
- Separate raw base payload from normalized robot state.

### 5A. Raw feedback

Topic:
- `/robot/state/feedback_raw`

Schema:

```yaml
stamp: builtin_interfaces/Time
payload_json: string
source: string
```

Purpose:
- debug
- logging
- firmware bring-up
- temporary compatibility

### 5B. Normalized robot feedback

Topic:
- `/robot/state/feedback`

Required fields:
- `stamp`
- `source`
- `battery_voltage`

Recommended fields:
- `motion_state`
- `gimbal_state`
- `light_state`
- `fault_flags`
- `raw_available`

Schema:

```yaml
stamp: builtin_interfaces/Time
source: string
battery_voltage: float32
motion_state: string
gimbal_pan: float32
gimbal_tilt: float32
base_light_pwm: uint16
head_light_pwm: uint16
fault_flags: string[]
raw_available: bool
```

Notes:
- `fault_flags` can begin simple and expand later.
- UI, CV, and mux should prefer normalized feedback.
- Only debug or protocol tooling should depend on raw payloads.

## CV Intent Contract

Topic:
- `/cv/control_intent`

Purpose:
- Let CV express what it wants without directly commanding hardware.

Schema:

```yaml
stamp: builtin_interfaces/Time
source: string
intent_type: string
target_present: bool
target_offset_x: float32
target_offset_y: float32
suggested_linear: float32
suggested_angular: float32
suggested_pan: float32
suggested_tilt: float32
confidence: float32
```

Examples:
- `track_target`
- `follow_line`
- `hold_position`
- `request_head_light`

Rule:
- `cv_node` publishes this.
- `command_mux_node` decides whether it becomes final robot actuation.

## State Aggregation Contract

Topic:
- `/system/state/summary`

Purpose:
- Optional aggregated state for UI and dashboards.

Inputs:
- `/robot/state/feedback`
- `/cv/status`
- `/system/state/host`
- `/system/state/command_source`

Why:
- UI does not need to subscribe to many low-level topics.
- Reduces coupling to firmware/hardware details.

## Priority Rules To Preserve

- `command_mux_node` is the single writer for:
  - `/robot/cmd/motion`
  - `/robot/cmd/gimbal`
  - `/robot/cmd/lights`
- `web_bridge_node` must not publish final robot actuation topics.
- `cv_node` must not publish final robot actuation topics.
- `/robot/cmd/raw_json` is not the main API.

## Freeze Recommendation

Freeze these five contracts before building ROS nodes:

1. Motion command
2. Gimbal command
3. Light command
4. CV status
5. Robot feedback

Once frozen:
- build `robot_base_node`
- move mux to ROS-side publishing
- split CV into perception + intent
- move web to a stateless ROS bridge
