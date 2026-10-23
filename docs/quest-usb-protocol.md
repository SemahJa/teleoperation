# Quest USB telemetry protocol v1

The Quest APK connects to the PC listener through `adb reverse`. TCP carries a sequence of frames:

```text
uint32 payload_length (network byte order)
UTF-8 JSON payload (exactly payload_length bytes)
```

Maximum payload is 4096 bytes. Reject zero/oversized lengths, malformed JSON, unsupported versions/types, invalid arrays/scalars, and out-of-order input. A TCP connection carries exactly one OpenXR session; reconnect starts a new session ID and sequence space. Never replay buffered commands after reconnect.

## Clock synchronization

Quest periodically sends a request on the same TCP connection. Both ends keep
their own monotonic clocks, in nanoseconds:

| Time | Event |
| --- | --- |
| `q1` | Quest immediately before sending |
| `p2` | PC after receiving the complete request |
| `p3` | PC immediately before replying |
| `q4` | Quest on receipt of the reply |

```text
PC-minus-Quest offset = ((p2-q1) + (p3-q4)) / 2
Network delay = (q4-q1) - (p3-p2)
```

Negative delay is rejected. Estimated uncertainty is half the network delay
plus 0.1 ms and grows at 50 ppm until the next synchronization. This assumes
approximately symmetric transport delay; it is not a hard bound. Initial gates
are uncertainty at most 10 ms and synchronization age at most 3 s. A sync
timeout invalidates the estimate and forces disarm/reconnect.

Unsigned decimal-string fields accept digits only; signs, whitespace, overflow,
and nondecimal text are rejected. Clock offsets accept an optional leading
minus sign. The bridge rejects input fields belonging to the opposite hand
or unexposed buttons instead of silently accepting them.

## ROS button representation

On ROS topics, `QuestControllerState` contains a bounded (maximum four)
`QuestButtonState[] buttons` list. Updated APK packets produce `thumbstick`,
`a`, `b` for the right controller and `thumbstick`, `x`, `y`, `menu` for the left.
Each entry preserves `pressed` and `active`
independently. `active=true` means the OpenXR action is available, including
when the button is released; it does not mean the button is pressed.
Disconnect notifications have an empty list and `connection_valid=false`.

Example while holding the right A button:

```yaml
controller_id: right
buttons:
- name: thumbstick
  pressed: false
  active: true
- name: a
  pressed: true
  active: true
- name: b
  pressed: false
  active: true
```

This is a ROS message-schema change. Rebuild `quest_teleop_interfaces` and
`quest_teleop`, and restart the bridge and subscribers against the updated
workspace. Updated APKs send the joystick click/value availability pair as well. Older
packets without both fields remain accepted, but have no joystick button and
cannot request gripper opening. A partial pair is malformed and invalidates input.
Install the newly built APK for joystick opening; stop/restart all affected ROS
nodes after rebuilding the message and bridge.

Request:

```json
{"version":1,"type":"clock_sync_request","sync_id":"7","quest_send_time_ns":"1234567890"}
```

Reply:

```json
{"version":1,"type":"clock_sync_reply","sync_id":"7","quest_send_time_ns":"1234567890","pc_receive_time_ns":"1234567990","pc_send_time_ns":"1234568000"}
```


Controller input JSON schema (all arrays use metres, radians, m/s, rad/s; quaternion order is x,y,z,w):

```json
{
  "version": 1,
  "type": "controller_input",
  "session_id": "opaque-session-id",
  "sequence": "123",
  "acquisition_time_ns": "1234567890",
  "openxr_location_time_ns": "1234567890",
  "clock_sync_valid": true,
  "clock_offset_pc_minus_quest_ns": "-5000000",
  "clock_uncertainty_ns": "2000000",
  "clock_sync_age_ns": "100000000",
  "reference_space": "LOCAL",
  "reference_space_generation": "0",
  "controller_id": "right",
  "session_focused": true,
  "pose": {
    "pose_action_active": true,
    "position": [0.0, 0.0, 0.0],
    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    "position_valid": true,
    "orientation_valid": true,
    "position_tracked": true,
    "orientation_tracked": true
  },
  "native_twist": {
    "linear": [0.0, 0.0, 0.0],
    "angular": [0.0, 0.0, 0.0],
    "linear_valid": true,
    "angular_valid": true
  },
  "inputs": {
    "trigger": 0.0,
    "trigger_active": true,
    "grip": 0.0,
    "grip_active": true,
    "thumbstick_click": false,
    "thumbstick_click_active": true,
    "button_a": false,
    "button_a_active": true,
    "button_b": false,
    "button_b_active": true
  }
}
```

Inputs contain only controller-specific buttons: right packets carry A/B;
left packets carry X/Y/menu. Both additionally carry the assigned joystick click
as `thumbstick_click` and `thumbstick_click_active` (optional only for old APKs). The System/Meta button is not exposed. The
`*_active` flags preserve OpenXR availability separately from each value;
an inactive action must not be interpreted as a released button.

Raw samples publish on shared `~/controller_state` and hand-specific
`~/left/controller_state` and `~/right/controller_state`. With the default bridge
node name these resolve under `/quest_usb_bridge`.

Timestamps and sequence are decimal strings to preserve 64-bit integer
precision. The bridge computes:

```text
source_time_pc_monotonic_ns = acquisition_time_ns + clock_offset_pc_minus_quest_ns
```

`source_time_valid` requires fresh synchronization, acceptable uncertainty and
a converted source time within the configured age/future gates (100 ms old /
10 ms future by default). `reference_space_generation` changes on LOCAL-space
recenter; the receiver clears anchors and requires a fresh trigger release/press.

`acquisition_time_ns` is Quest monotonic sample acquisition time.
`openxr_location_time_ns` is OpenXR space-location time, which may include
prediction. Use converted acquisition time for freshness and retain location
time for pose/twist interpretation. Never compare the raw OpenXR clock directly
with PC monotonic time. This protocol sends pose and native twist, not raw
accelerometer data, acceleration or jerk.
