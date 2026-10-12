# Quest USB telemetry protocol v1

The Quest APK connects to the PC listener through `adb reverse`. TCP carries a sequence of frames:

```text
uint32 payload_length (network byte order)
UTF-8 JSON payload (exactly payload_length bytes)
```

Maximum payload is 4096 bytes. Reject zero/oversized lengths, malformed JSON, unsupported versions/types, invalid arrays/scalars, and out-of-order input. A TCP connection carries exactly one OpenXR session; reconnect starts a new session ID and sequence space. Never replay buffered commands after reconnect.

Clock synchronization uses periodic Quest-originated request/reply frames on the same TCP connection. Quest records `q1` immediately before sending; PC records `p2` after receiving the complete request and `p3` immediately before replying; Quest records `q4` on receipt. All timestamps are monotonic nanoseconds in their respective device clocks. Estimate PC-minus-Quest offset as `((p2-q1)+(p3-q4))/2`, and network delay as `(q4-q1)-(p3-p2)`. Reject negative delay. Uncertainty is estimated as half the network delay plus 0.1 ms, assuming approximately symmetric path delay; this is not a hard bound. The estimate grows at 50 ppm until the next sync. Initial gate: <=10 ms uncertainty and sync age <=3 s. A sync timeout invalidates the estimate and forces disarm/reconnect.

Unsigned decimal-string fields accept digits only; signs, whitespace, overflow,
and nondecimal text are rejected. Clock offsets accept an optional leading
minus sign. The bridge rejects input fields belonging to the opposite hand
or unexposed buttons instead of silently accepting them.

## ROS button representation

The TCP JSON schema below is unchanged. On ROS topics, `QuestControllerState`
now contains a bounded `QuestButtonState[] buttons` list instead of fixed
A/B/X/Y/Menu/System fields. Right messages contain `a`, `b`; left messages
contain `x`, `y`, `menu`. Each entry preserves `pressed` and `active`
independently. `active=true` means the OpenXR action is available, including
when the button is released; it does not mean the button is pressed.
Disconnect notifications have an empty list and `connection_valid=false`.

Example while holding the right A button:

```yaml
controller_id: right
buttons:
- name: a
  pressed: true
  active: true
- name: b
  pressed: false
  active: true
```

This is a ROS message-schema change. Rebuild `quest_teleop_interfaces` and
`quest_teleop`, and restart the bridge and subscribers against the updated
workspace. Existing APKs already send hand-specific fields and remain compatible.

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
    "button_a": false,
    "button_a_active": true,
    "button_b": false,
    "button_b_active": true
  }
}
```

Inputs contain only controller-specific buttons: right packets carry A/B; left packets carry X/Y/menu. The system/meta button is not exposed by the current app binding. The `*_active` flags preserve each OpenXR action's `isActive` state separately from its value; inactive actions must not be interpreted as a released button. Raw samples retain the shared `~/controller_state` topic and are also mirrored to `~/left/controller_state` and `~/right/controller_state` for per-controller recording/display. Timestamps and sequence are decimal strings to avoid JSON implementations losing 64-bit integer precision. The bridge computes `source_time_pc_monotonic_ns = acquisition_time_ns + clock_offset_pc_minus_quest_ns`; `source_time_valid` is true only when synchronization is fresh, uncertainty is within threshold, and converted source time is neither >10 ms in the future nor older than the configured 100 ms command age. `reference_space_generation` increments when the LOCAL space changes/recenters; the receiver must invalidate anchors and require a fresh trigger release/re-press. The two timestamp domains have distinct meanings: `acquisition_time_ns` is Quest monotonic sample acquisition time; `openxr_location_time_ns` is the OpenXR space-location time. Use converted Quest acquisition time for command freshness; preserve OpenXR location time for pose/twist interpretation. OpenXR location time remains in its runtime clock domain and must not be directly compared to PC monotonic time.
