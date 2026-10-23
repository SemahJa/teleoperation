# Integrated runtime and robot-free checks

Overall bringup requires a complete runtime file and exposes only
`runtime_config` and `operator_confirm`. Package-local configs remain the defaults
for standalone features. Configuration/editing and operational commands are in
[README.md](../README.md); schema and ownership are in
[PACKAGE_STRUCTURE.md](PACKAGE_STRUCTURE.md).

For these checks, set `pipeline.backend: mock` in the selected file first; the
current single-arm working configuration selects hardware. Mock state is fixed
and does not simulate physical dynamics, clearance or realtime timing.
Reuse the validated APK/USB setup. Stop earlier launches before starting another
owner/mapper/bridge on the same namespace or port.

## Launch selection

In a sourced ROS container at `/ws`, select single or dual explicitly:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch teleop_bringup teleoperation.launch.py \
  runtime_config:=/ws/src/teleop_bringup/config/single_panda.yaml
```

Stop that launch before using the dual template:

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  runtime_config:=/ws/src/teleop_bringup/config/dual_panda.yaml
```

Set single-left selection in the single file's `pipeline.single_controller`.
Set bridge/Hand/recording/home/CBF switches in `pipeline`, not command arguments.
The enabled Hand calibrates its empty fingers at startup. Configure `hand.speed_mps`
for closing, `open_speed_mps` for opening, and `close_width_m`/`open_width_m` for
travel. All widths are full gaps between the fingers.


## Observe behavior

In a second sourced container shell, run one observer at a time:

```bash
ros2 topic echo /panda_1/controller_status
ros2 topic echo /panda_1/cartesian_teleop_command
ros2 topic echo /panda_1/homing_status
ros2 topic echo /panda_1/gripper_events
ros2 topic echo /panda_1/cbf_status
ros2 topic echo /quest/recording_status
```

For dual substitute `/panda_left` or `/panda_right` for `/panda_1`.

Startup logs distinguish three separate stages for each arm:

- `FCI startup ... mode="Idle"` confirms a state read before requesting torque
  control. It does not confirm that torque callbacks have started.
- `Arm control ready: accepted torque cycle=...` confirms that the arm produced
  a current-cycle, safety-checked torque and measured-state telemetry. An arm
  that still has no accepted cycle after five seconds reports a warning with
  `control_requested`; this is diagnostic only and does not clear robot faults
  or restart control.
- `Hand calibration ready: robot_ip=... maximum_gap_m=...` confirms that the
  Hand's calibration command completed successfully with valid readback.
  Release the assigned Quest side grip after this message, then squeeze it.

These stages are independent of Quest engagement. Arm startup holds the
current configuration; it does not automatically move the arm home. The
`Quest input`, `Mapped input` and `Hand input` log lines report changes in input
availability and release gates, limited to one line per type every 500 ms.
`Hand input: release_required=1` means a fresh
valid side-grip release is needed, including after calibration. An inactive
grip action cannot provide that release. Arm engagement uses the trigger,
not the side grip, and additionally needs fresh tracked pose input.

Desk's Hand panel may show **end-effector not connected** during FCI control.
[Franka's maintainers describe this as expected](https://github.com/frankarobotics/franka_ros/issues/118):
Desk's control path and FCI are distinct. Use the worker's calibration-ready
message and `gripper_events` to diagnose our Hand connection. A successful
firmware `Hand result` followed by a software rejection is a different fault;
it now produces an immediate error with raw width, calibrated maximum width,
endpoint excess and communication status. A fixed **10 micrometre** readback
allowance accepts small endpoint discrepancies and clamps the internal width
to `[0, maximum_width]`. This is a software policy, not a manufacturer accuracy
specification. It never increases commanded travel. Nonfinite values, invalid
maximum width, larger discrepancies and communication failures still require
explicit calibration; none automatically reconnect or retry calibration.

Measured-pose feedback and Quest callbacks run separately from bulk status
publication. `teleop_anchor_pose` publishes at up to 100 Hz with **best-effort,
depth-one QoS**; the mapper requests the same QoS. A slow status subscriber must
not starve these callbacks. Other custom subscribers to the anchor topic must
request best-effort reliability, for example:

```bash
ros2 topic echo /panda_right/teleop_anchor_pose --qos-reliability best_effort
```

The 100 ms input/pose freshness gates remain enforced. Large `source_age_ms`
values indicate delivery delay; increasing stiffness will not clear this gate.
The full-cycle tracking stream retains its separate bounded queue and publisher.

For an unresponsive right arm, in a sourced **ROS container terminal**:

```bash
timeout 3s ros2 topic echo /panda_right/controller_status --once
timeout 3s ros2 topic echo /panda_right/gripper_events --once --qos-durability transient_local
timeout 3s ros2 topic echo /quest_usb_bridge/right/controller_state --once
```

A controller status sample confirms callback output; no sample means input
gating alone has not been established as the cause. Inspect the readiness
messages before changing gains or bypassing release gates. `calibrated: true`
with a nonzero maximum width is the Hand's reported calibration result;
absence of later close/open requests does not itself prove calibration failed.

- Fresh trigger release/press enables only the assigned arm. Input loss, focus
  loss, recenter and USB disconnect disable motion; held trigger cannot resume
  until a valid release/press. The first target anchors at the measured pose.
- Normal trigger release captures all seven measured joint positions once and
  selects `reason: position_hold` with `motion_enabled: false`. The reference
  stays fixed until re-engagement, homing or a fault. Stiffness ramps over
  `hold_stiffness_ramp_seconds` (default 0.25 s); `hold_stiffness` and
  `hold_damping` are seven-element joint gains. Startup and pauses retain a
  fixed local joint anchor even with missing/invalid/stale Quest input. If input
  fails during engagement, capture measured joints once and hold there instead
  of pursuing the old Quest target. Repeated faults and release messages cannot
  move that anchor or reset its stiffness ramp. Fresh release/re-engagement is
  still required. Safety fallback, invalid-state termination, homing priority
  and shutdown retain their separate stop behavior.
  The mock is fixed, so restoring torque under displacement is verified in
  controller tests rather than inferred from mock motion.
- With trigger released, hold right A/left X at least one second then release.
  Only its assigned arm requests home. The fixed mock is already at home and
  settles quickly. Trigger/input loss cancels through stopping with no restart.
- Wait for Hand calibration, then release the assigned Quest grip. Hold grip
  above `grip_press` to close toward `close_width_m` at `speed_mps`; release
  below `grip_release` sends stop without release debounce. One press of the assigned joystick
  opens automatically to `open_width_m` (`-1` uses calibrated maximum).
  Releasing the joystick or side grip does not cancel this opening. Input faults
  and watchdog expiry stop opening/closing and require a fresh release/press.
- Fresh left Menu presses toggle unique recording episodes, even in single-right
  mode. Holding does not repeat; observe recording then idle/bag finalized.
  Input loss leaves a started bag running; normal shutdown finalizes it.

For deliberate mock recalibration after startup:

```bash
ros2 service call /panda_1/calibrate_gripper \
  quest_teleop_interfaces/srv/CalibrateGripper \
  '{empty_hand_confirm: true, operator_confirm: true}'
```

Recording profile, namespace, debounce and output folder live under `recording`
in the runtime file. Only selected published topics are included at episode start;
inactive arm topics are excluded in Menu mode. Inspect the reported finalized path:

```bash
ros2 bag info /ws/log/quest_bags/REPLACE_WITH_REPORTED_EPISODE
```

## Hand position control and joystick opening

The gripper uses `franka::Gripper::move(width, speed)` for closing and opening,
with a separate `stop()` worker for cancellation. Holding the Quest grip starts
a move toward the lower width bound; longer holds permit more travel. Release
requests a stop immediately on receipt of a fresh released input. The ROS
executor never waits for the blocking move to finish before requesting stop.
Input sampling, USB/ROS delivery, robot communication and mechanical deceleration
still introduce latency: stopping is not physically instantaneous.

```yaml
hand:
  speed_mps: 0.05             # Closing speed; existing operator value is preserved.
  open_speed_mps: 0.1         # Opening speed, independently adjustable.
  close_width_m: 0.0
  close_retry_interval_ms: 200.0
  worker_poll_ms: 1.0
  open_width_m: 0.025
  grip_press: 0.75
  grip_release: 0.25
  grip_debounce_ms: 0.0       # Optional closing debounce, 0..100 ms.
```

A single **joystick press** opens on the rising edge. There is no duration test,
second click or wait for joystick release. Each joystick must first be observed
active and released after calibration/reconnection/fault; holding it at startup
cannot open the Hand. Holding it down afterward cannot repeat the command.
If closing is active, the press cancels held-close refreshes, sends stop, and
opens after the prior move/stop finishes with valid readback. A side grip that
was already held cannot immediately reverse that opening: release it, then
press again to close. Side-grip release alone stops closing; it does not cancel
a joystick-requested opening. Input faults stop ordinary opening/closing.

The updated native APK sends `thumbstick_click` and `thumbstick_click_active`;
the bridge exposes the assigned button as `buttons[name=thumbstick]`. Older
APKs remain usable for closing and arm control but cannot request opening.
Rebuild/install the APK using [the host instructions](../README.md#3-start-the-quest-app-over-usb)
and restart ROS nodes against the updated four-button message bound. No APK
installation or physical motion is performed by the robot-free checks.
The old `grip_click_max_ms`/`grip_double_click_ms` keys are compatibility-only and
have no effect. Settings are loaded at startup and support per-arm `hand` overrides.

Holding grip maintains a persistent request to close to `close_width_m`.
`close_retry_interval_ms` is the refresh period, measured from the start of
each close request, rather than a delay started by failure. Refresh continues
after success, an unsuccessful move, or a libfranka command exception with valid
width readback; the API does not identify contact as the cause. Each request
uses the same target and speed. While a move is still running it already owns
the closing goal, so periodic checks leave it uninterrupted. If its refresh is
overdue when it ends, the next worker check can dispatch another request.
Calls never overlap and missed periods are not replayed as a burst.

Reaching the lower bound also keeps the held demand active: later refreshes
read live width, skip redundant hardware moves while already closed, and close
again if a nonzero gap reappears. Release cancels future refreshes and stops an
active close; another deliberate press can close again. Input, transport,
invalid-state and stop faults cancel the demand; communication/state/stop faults
require recalibration. Set the period to `0` to disable refreshing, otherwise
use 50--10000 ms. The default 200 ms permits at most about five refresh requests
per second; long-running moves reduce the actual dispatch rate. No separate
release/press is needed between ordinary attempts.

Repeated attempts do not set or guarantee clamping force. There is no automatic
force increase, force-controlled grasp or tactile slip detection. Correcting
a reopened width is not a measurement of clamping force. The legacy
`grasp_force_n`, `grasp_width_m`, `epsilon_inner_m` and `epsilon_outer_m` parameters
are accepted for old configuration compatibility but are unused by this mechanism.
Use a measured/contact-appropriate lower width bound for your experiment;
position closing does not guarantee a specified cable clamping force or allow
zero width through an object. Every new close checks live device width; the
last completion snapshot cannot reject a command or cause a close to open.

Observe status in a sourced ROS container:

```bash
ros2 topic echo /panda_1/gripper_events --qos-durability transient_local
ros2 param get /panda_1/franka_hand_worker close_width_m
ros2 param get /panda_1/franka_hand_worker open_width_m
```

For dual arms substitute `/panda_left` or `/panda_right`. `operation` identifies
`close`, `stop`, `open`, `calibrate` or `idle`; completion states include `stopped`,
`closed`, `open` and `unknown`. `closed` means the close move completed, not that
an object is securely grasped. Grip requires connection, focus, available grip
action and valid fresh clocks; Cartesian pose tracking is not required for
finger control. Width remains a completion snapshot rather than a continuous
force/position stream. Calibration remains independent of Quest
input; input loss stops ordinary moves, while shutdown also requests calibration
cancellation. Communication/stop faults require explicit calibration before
further movement. No new motion starts until both the preceding move and its
stop have completed.

### When joystick opening and further closing both stop working

Check `gripper_events.calibrated`, `busy` and `message` first. A stop,
communication or invalid-state fault clears calibration and disables both
opening and closing. Fresh Quest packets, manually changing the finger gap,
or releasing/re-pressing grip cannot clear that fault. The original backend
error remains in `message` until an explicit calibration starts.

For the 2026-10-23 right-Hand trial, joystick presses arrived with valid input
and less than 1 ms source age, but the preceding SDK stop call took 9124.666 ms
and the worker had already cleared calibration. The older generic error did
not retain the stop response, so it cannot establish whether that particular
stop failed at the device or hit the software acknowledgement race described
below. Preserved logs are under
`log/diagnostics/right_hand_open_review_20261023/`.

Once the worker is idle, remove any object and prepare for full finger travel.
In a sourced **ROS container terminal**, explicitly recalibrate the affected
right Hand; this command moves the fingers:

```bash
ros2 service call /panda_right/calibrate_gripper \
  quest_teleop_interfaces/srv/CalibrateGripper \
  '{empty_hand_confirm: true, operator_confirm: true}'
```

Wait for `calibrated: true`, then release both side grip and joystick before
trying again. Use `/panda_left` for the left Hand or `/panda_1` for single-arm
bringup. Calibration is never retried automatically after a fault. If a genuine
stop/device fault repeats, retain the new `Hand result` line with its exact
backend error; another joystick click cannot repair the connection.

### Command timing and diagnosing delay

The Hand uses event-driven commands rather than the arm's 1 kHz torque loop.
Quest samples grip once per OpenXR frame; the source frequency follows the
running headset frame loop and is not fixed by a gripper ROS parameter.
Closing starts on the first qualifying sample by default (`grip_debounce_ms: 0`).
An optional debounce applies only to closing; joystick opening uses the press edge. Release below
`grip_release` bypasses that debounce and dispatches stop directly from its
callback, before logging. The subscription retains only the latest queued grip
sample to avoid processing a backlog of held samples before a release.

`worker_poll_ms` controls completion collection, retry dispatch and watchdog
checks: 1 ms by default (1000 Hz), configurable from 1 to 20 ms. It is not a
1000 Hz hardware command stream or an end-to-end stopping guarantee. Opening
waits only for its fresh press and any preceding stop/move to finish; it is
dispatched before completion logging or status publication. Changing
`open_speed_mps` changes opening travel time without increasing closing
speed; it cannot remove firmware acknowledgement or mechanical stop latency.
A 25 mm opening at 0.1 m/s takes at least roughly 0.25 s before acceleration and
other overhead, even with prompt command dispatch.
Hand status publishing/logging run in a separate reporting thread with latest-only
status and a bounded diagnostic queue, so a blocked observer cannot hold up input.
Completion polling also has its own thread rather than a high-rate ROS timer;
the short core lock never spans SDK, DDS or console calls.
Cancellation waits for command completion separately from final state readback:
it does not repeatedly send stops while only readback is pending. A potentially
overtaken stop is retried only while the command remains outstanding; the worker
wakes immediately on command return. A successful acknowledgement and command
completion remain valid even when a blocking SDK call has exceeded the two-second
retry window; previously, completion during the final wait could be incorrectly
reported as a stop timeout. The retry window cannot interrupt a blocking SDK
call and does not guarantee a two-second physical stop. Actual unsuccessful stop
responses and transport/protocol faults remain faults and are not retried.
Timing settings are read at launch;
restart after editing YAML.

In a sourced **ROS container terminal**, use your assigned controller's raw
topic (the default right-hand topic is shown):

```bash
ros2 topic hz /quest_usb_bridge/right/controller_state
ros2 topic echo /panda_1/gripper_events --qos-durability transient_local
```

Launch logs report `input_age_ms` at dispatch, `duration_ms` for the blocking
operation, `command_ms`/`readback_ms` for blocking command/readback time,
`stop_call_ms`/`stop_calls` for actual SDK stop calls, and `stop_completion_ms`
from software stop dispatch until both workers have completed and the result
is collected. `stop_acknowledged` distinguishes a successful firmware response
from no acknowledgement; `stop_retry_exhausted` identifies command completion
still outstanding at the end of the retry window. Acknowledgement alone does
not allow another motion: the preceding command and valid readback must also
complete. Failed responses and exceptions retain their original error text.
An input age of `-1`
means no usable source time, for example during startup calibration. Source
age uses clock synchronization and inherits its uncertainty. A large source
age points to acquisition/delivery/executor delay; a small age with a large
stop completion time points to the downstream cancellation/readback path.
These values are software timing evidence, not a measurement of the physical
instant that the fingers stop. The robot-free tests cannot establish real
hardware stopping latency.
Diagnostic lines can be emitted later by the reporting thread: use captured
`dispatch_pc_monotonic_ns`, `collected_pc_monotonic_ns` and duration fields,
not differences between the console log prefixes, to assess command timing.

### Future contact feedback

A fingertip force/tactile sensor can replace blind closing retries with a
contact-aware policy: approach the object, regulate a bounded desired normal
force after contact, and react to measured slip. Sensor calibration, freshness,
force limits and the Hand's available actuation interface must be accounted
for; the current blocking `move()` interface is not a streaming force servo.
Keep that policy in the Hand core/device path, separate from arm torque
control, while retaining release/stop and joystick opening arbitration.
Sending a force/slip cue back to Quest would be a separate haptic-output path;
controller vibration alone does not supply a contact-force measurement.

## Startup and stop diagnostics

| Symptom | Meaning and next check |
| --- | --- |
| Missing `runtime_config` | Pass the complete selected YAML path; combined bringup has no default file. |
| Hardware confirmation rejected | The selected file has `pipeline.backend: hardware`; hardware requires the explicit launch argument `operator_confirm:=true`. For software tests, select mock in YAML. |
| `Move command rejected` with mode `Reflex` | The robot rejected entry into torque control. Inspect the startup `current_errors` and `last_motion_errors`, and restore readiness through the operator's normal robot procedure. The runtime does not recover errors automatically. |
| `reason: position_hold`, `motion_enabled: false` | Local joint holding is active at startup or during a pause, including ordinary Quest input faults. Fresh release/re-engagement is required for teleoperation. |
| Input fault or stale command | Check the raw input, mapped command, clock validity and bridge connection; recovery requires a fresh valid trigger release/press. |
| `Latched CBF stop` | Inspect the named cause and timing/model diagnostics. This can occur with CBF disabled because model, actuator and deadline checks remain active. See [CBF_PANDA.md](CBF_PANDA.md). |
| Dual startup `result_deadline`, cycle 1, CBF disabled | The older coordinator waited for both robots before checking local actuator bounds. The current actuator-only path services arms independently; rebuild and restart. Own-cycle deadlines remain enforced. CBF-enabled dual operation still needs paired states. |
| No saved trajectory report data | A live PlotJuggler window does not record topics. Start/finalize an episode and check that it includes `tracking_samples`; see [TRAJECTORY_MONITORING.md](TRAJECTORY_MONITORING.md). |

Controller or Hand process failure stops combined bringup. Keep its terminal
output when investigating a failure. Parameters are loaded at startup; edit
the selected YAML and restart to apply changes.

The launch log may contain only the process exit and shutdown sequence. The
controller's full ROS log is stored separately in the same container, for
example `/root/.ros/log/cartesian_impedance_controller_PID_TIMESTAMP.log`.
Read the first `Controller stopped:` / `Latched CBF stop:` message and its timing
lines to identify the initiating failure; subsequent Hand stop requests and
Quest disconnection are expected bringup cleanup.
