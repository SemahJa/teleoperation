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
The enabled Hand calibrates its empty fingers at startup. For mock grasp only,
set `hand.grasp_width_m: 0.006`, `epsilon_inner_m: 0.001`,
`epsilon_outer_m: 0.002`; these are software examples, not object settings.

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

- Fresh trigger release/press enables only the assigned arm. Input loss, focus
  loss, recenter and USB disconnect disable motion; held trigger cannot resume
  until a valid release/press. The first target anchors at the measured pose.
- Normal trigger release captures all seven measured joint positions once and
  selects `reason: position_hold` with `motion_enabled: false`. The reference
  stays fixed until re-engagement, homing or a fault. Stiffness ramps over
  `hold_stiffness_ramp_seconds` (default 0.25 s); `hold_stiffness` and
  `hold_damping` are seven-element joint gains. Startup captures the initial
  configuration. Faults/stale input and shutdown retain damping behavior.
  The mock is fixed, so restoring torque under displacement is verified in
  controller tests rather than inferred from mock motion.
- With trigger released, hold right A/left X at least one second then release.
  Only its assigned arm requests home. The fixed mock is already at home and
  settles quickly. Trigger/input loss cancels through stopping with no restart.
- Wait for Hand calibration completion. First deliberate grip press opens;
  later release/press toggles grasp/open with configured object settings. Holding
  does not repeat, release alone does not open, busy presses are discarded.
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

## Diagnosing a slipping cable grasp

The Hand's force is independent of arm torque clipping. The worker passes
`grasp_force_n`, `grasp_width_m` and both epsilon values directly to
`franka::Gripper::grasp`; it does not reduce the force or silently substitute a
position-only closing command. Values are loaded at worker startup: editing a
YAML file does not update an already running worker. Restart the normal launch
to apply edits; parameter changes through `ros2 param set` are not a supported
live tuning interface.

In a sourced ROS container, inspect the effective settings and completion events:

```bash
ros2 param get /panda_1/franka_hand_worker grasp_force_n
ros2 param get /panda_1/franka_hand_worker grasp_width_m
ros2 param get /panda_1/franka_hand_worker epsilon_inner_m
ros2 param get /panda_1/franka_hand_worker epsilon_outer_m
ros2 topic echo /panda_1/gripper_events --qos-durability transient_local
```

After calibration, the first deliberate grip press opens; a subsequent fresh
press grasps. Check `busy: false`, `state: grasped` and `message: grasp completed`
after completion. A failed operation leaves `state: unknown` and its diagnostic
message; the next deliberate press opens rather than retrying automatically.

`grasp_width_m` is the expected object width between the fingertips, not a
generic command to close as tightly as possible. It is the full opening between
the opposing fingers, not the distance from the center to one finger: an
expected 3 mm gap corresponds to `0.003` m, not `0.0015` m. Exact equality with
the object's final width is not required; epsilon values allow deviations.
Force, width and tolerances are sent together in a single `grasp` call; this
worker does not issue a separate force command after the success result.
Success is not a measurement of the exact force at the cable contact.

With width `0.001` and epsilon
values `0.0005`, the accepted gap is `0.0005 < d < 0.0015` m. A thicker cable
can be contacted while the grasp still fails that check.
With epsilon values `0.00005` instead, that same target accepts only
`0.00095 < d < 0.00105` m (0.95--1.05 mm). Smaller tolerances do not increase
clamping force; they make grasp acceptance stricter. Measure the cable's
diameter and account for compression when choosing width/tolerances; widening
the tolerance is not a way to increase force. Semantics follow the pinned
libfranka header and [Franka's API documentation](https://frankarobotics.github.io/libfranka/0.15.0/classfranka_1_1Gripper.html).

Startup prints `Hand grasp settings`; each actual grasp prints its requested
width/force/speed/tolerances, and completion prints success, measured width and
`is_grasped`. These are command values and a completion snapshot, not measured
contact force or continuous slip detection. `gripper_events.width` is also the
last operation's readback, not a continuous width stream.
Retain the launch output and the actual runtime YAML with each experiment:
`gripper_events` does not store the issued force or grasp tolerances, so an old
bag alone cannot confirm those settings.

If the grasp succeeds yet the cable slides with standard fingers, inspect
whether both fingers actually pinch the cable, whether the fingers touch each
other or reach a mechanical stop first, and the contact surface/geometry. The
available friction depends on normal contact forces and surface friction; a
requested 30 N is not proof of 30 N being applied at the cable contact. Thin,
smooth or deformable cables can require more suitable contact surfaces or
fingertip geometry. Calibrate again after changing fingers, as required by the
API. The current software has no force readback that can determine the friction
coefficient or the physical cause of a slip. Do not infer that increasing the
force alone will solve a width/contact problem.

Physical operation requires hardware backend in YAML, actual IPs and explicit
`operator_confirm:=true`, plus installation commissioning. Hardware dual requires
both surveyed transforms under `arm_settings`; the mock uses internal fixture
spacing only. No physical command is part of these checks. Timing, fallback,
geometry and singularity limitations remain in [CBF_PANDA.md](CBF_PANDA.md).

## Startup and stop diagnostics

| Symptom | Meaning and next check |
| --- | --- |
| Missing `runtime_config` | Pass the complete selected YAML path; combined bringup has no default file. |
| Hardware confirmation rejected | The selected file has `pipeline.backend: hardware`; hardware requires the explicit launch argument `operator_confirm:=true`. For software tests, select mock in YAML. |
| `Move command rejected` with mode `Reflex` | The robot rejected entry into torque control. Inspect the startup `current_errors` and `last_motion_errors`, and restore readiness through the operator's normal robot procedure. The runtime does not recover errors automatically. |
| `reason: position_hold`, `motion_enabled: false` | Normal trigger release: joint holding is active; this is not a controller failure. |
| Input fault or stale command | Check the raw input, mapped command, clock validity and bridge connection; recovery requires a fresh valid trigger release/press. |
| `Latched CBF stop` | Inspect the named cause and timing/model diagnostics. This can occur with CBF disabled because model, actuator and deadline checks remain active. See [CBF_PANDA.md](CBF_PANDA.md). |
| No saved trajectory report data | A live PlotJuggler window does not record topics. Start/finalize an episode and check that it includes `tracking_samples`; see [TRAJECTORY_MONITORING.md](TRAJECTORY_MONITORING.md). |

Controller or Hand process failure stops combined bringup. Keep its terminal
output when investigating a failure. Parameters are loaded at startup; edit
the selected YAML and restart to apply changes.
