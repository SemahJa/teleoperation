# Integrated runtime and robot-free checks

Overall bringup requires a complete runtime file and exposes only
`runtime_config` and `operator_confirm`. Package-local configs remain the defaults
for standalone features. Configuration/editing and operational commands are in
[README.md](../README.md); schema and ownership are in
[PACKAGE_STRUCTURE.md](PACKAGE_STRUCTURE.md).

For these checks, keep `pipeline.backend: mock`. Mock state is fixed and has
synthetic dynamics; it does not prove physical clearance or realtime timing.
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

Physical operation requires hardware backend in YAML, actual IPs and explicit
`operator_confirm:=true`, plus installation commissioning. Hardware dual requires
both surveyed transforms under `arm_settings`; the mock uses internal fixture
spacing only. No physical command is part of these checks. Timing, fallback,
geometry and singularity limitations remain in [CBF_PANDA.md](CBF_PANDA.md).
