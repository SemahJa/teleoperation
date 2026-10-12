# Integrated controller launch and Menu recording

Run these checks with `backend:=mock`. The mock has fixed joint/Cartesian state
and synthetic inertia; it verifies software behavior, not robot motion. Reuse
the validated APK and USB bridge. Stop older mock controllers, Hand workers and
`robot_free.launch.py` mappers with Ctrl+C before starting this integrated launch,
so each arm has one controller, mapper and Hand worker.

## Terminals and launch commands

On the **PC host**, open a ROS container terminal for each command group:

```bash
cd /home/jiachun/franka_teleoperation
docker compose -f docker/compose.yaml run --rm panda-ros2
```

At the start of **each ROS container terminal**:

```bash
cd /ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

In **container terminal 1**, launch the default single-right mock:

```bash
ros2 launch franka_teleop_controller cartesian_impedance_controller.launch.py \
  backend:=mock mode:=single start_bridge:=false
```

If the bridge is not already running, use `start_bridge:=true` instead; retain
the existing ADB reverse procedure in `docs/DEVELOPMENT.md`. Do not run
two bridges on port 45454. To restore the existing USB mapping, on the **PC
host in `/home/jiachun/franka_teleoperation`** run:

```bash
bash scripts/quest_adb_reverse.sh 45454
```
 The launch starts the controller, its pose mapper,
separate Hand worker, and independent left Menu recorder. Neither homing,
Hand calibration nor recording starts automatically.

Stop that launch with Ctrl+C before trying **single-left**:

```bash
ros2 launch franka_teleop_controller cartesian_impedance_controller.launch.py \
  backend:=mock mode:=single single_controller:=left start_bridge:=false
```

Stop that launch before trying **dual** with the supplied two-arm fixture:

```bash
ros2 launch franka_teleop_controller cartesian_impedance_controller.launch.py \
  backend:=mock mode:=dual start_bridge:=false \
  arms_config:=/ws/src/quest_teleop/config/arms_two_mock.yaml
```

A dual configuration must contain exactly one left-assigned and one
right-assigned arm, with distinct namespaces/base frames/IPs. Single mode with a
two-arm configuration selects the arm assigned to `single_controller`; with a
one-arm default configuration it assigns that arm to the chosen hand. Custom raw
topics must explicitly match the chosen assignment. Dual mode runs one process
with two exclusive arm owners and one coupled CBF coordinator.

## Observe and check behavior

In **container terminal 2**, use one echo command at a time (Ctrl+C switches):

```bash
ros2 topic echo /panda_1/controller_status
ros2 topic echo /panda_1/homing_status
ros2 topic echo /panda_1/cbf_status
ros2 topic echo /panda_1/gripper_events
ros2 topic echo /quest/recording_status
```

For the two-arm fixture substitute `/panda_left` or `/panda_right` for
`/panda_1`. To see which nodes/topics were actually launched:

```bash
ros2 node list
ros2 topic list
```

On the **headset**:

- Release and press the assigned trigger: only the selected arm enables. USB
  disconnect/tracking loss disarms and recovery requires release/press.
- With trigger released, hold X on the left or A on the right for at least
  one second, then release. Only its assigned arm requests home. The fixed
  mock is already at home, so it reports homing then succeeded. Trigger press
  cancels through stopping; held trigger cannot resume after cancellation.
- Release left Menu before its first press. Press Menu once to start a bag;
  holding does not repeat. Release and press again to stop/finalize. Presses
  within 350 ms are debounced. Startup-held, stale, duplicate, inactive-button
  and reconnected-held inputs cannot toggle until a fresh release.
- Menu works in single-right mode without any left-arm mapper/controller.
  Valid button/focus/connection/source-clock metadata is required; left pose
  tracking is not needed for recording. USB/input loss leaves a started bag
  running. A new valid release/press can stop it after reconnect.
- Normal launch shutdown finalizes an active bag. Wait for recording status
  `idle` with message `bag finalized` before starting the next episode.

For **Hand mock calibration**, run in container terminal 2:

```bash
ros2 service call /panda_1/calibrate_gripper quest_teleop_interfaces/srv/CalibrateGripper \
  '{empty_hand_confirm: true, operator_confirm: true}'
```

Observe actual calibration completion in `gripper_events`. Fresh grip presses
then toggle open/grasp; grasp requires configured object width and tolerances.
The supplied runtime config leaves those unknown, so grasp is refused until
specified. Mock-only example settings:

```bash
mkdir -p /ws/log
cp /ws/src/franka_teleop_controller/config/teleoperation_runtime.yaml /ws/log/mock_runtime.yaml
```

Edit the `hand` section of `/ws/log/mock_runtime.yaml` to set
`grasp_width_m: 0.006`, `epsilon_inner_m: 0.001`, `epsilon_outer_m: 0.002`.
Restart the mock launch with `runtime_config:=/ws/log/mock_runtime.yaml`.
These are test values, not object commissioning settings. The first deliberate
post-calibration press opens; subsequent release/press grasps, then opens.

## Customize recording and inspect bags

`recording_config` defaults to `quest_teleop/config/recording.yaml`. Its default
profile derives active-arm topics, plus independent left Menu input. Edit its
`topics` list to extend selection, or set `include_default_topics: false` to
replace it. At each start, the recorder selects only currently published topics
from that list and excludes namespaces of inactive configured arms. Topics
absent at start are omitted for that episode; a later episode re-evaluates the
graph. A complete JSON override is also supported:

```bash
ros2 launch franka_teleop_controller cartesian_impedance_controller.launch.py \
  backend:=mock mode:=single \
  recording_topics:='["/panda_1/controller_status","/panda_1/cbf_status","/panda_1/robot_state"]'
```

`recording_output_root` overrides the absolute output root, default
`/ws/log/quest_bags`, which is `/home/jiachun/franka_teleoperation/log/quest_bags`
on the host. Every episode gets a unique path. `recording_namespace` defaults to
`/quest`; `left_controller_state_topic` overrides the independent Menu input.
Use `recording_enabled:=false` to omit the Menu recorder, or `start_hand:=false`
to omit Hand workers. ROS Humble's existing sqlite3 plugin is used.

After stopping recording, in **container terminal 2** inspect the exact path
reported in recording status:

```bash
ros2 bag info /ws/log/quest_bags/REPLACE_WITH_REPORTED_EPISODE
```

Recorder startup/finalization failures appear in recording status and the
adjacent `.recorder.log`. An empty publisher intersection refuses to start.
No unselected topic is added automatically.

## Configuration and commissioning boundary

`arms_config` controls assignment, namespaces, IPs, mapper basis/scales, topics
and clock gates. `runtime_config` controls controller/Hand/CBF settings; its
`arm_settings` keys are configured arm names, with optional `controller`, `hand`
and `world_from_base_xyz_xyzw` overrides. Ownership/assignment/topic identities
cannot be overridden from gain settings. Both hardware dual base transforms
must be supplied explicitly. Unknown configuration settings are rejected.

Hardware still requires explicit backend, IPs and operator confirmation; no
hardware command is part of these checks. Survey home paths, base/table/tool/
payload and collision geometry, choose object-specific Hand settings and gains,
and verify deployment compatibility/RT/worst-case timing/stop behavior before
physical use. CBF fallback and singularity limitations remain in `CBF_PANDA.md`.
