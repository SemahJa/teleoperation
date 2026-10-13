# Teleoperation package structure

The workspace separates input, arm control, gripper control, safety and recording.
`quest_teleop_interfaces` remains the shared ROS message/service package.
`panda_bringup` and `panda_demos` are unchanged. The Android application is unchanged.

| Package | Implementation | Launch entry points | Configuration |
| --- | --- | --- | --- |
| `quest_teleop` | C++ Quest USB bridge; Python pose mapper, arm configuration and measured-pose fixture | `quest_teleop.launch.py`, `quest_usb_bridge.launch.py`, `pose_mapper.launch.py` | `arms.yaml`; mapper/bridge launch parameters |
| `recording` | Menu episode state machine, rosbag process management and recording topic plans | `recording.launch.py` | `recording.yaml` |
| `teleop_controllers` | Arm runtime/FCI ownership, Cartesian impedance core, homing core and joint trajectory generator | `cartesian_impedance_controller.launch.py`, `homing.launch.py` | `controllers.yaml` |
| `teleop_gripper` | Hand decision state machine and separate blocking-operation worker | `gripper.launch.py` | `gripper.yaml` |
| `safety_filter` | CBF constraint geometry, OSQP filter, coordinator and fixed-size model/handoff utilities | `safety_filter.launch.py` | `safety.yaml` |
| `teleop_bringup` | Feature composition and integration tests | `teleoperation.launch.py`, `robot_free.launch.py` | `single_panda.yaml`, `dual_panda.yaml` |

C++ public headers and namespaces follow their owning package. `safety_filter`
exports its library and pinned OSQP dependency; `teleop_controllers` links that
library. The gripper has no arm-controller or CBF dependency. Recording consumes
Quest messages and shared arm configuration, but its implementation is entirely
in `recording`. No runtime behavior or nominal control equation is changed by
moving these implementations.

## Ownership and execution

The arm process selects impedance or homing nominal torque, then invokes the
safety coordinator/filter. Homing never creates another FCI connection. The
safety package is a linked library with its existing coordinator thread, not a
ROS service exchanging torque at 1 kHz. Its launch loads scoped `cbf.*`
parameters for the arm owner; it intentionally starts no robot process by itself.
Asynchronous CBF execution remains a separate Milestone 2 task.

The Hand process owns only the gripper connection. Its launch performs one
empty-hand startup calibration whenever it starts; combined bringup enables it
with `start_hand:=true`. There is no calibration launch switch. The manual
service remains available for deliberate recalibration after startup. Startup calibration
moves fingers, requires empty fingers, and does not automatically grasp or retry.
Menu recording runs in its own process; continuous
recording starts rosbag directly. Neither belongs to the Quest mapper package.

## Arm runtime and future controller structures

`teleop_controllers/src/cartesian_impedance_controller.cpp` is the executable
runtime and hardware adapter. It loads parameters, consumes atomic Quest
commands and raw homing buttons, owns the exclusive libfranka arm connection,
prepares state/model snapshots, selects Cartesian or homing nominal torque,
hands torque to the safety coordinator, and publishes non-realtime telemetry.
Mock operation uses the same control cores without an FCI connection. It also
manages startup, controlled shutdown and fault diagnostics.

The Cartesian control law is implemented in `control_core.cpp`; the joint
homing law and trajectory are in `homing_core.cpp` and
`joint_motion_generator.cpp`. The Hand worker belongs to `teleop_gripper`.

Future controller structures can reuse this runtime, but it currently selects
one Cartesian law and has no general controller plugin/selection interface.
Before adding alternatives, separate command engagement, freshness and actuator
checks from the nominal torque calculation. A preselected controller strategy
should consume a fixed-size arm state and validated reference and produce
nominal joint torque without allocation or blocking in the 1 kHz callback.
All strategies should retain common homing arbitration, safety filtering,
telemetry and FCI ownership. This is the planned extension, not an implemented
runtime switch.

## Build after migration

The standard workspace uses `build/` and `install/`. Source
`/ws/install/setup.bash` in each new container terminal. The temporary
`install/reorganized` overlay has been retired. Previous generated artifacts
are archived under `.workspace-artifacts-*` with `COLCON_IGNORE`;
commissioning logs remain under `log/commissioning/current`.

For a fresh rebuild, start from a shell sourced only from Humble. The new arm
and Hand packages declare their libfranka dependency, so colcon builds it first
within the same invocation. README.md contains the single-command pipeline build.
Building the entire legacy workspace still requires libfranka to be available
before `panda_bringup`, whose manifest omits that dependency. To build the
teleoperation packages and their declared dependencies:

```bash
cd /ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install --packages-up-to teleop_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DBUILD_EXAMPLES=OFF
source /ws/install/setup.bash
colcon test \
  --packages-select quest_teleop recording safety_filter teleop_controllers teleop_gripper teleop_bringup
colcon test-result --test-result-base build --verbose
```

The old `franka_teleop_controller` package no longer exists in source. Combined
launch commands migrate to `teleop_bringup teleoperation.launch.py`; direct arm
execution migrates to `teleop_controllers cartesian_impedance_controller`; direct
Hand execution migrates to `teleop_gripper franka_hand_worker`.

## Combined launch

Overall bringup has exactly two launch arguments:

| Argument | Default | Effect |
| --- | --- | --- |
| `runtime_config` | None; required | Select complete `single_panda.yaml`, `dual_panda.yaml` or an installation-specific runtime YAML |
| `operator_confirm` | `false` | Required `true` for hardware; explicit authorization for FCI and enabled Hand startup calibration |

The templates live under `teleop_bringup/config` and default to mock operation.
Example:

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  runtime_config:=/ws/src/teleop_bringup/config/single_panda.yaml
```

All pipeline switches, mode/backend, bridge settings, arm identities/mapping,
controller/Hand/CBF settings, per-arm overrides and recording settings come from
the selected YAML. No overall CLI feature overrides or separate config-file
arguments remain. The old `teleoperation_runtime.yaml` template is replaced by
explicit single/dual templates. Package-local config defaults remain unchanged
for standalone launches.

Hardware confirmation is launch-only and unnecessary for mock; selecting a
hardware file alone cannot authorize physical operation. It is not a trigger
arming command or a safety certificate. Enabling Hand retains one empty-hand
startup calibration. Hardware dual still requires distinct IPs and both measured
base transforms; these are deliberately not filled with mock spacing in the
dual template. No synthetic measured-pose publisher may run alongside an owner.

## Standalone feature launches

```bash
# Transport and selected-arm mapping; no arm or gripper connection
ros2 launch quest_teleop quest_teleop.launch.py

# Arm owner only; defaults to mock, loads controller and safety configuration
ros2 launch teleop_controllers cartesian_impedance_controller.launch.py

# Same exclusive owner with guarded homing available; no startup home motion
ros2 launch teleop_controllers homing.launch.py

# Gripper only; defaults to mock and calibrates once at startup
ros2 launch teleop_gripper gripper.launch.py

# Independent Menu recorder
ros2 launch recording recording.launch.py

# Independent continuous recording
ros2 launch recording recording.launch.py recording_mode:=continuous

# Load embedded safety parameters (normally included by the arm launch)
ros2 launch safety_filter safety_filter.launch.py

# All configured mappers and optional recording, without arm/gripper owners
ros2 launch teleop_bringup robot_free.launch.py mock_measured_pose:=true
```

Feature launches accept their necessary parameter files. Defaults live in the
owning package. Arm and gripper launches accept `runtime_config`; safety accepts
`safety_config`; recording accepts `recording_config`; Quest accepts `arms_config`
and its bridge/mapper arguments. Future controller implementations belong under
`teleop_controllers`, while constraint verification and asynchronous solver work
belong under `safety_filter`.

## Verification

On 2026-10-12, an isolated Humble build passed all nine packages in the new
bringup dependency graph. All 97 individual robot-free cases passed across the
six feature packages, including standalone feature launches, custom safety
configuration, single/dual operation, feature switches and finalized bags.
The nine legacy Panda package files match their pre-migration SHA-256 hashes.
Physical control and Milestone 2 algorithm changes were not part of this check.
