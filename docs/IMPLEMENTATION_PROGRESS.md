# Revised implementation procedure

Current scope: complete controller, Hand, homing, recording and Panda CBF-QP
code using thesis_ws as the reference; stop before deployment commissioning.
Physical FCI/device operations are not part of this coding pass. Docker,
`panda_bringup`, and `panda_demos` remain unchanged.

## Stages and evidence

1. **Quest/bridge/mapper baseline — verified.** Quest UI/passthrough/USB
   reconnect and per-hand button schema accepted by the user. Per-arm topics,
   calibration, clutch anchors and twist processing have robot-free tests.
2. **Atomic consumer and controller safety — verified robot-free.**
   Added PC-monotonic source time, clock uncertainty, reference generation and
   valid-release metadata to the atomic command. Fixed-size SPSC input queue
   and latest-only telemetry remove mutexes and ROS publications from the
   torque callback. Core enforces source/receipt age, ordering, identity,
   release-before-engagement, measured-pose damping hold, outer envelope and
   actuator magnitude/slew intersection. Hardware adapter uses `tau_J_d` as
   its rate-limit anchor and retains libfranka's limiter. Mock defaults to a
   fixed fixture; it does not simulate dynamics or physical safety.
   Docker build and 11 C++ core/handoff + 2 actual controller-process tests pass,
   along with the 23 existing Quest/configuration/launch cases. No physical FCI
   connection or Android rebuild occurred.
3. **Franka Hand worker — verified robot-free.** Non-RT backend, explicit calibration,
   50 ms grip debounce, discrete toggles, no queued/repeated/retried motion,
   unknown-state handling and configurable width/tolerances/status. Nine core
   tests and two actual worker-process tests pass after one Docker build/test
   batch, including busy-press discard and issued-grasp completion on input loss.
4. **Independent homing — verified robot-free.** X/left and A/right one-second hold then
   release, separate nominal joint controller, controlled cancellation,
   exclusive ownership, measured-pose return and explicit re-engagement.
   No automatic homing/calibration at startup. Use the specified fixed Panda
   home joints; path clearance remains unverified. Eight pure core/trajectory
   tests and one actual two-controller process test pass after one Docker
   build/test batch. X and A are independent, startup-held/short holds are
   ignored, and trigger cancellation requires fresh release/press to resume.
5. **Panda CBF-QP — implemented and verified robot-free.** The unchanged
   impedance and separate homing nominal laws both pass through the Panda filter.
   One coordinator owns the coupled QP, with fixed-size same-cycle model/result
   handoffs, active-arm masking, state-age/skew/result/filter deadlines,
   diagnostics and a latched shared stop. Invalid models or timing faults stop
   ownership; repeated infeasibility stops after the configured failure count.
   Temporary fallback is actuator-bounded and cancels motion/homing, without a
   geometric guarantee. Nine filter + five coordinator + three actual runtime
   cases pass. Hardware same-RobotState model collection compiles; physical
   execution and 1 kHz worst-case timing remain commissioning work.
6. **Single/dual controller launch and Menu recording — verified robot-free.**
   Single-right default, single-left override and dual independent assignments
   launch actual mappers/owners. Left Menu starts/stops recording even with no
   left-arm mapper/controller. Each start records the publisher intersection
   of the customizable topic list, excluding inactive arm namespaces; normal
   shutdown finalizes bags. Three gate tests, four configuration cases and
   three actual integrated launch cases pass, including default Hand worker startup without calibration/motion, independent
   dual X/A gestures, unique successive episodes and sqlite3 topic/message inspection.
7. **Robot-free full acceptance — passed.** A combined Docker build/test batch
   and focused fixes pass all 81 individual cases across Quest/controller/Hand/
   homing/CBF/coordinator/launch/recording. The DDS bridge fixture waits for actual
   telemetry acknowledgement, and bag checks wait for recorder subscriptions.
   The user-verified APK/USB/display acceptance remains valid; no Android rebuild
   or physical FCI operation occurred. Docker and legacy Panda paths match their
   pre-change hashes. Explicit integrated commands: `docs/TELEOPERATION_RUNTIME.md`.
8. **Deployment commissioning — intentionally outside this task.** Physical
   clearance/calibration/tool/base/payload survey, gains, object settings,
   compatibility, RT and worst-case timing, hardware stop/recovery acceptance.

## Controller envelope and hold semantics

The coarse sphere is centered at Panda joint1 `[0,0,0.333]` in base coordinates.
Its radius is the sum of pinned Panda URDF joint-origin distances from joint1
to flange: `0.316 + 0.0825 + hypot(0.0825,0.384) + 0.088 + 0.107`, plus the
configured flange-to-tool distance and numerical margin. Default tool distance
`0.1034 m` is a reference Franka Hand offset and requires physical confirmation.
This outer envelope admits unreachable poses and does not check collisions,
joint feasibility, or singularity. It is not an arbitrary small workspace box.

Invalid/stale input removes target error and desired twist immediately. Output
torque transitions toward measured-pose damping/coriolis through magnitude and
rate bounds. No rejected target is pursued during the transition. Default
`900 Nm/s` and FER torque ceilings are software reference bounds, not approved
physical commissioning values. The hardware adapter leaves existing robot
collision configuration unchanged, requires explicit operator confirmation,
and finishes shutdown after a bounded damping-hold interval. Fault exceptions
terminate control through libfranka; they never reconnect/recover automatically.

## Commands for the current stage

On the **PC host**, open the existing ROS container:

```bash
cd /home/jiachun/franka_teleoperation
docker compose -f docker/compose.yaml run --rm panda-ros2
```

Inside the **container**, after the packages are built:

```bash
cd /ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run franka_teleop_controller cartesian_impedance_controller --ros-args \
  -r __ns:=/panda_1 -p backend:=mock
```

In a **second sourced ROS container terminal**, reuse the validated Quest USB
bridge and launch the mapper without another synthetic pose publisher:

```bash
ros2 launch quest_teleop robot_free.launch.py start_bridge:=false mock_measured_pose:=false
```

In a **third sourced ROS container terminal**:

```bash
ros2 topic echo /panda_1/controller_status
ros2 topic echo /panda_1/teleop_anchor_pose
```

These commands use only the mock controller. Release then press the assigned
trigger to enable; tracking/USB loss must disarm and require another release/
press. The mock pose stays fixed while reported nominal/commanded torques
change. No new APK installation is needed for this ROS-only stage.

## Franka Hand mock checks

In a **sourced ROS container terminal**, run the worker with synthetic object
settings (the width/tolerances below are mock-test values, not cable settings):

```bash
ros2 run franka_teleop_controller franka_hand_worker --ros-args \
  -r __ns:=/panda_1 -p backend:=mock -p controller_id:=right \
  -p grasp_width_m:=0.006 -p epsilon_inner_m:=0.001 -p epsilon_outer_m:=0.002
```

In another **sourced ROS container terminal**, observe and explicitly request
the mock calibration:

```bash
ros2 topic echo /panda_1/gripper_events
```

```bash
ros2 service call /panda_1/calibrate_gripper quest_teleop_interfaces/srv/CalibrateGripper \
  '{empty_hand_confirm: true, operator_confirm: true}'
```

The service reports acceptance; `gripper_events` reports actual completion.
The worker starts unknown and uncalibrated, with no motion. Calibration is a
separate explicit command and does not initialize the toggle to grasp. After
calibration, release grip then press above 0.75 for at least 50 ms: the first
press opens, the next release/press grasps, then the next opens. Holding grip
does not repeat; release alone does not open. Presses while busy are discarded.
Tracking/USB loss blocks new presses but leaves an issued operation running.
After reconnection, a new grip release is required. All commands above use the
mock backend; hardware width, tolerances and calibration remain commissioning
work.

Implementation adapts thesis_ws's one-operation-at-a-time non-RT Hand adapter
pattern to direct libfranka, discrete force-controlled grasp/open toggles,
and this repository's explicit calibration policy. It deliberately uses the
current policy for input loss rather than cancelling already-issued operations.
Command failures report unknown state and require a deliberate open gesture;
communication/state faults additionally invalidate calibration and require an
explicit calibration/reconnect request. There is no automatic retry or force
increase.

## Homing mock checks

Reuse the **mock arm controller** command above. Its raw input defaults to the
right controller, so hold A for at least one second and release it with the
trigger released. In a **sourced ROS container terminal** observe:

```bash
ros2 topic echo /panda_1/homing_status
ros2 topic echo /panda_1/controller_status
```

The fixed mock already reports home joints: expect `homing` then `succeeded`.
It tests gesture/mode ownership, not a physical path. Trigger press while
homing produces `stopping` then `cancelled`; holding the trigger cannot resume
teleoperation. Release and press it again after cancellation. Tracking loss,
source timeout, reference/session change and queue overflow also cancel through
the stopping trajectory. No home gesture is queued while busy.

For **left-controller homing**, stop that mock process and run in its place:

```bash
ros2 run franka_teleop_controller cartesian_impedance_controller --ros-args \
  -r __ns:=/panda_1 -p backend:=mock -p controller_id:=left
```

Hold X for at least one second and release with the left trigger released.
Change the mapper's saved arm assignment to left for left teleoperation; the
controller's homing raw-input subscription follows its own controller ID.

Homing uses the separate thesis_ws-derived joint PD nominal law and a
preallocated synchronized trajectory generator; it preserves the source
license. Cancellation ramps desired joint velocity to zero with a cosine
profile bounded by configured stopping acceleration, then waits for measured
settling before returning to Cartesian damping hold. Torque magnitude/slew
checks remain downstream for both nominal laws. A stop that cannot settle
within its timeout terminates hardware control via libfranka, without an
automatic restart. Gains, speed and stop timing are reference values requiring
physical commissioning. Both nominal laws now pass through the same CBF filter; no physical homing
operation has been performed.

## Pinned CBF dependency and build

Inside the **ROS container at `/ws`**, use this helper for a fresh checkout:

```bash
bash scripts/fetch_teleop_dependencies.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select libfranka franka_msgs \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
source install/setup.bash
colcon build --symlink-install --packages-select quest_teleop_interfaces quest_teleop franka_teleop_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
```

The helper imports the existing legacy repositories and OSQP commit
`0dd00a578cf1c2691c5c379965d504c75bf6cfad` (v0.6.3), plus pinned submodules.
It excludes OSQP from independent colcon discovery because the controller
compiles the solver with fixed double precision and disabled printing,
signal handlers and MKL loading. Docker files and ROS/Android versions are
unchanged. Existing fetched/built dependencies need not be rebuilt for each
controller edit. Detailed filter design and deployment limitations:
`docs/CBF_PANDA.md`.

## Integrated runtime and acceptance audit

The integrated launch supersedes the earlier separate-process mock commands
above when checking the complete stack. See `docs/TELEOPERATION_RUNTIME.md` for
host/container setup, single-right/left/dual commands, X/A home gestures, left
Menu recording, Hand calibration, topic customization and `ros2 bag info`.

| Requirement | Implementation | Authoritative robot-free evidence |
| --- | --- | --- |
| Atomic consumer, safety and unchanged nominal law | `control_core.cpp`, bounded handoff, actual adapter | 12 core/handoff + 2 mapper/controller process cases |
| Separate Hand worker/calibration/fault policy | `hand_core.cpp`, `franka_hand_worker.cpp` | 9 core + 2 actual worker cases |
| Independent one-second X/A release homing | `homing_core.cpp`, hand-specific raw subscriptions in each owner | 8 trajectory/core + 1 actual two-owner gesture/cancellation case + independent X/A through the coupled dual launch |
| Panda CBF filters impedance and homing torque; coupled dual safety | `cbf_safety_filter.cpp`, `safety_coordinator.cpp`, same-cycle model adapter | 9 filter + 5 coordinator + 3 actual runtime cases |
| Single-right default, single-left option, dual launch | `cartesian_impedance_controller.launch.py`, `select_arms` | Configuration selection and all three actual launch modes |
| Left Menu toggles independent of active left arm | `menu_recorder.py`, independent launch subscription | 3 gate cases + actual single-right launch with no left owner/mapper |
| Configurable recording contains only active selected topics | Publisher intersection, inactive namespace exclusion | Finalized sqlite3 bags exclude published inactive/unpublished topics; successive episodes and shutdown finalization |
| Preserve Docker, legacy packages and safety remarks | Existing containers/dependencies, isolated new packages | Builds in existing container; 17 protected file hashes unchanged; CBF limitations retained |
| Stop before deployment commissioning | No physical operation in this pass | Mock-only process arguments; remaining deployment decisions in `AGENTS.md` |

Results are the 15 case-level XML files under
`build/{quest_teleop,franka_teleop_controller}/test_results/`: 27 Quest and 54
controller cases, zero failures/errors. CTest wrapper counts are not additional
individual cases. Relevant builds/tests were batched; only failures/substantive
changes prompted focused reruns. These results establish software acceptance,
not physical dynamics, clearance or deployment timing.
