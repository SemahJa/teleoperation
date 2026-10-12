# Teleoperation implementation policy

## Startup Hand calibration override (2026-10-12)

The operator explicitly requests automatic calibration when `start_hand:=true`.
This supersedes historical instructions below requiring manual-only startup
calibration. The operator further requests no separate calibration launch
configuration: enabling Hand automatically requests startup calibration. Operator launch
with Hand enabled requests one empty-hand calibration; document physical finger
motion and empty-hand preparation. Retain manual recalibration, hardware operator
confirmation, no automatic retries/grasp, and robot-free agent validation.

## Current prerequisite: package reorganization (2026-10-12)

Before Milestone 2 implementation, the operator explicitly requests separate
`quest_teleop`, `recording`, `teleop_controllers`, `teleop_gripper`, `safety_filter`
and `teleop_bringup` packages with feature launches and configurable combined
bringup. Preserve `panda_bringup` and `panda_demos` unchanged. Current boundaries
and migration commands: [PACKAGE_STRUCTURE.md](PACKAGE_STRUCTURE.md).

## Latest scope: Milestone 2 (2026-10-12)

The operator accepted Milestone 1 after successful single-arm hardware
teleoperation with CBF disabled. Next implement asynchronous CBF-QP for
single-arm experiments first, plus controller performance measurement,
smooth reference generation and distinct controller structures for comparison.
See [MILESTONE_2.md](MILESTONE_2.md) for the design and acceptance criteria.
Preserve the working nominal controller as the selectable baseline; the new
request authorizes additional controller structures despite older instructions
below to preserve only that law. Keep dual-arm execution unchanged in the first
asynchronous stage. Agent development and automated tests remain robot-free;
physical FCI/device trials remain operator actions.

The sections below retain Milestone 1 requirements and historical evidence.

Status: milestone 1 implementation policy and implementation order approved. Remaining technical details and deployment commissioning values are listed below; approval of the plan alone is not authorization to change implementation code. The repository owner has since explicitly requested continued implementation within this approved scope.

## Development working rules (user instruction, 2026-10-10)

- Continue authorized implementation autonomously within the agreed scope and sequence. Use practical judgment; do not add unnecessary confirmation steps or hypothetical-risk warnings.
- Preserve the existing Docker environments, safety requirements, and unrelated working-tree changes. Practical autonomy does not change the physical hardware/FCI boundary. The revised scope below authorizes homing and CBF-QP code before deployment commissioning.
- Batch related implementation edits. Run builds and tests after a major code change or a completed coherent implementation stage, rather than after each small edit. Documentation-only and minor low-impact edits do not require a build/test run.
- After a major change, run the relevant build and focused tests once. Repeat only to check fixes for an actual failure or a substantive follow-up change; avoid redundant full-suite runs.
- Provide explicit commands and identify the PC terminal, container, and headset actions when asking the user to check runtime behavior. Reuse the existing APK and containers when code has not changed.

## Revised implementation scope (2026-10-10)

The latest user instruction supersedes older homing/CBF deferrals in historical notes. Implement the atomic controller consumer, controller safety, Franka Hand worker, independent arm homing, Menu recording, single/dual controller launch, and Panda-adapted CBF-QP now. Stop before deployment commissioning or physical FCI operation. Preserve Docker environments, the filtered-error nominal impedance law, exclusive arm ownership, explicit operator confirmation, and all physical-clearance/commissioning limitations.

- Left X held at least one second then released requests homing of its assigned arm; right A uses the same gesture independently. No dual-button chord. The assigned trigger must remain released, with fresh valid tracking. Cancellation, measured-pose return, and fresh trigger release/re-press remain required.
- Left Menu press toggles configurable rosbag2 recording, including single-right-arm operation. Subscribe to left input independently of active-arm mapping. Record only published topics selected from the configurable list.
- Controller launch exposes single/dual mode. Single defaults to right control and permits left selection. Dual has independent left/right assignment and exclusive FCI ownership per arm; collision filtering must account for both arms.
- Adapt thesis_ws homing/recording/CBF architecture to legacy Panda and ROS Humble. Do not import FR3 limits or replace the agreed nominal impedance law. Retain CBF geometry, solver timing/failure, singularity, physical-clearance and commissioning limitations.
- Track coherent stages and verification in `docs/IMPLEMENTATION_PROGRESS.md`; run focused builds/tests after meaningful batches.

## Current handoff snapshot (2026-10-10)

### Current revised implementation complete; deployment commissioning remains (2026-10-10)

- Completed CBF runtime integration for single/dual owners. Impedance and homing nominal torque both pass through one Panda-adapted coupled solver. Same-RobotState hardware models, exact-cycle fixed-size handoff, active-arm mask, age/skew/result/filter deadlines, transient fallback cancellation and latched shared stop are implemented. Normal finishing is paired across both owners. No physical control operation occurred.
- Added single/dual integrated launch: single defaults right and permits left; dual uses distinct configured owners/assignments. Optional Hand workers remain separate and require explicit calibration. Runtime gain/Hand/CBF settings and per-arm surveyed base transforms are configurable; hardware retains explicit backend/IP/operator confirmation, enforced realtime and memory-lock prerequisites.
- Added independent left Menu recording even in single-right mode. Fresh release/press and 350 ms debounce toggle successive unique episodes; only published selected topics are included, with inactive arm namespaces excluded. Started bags survive input loss and finalize on normal launch shutdown. Topic lists/output roots remain configurable in the existing mounted workspace.
- Combined Docker build and robot-free acceptance plus focused corrections pass 81 individual cases (27 Quest, 54 controller/Hand/homing/CBF/coordinator/runtime/launch), zero failures/errors. Fixed snapshot clock/acquire ordering and test fixture discovery/source timestamp issues without relaxing hardware or Quest gates. Verified finalized bags, all three launch modes, shipped runtime defaults/Hand startup, and independent X/A gestures through the coupled dual launch. Docker/legacy protected hashes remain unchanged; validated APK was reused.
- Current evidence/audit: `docs/IMPLEMENTATION_PROGRESS.md`. Explicit host/container/headset check commands: `docs/TELEOPERATION_RUNTIME.md`. CBF timing/fallback/geometry/singularity limitations: `docs/CBF_PANDA.md`.
- Remaining work is deployment commissioning: object settings/calibration, physical clearance/home path/base/table/tool/payload, gains, firmware compatibility, PREEMPT_RT/scheduling/memory locking and worst-case 1 kHz timing, controlled hardware stop/recovery acceptance. Do not infer these from coding-host or mock results. Older snapshots below are historical and do not reopen completed implementation stages.

### Panda CBF library verified; runtime integration still pending (2026-10-10)

- Adapted thesis_ws fixed-pattern OSQP CBF filter to FER/Panda limits and pinned Panda collision geometry/SRDF; preserved source/dependency licenses. Added single-arm masking, actual tau_J_d torque-rate anchors, invalid-model/active-constraint faults, fixed-size runtime statuses and total filtering timing.
- Added OSQP 0.6.3 by exact commit to third_party.repos. `scripts/fetch_teleop_dependencies.sh` imports pinned dependencies/submodules and excludes standalone OSQP colcon discovery; the controller builds the solver with fixed double precision, no printing/signal handlers/MKL. Docker definitions and legacy Panda packages remain untouched.
- Nine CBF tests pass, covering joint/actuator rows, coupled dual-arm collision torque, invalid models and infeasible fallback. Instrumented C++/C solver allocation paths show zero runtime allocations after configure. A failed transform test exposed Eigen rotation extraction hiding scale; validation now uses actual linear matrices. Added consumer nonfinite-nominal status regression (12 core cases).
- The filter is not yet called by arm owners. Complete model snapshots, coupled coordinator, state-age/skew/result deadlines, repeated-failure stop handling and runtime mock acceptance next. Then finish controller mode launch and independent left Menu recording. No formal singularity barrier or fallback geometry guarantee is claimed. Design/limitations: `docs/CBF_PANDA.md`.

### Independent arm homing verified (2026-10-10)

- Added separate `HomingCore` and adapted/preallocated thesis_ws synchronized joint trajectory generator with preserved license headers. A single permanent direct-libfranka owner selects Cartesian or homing nominal torque; both use the same actuator bounds. There is no competing homing FCI process.
- Raw hand-specific input drives one-second X/left or A/right hold-then-release. Startup-held/short/busy gestures cannot initiate or queue home. Fresh focus/tracking/clock, active button/trigger and trigger release are required. Session/reference change, tracking loss, timeout, trigger press, queue overflow and shutdown cancel through bounded desired-velocity deceleration and measured settling.
- Uses fixed specified Panda home joints. Success/cancellation returns to measured Cartesian hold with a fresh trigger release/press required. Failure to settle the controlled stop terminates hardware control rather than restarting. Homing gain/speed/path clearance and stop timing remain commissioning concerns.
- One Docker build/test batch passes 8 trajectory/core + 1 actual two-process per-arm acceptance cases, plus affected consumer/core regression checks. Prior Hand and Quest results remain recorded passing (56 actual cases across stages). No physical operation or Docker/legacy package changes. Explicit mock commands: `docs/IMPLEMENTATION_PROGRESS.md`.
- Next: Panda CBF-QP filtering and dual-arm coordinator, single/dual launch and independent left Menu recording, then combined robot-free acceptance.

### Franka Hand worker verified (2026-10-10)

- Added separate `franka_hand_worker` executable with blocking direct-libfranka calls in an independent non-real-time worker thread; no arm torque callback access. Defaults to mock and has no startup motion or connection.
- `calibrate_gripper` service requires explicit empty-hand and operator confirmation. Completion establishes calibrated maximum width but leaves toggle state unknown; the next deliberate press opens. Object width/inner/outer tolerances must be configured before grasping.
- Fresh source-based 50 ms grip debounce, 0.75/0.25 hysteresis, release after reconnect, busy-press discard/no queue, discrete grasp/open toggles, and no automatic retry/open/force increase are implemented. Tracking/USB loss leaves an already-issued operation running. Command failure leaves unknown state for deliberate open; communication/state failure requires explicit recalibration/reconnect.
- One Docker build and focused Hand test run pass: 9 core + 2 actual process cases. Prior controller and Quest results remain passing (47 actual cases recorded across coherent stages). No physical Hand/FCI connection, Android build or Docker change. Mock calibration and runtime commands are in `docs/IMPLEMENTATION_PROGRESS.md`.
- Next: independent homing, Panda CBF-QP, single/dual controller launch and left Menu recording, then combined robot-free acceptance. Physical commissioning remains outside the task.

### Atomic controller and safety core verified (2026-10-10)

- Controller consumes the extended atomic command, including PC-monotonic acquisition time, clock uncertainty, reference generation and explicit valid-release metadata. It retains the original filtered-error impedance law.
- Bounded SPSC input and latest-only telemetry replace mutex/ROS work in the torque callback. Source/receipt age, ordering, session/reference identity, release/re-engagement, coarse model-derived envelope, torque magnitude and torque-rate checks are implemented. Invalid input removes stale target/feedforward and slews toward measured-pose damping hold.
- Backend defaults to a fixed robot-free fixture. Hardware requires explicit backend, IP and operator confirmation; no hardware connection was made. The hardware callback uses tau_J_d as its slew anchor and keeps libfranka limiting enabled.
- Existing declared legacy dependencies were fetched and built in the existing Docker container. One coherent build/test and a focused correction run passed: 36 actual cases (23 Quest/config/launch, 11 C++ safety/handoff, 2 actual controller-process acceptance). The bridge fixture now waits for DDS discovery and uses current source timestamps except deliberate stale injection.
- Docker/scripts and legacy Panda packages match their pre-change hashes. Android was not rebuilt. Procedure and explicit mock commands: `docs/IMPLEMENTATION_PROGRESS.md`.
- Next stage: separate non-real-time Franka Hand worker and calibration/toggle acceptance; then homing, CBF-QP, controller mode launch and left Menu recording. Physical commissioning remains outside the task.

### Verified Quest input and per-arm configuration/recording (2026-10-10)

- User confirms the latest button-schema changes are verified and explicitly requests the next implementation. Prior Quest display/passthrough and USB reconnect acceptance remains valid; do not repeat Android builds or headset checks without a relevant change.
- Completed the next configuration/robot-free acceptance batch: `quest_teleop/robot_free.launch.py` loads validated `arms.yaml`, starts independent namespaced mappers, supports per-arm left/right assignment, topics, frame IDs and saved calibration quaternions, and optionally starts the USB bridge. Milestone default remains one right-controlled Panda.
- Optional fixed synthetic measured-pose publisher is enabled only with `mock_measured_pose:=true`. It has no hardware access and is a fixture rather than a dynamics/workspace model. The launch starts no arm/gripper controller.
- Added disabled-by-default configurable rosbag recording with optional derived default profile, custom YAML or a complete JSON topic-list override, and unique output directories under the existing `/ws` mount. Recording can be replaced or disabled; no Docker environment changes. Details and commands: `docs/ROBOT_FREE_MAPPING.md`.
- Verification: one Docker build and one focused test run pass (23 tests: 16 mapper, 3 bridge integration, 3 configuration, 1 actual multi-arm launch/recording acceptance). The launch test verifies independent left/right clutch targets, tracking-fault/re-engagement isolation, controller-ID rejection and a finalized sqlite3 bag containing only the selected override topic. No Android build, hardware operation, or controller code change was required.
- Next implementation batch: stage 4 atomic controller command consumer and real-time-safe command/telemetry handoff, controller freshness/watchdog/measured-pose hold, configurable model-derived outer envelope and torque-rate checks, then gripper worker/calibration/toggle behavior. Homing and CBF-QP implementation are now authorized by the revised scope; physical FCI operation remains outside this coding task.

### Quest runtime acceptance and button-schema follow-up (2026-10-10)

- User reports all headset display checks valid: passthrough, readable HUD, aim rays and recenter placement. USB unplug/replug and local release/re-engagement work as expected. These runtime checks supersede the historical pending display/passthrough notes below.
- Supplied telemetry has valid focus/tracking/native linear and angular velocity on both hands, valid source time, clock uncertainty about 0.79–1.32 ms, and reference-space generation 17. Two consecutive right-hand samples retain A pressed; the left sample has Y pressed. These samples do not reproduce button flicker or demonstrate every button's sustained behavior.
- Reported duplicated button fields came from the common ROS schema including all controllers' buttons, despite hand-specific TCP packets. `QuestControllerState` now uses `QuestButtonState[<=3] buttons`: right a/b, left x/y/menu. Each entry retains pressed versus OpenXR active independently; unexposed System/Meta placeholders are removed. Disconnect notifications have empty buttons and invalid connection metadata.
- Bridge rejects cross-hand/unknown input fields and malformed signed/whitespace text in unsigned timestamps/sequences. Integration tests use isolated ROS namespaces to avoid interference with a user's live bridge. APK and Docker environments are unchanged; this ROS schema change requires rebuilding interfaces/bridge and restarting ROS subscribers, not rebuilding/reinstalling the validated APK.
- Verification: one Docker build of `quest_teleop_interfaces` and `quest_teleop` passes; all 19 ROS tests pass (16 mapper + 3 bridge integration tests). Added coverage checks exact per-hand button lists, pressed/activity distinction, disconnect clearing, malformed decimal fields, and opposite-hand/unexposed buttons. No Android rebuild or device operation was needed for this ROS-only change.

### VR app port follow-up (2026-10-10)

- User explicitly requested porting the `thesis_ws` VR app while preserving Docker, safety structures, and implementation sequencing. The OpenXR lifecycle, world-fixed readable HUD, optional passthrough, per-hand primary/secondary bindings, HMD/aim/thumbstick diagnostics, and controller rays are now ported into `android/quest_teleop`. Review/adaptation details: `docs/VR_APP_PORT.md`.
- The existing `UsbTransport` class is extracted verbatim into `usb_transport.h`; existing TCP/ADB framing, clock/freshness gates, ROS schemas, mapper and all PC-side packages are unchanged. No UDP/Wi-Fi transport or native-velocity fallback was imported. Docker definitions, scripts, controller code, and this document's approved implementation order are unchanged.
- HUD displays separate left/right pose/native twist/input values and activity flags, USB/clock diagnostics, and a right-controller local deadman latch. Robot arming is unknown; this is telemetry-only. No robot command path was added. Device button flicker is still unverified; raw values are not filtered.
- Gradle `ndkVersion` now matches the existing container's NDK 26.3.11579264; the historical NDK drift below is resolved. APK identity/signing volumes and loader/container versions are preserved.
- Docker APK build passes. Existing 16 mapper tests + one bridge-to-mapper integration test pass. New actual C++ adapter/USB sender test passes against a mock TCP receiver, including independent velocity/activity flags, clock uncertainty rejection/recovery, and diagnostic release/re-engagement faults. No APK installation, headset inspection, deployment inspection, or FCI connection occurred.
- Next runtime checks remain held-button telemetry and Quest passthrough/compositor/HUD testing. The readable UI implementation is complete; on-headset verification is pending. Continue the existing later controller/gripper steps only within their approved scope.

### Baseline before the VR app port

- Robot-free Quest TCP bridge → raw ROS state → mapper → mock measured-pose fault-injection test is implemented and passes (1 integration + 16 mapper unit tests). Do not use FCI.
- Verified manually: USB/ADB telemetry, left/right tracked samples, valid clock conversion. After TCP_NODELAY/single-write framing: RTT ~1.5–3 ms, uncertainty <4 ms (10 ms gate). Do not relax the gate.
- Docker ROS and Android builds pass. ROS verification command: `colcon test --packages-select quest_teleop` then `colcon test-result --all --verbose` inside ROS container; 16 unit tests + 1 end-to-end integration test pass.
- Mapper is robot-independent and publishes atomic command messages, but no arm controller consumes them. Default measured-pose anchor topic is a mock/integration input. Never connect to physical FCI.
- Unresolved: same-controller button flicker (provided sample had no pressed button; capture a held button), readable per-controller headset status UI, passthrough runtime retest after `XR_ERROR_LAYER_INVALID` (-23) fix, Android Gradle/container NDK drift. Full bridge-to-mapper integration test now passes.
- No arm homing before first VR teleoperation test. No changes to `src/panda_bringup` or `src/panda_demos`. No deployment PC/hardware inspection or robot connection.
- Changes are uncommitted; preserve the current working tree. Android debug signing uses the `android-signing` Docker volume; deleting it changes APK signatures and requires uninstalling the installed app (loses app data).

## Confirmed scope and environment

- Milestone 1: use a Quest 3 VR controller to teleoperate one legacy Panda / Franka Emika arm with Cartesian impedance control.
- Robot system version: 4.2.2. FCI will be enabled through Franka Desk.
- Deployment PC: Ubuntu 22, with PREEMPT_RT available. Verify the RT kernel is active during deployment commissioning.
- The current PC is for coding only. Do not inspect its environment or hardware to infer deployment capabilities.
- Keep milestone 1 PC-side services in containers, using the existing ROS 2 Humble baseline.
- Provide the Android development environment in a container as well.
- Build the Quest application using native C++ OpenXR, Android NDK, CMake, and Gradle; no Unity dependency.
- Milestone 1 headset UI is minimal: connection, tracking, and armed/disarmed status. Immersive robot visualization is out of scope.
- Quest-to-PC telemetry must use physical USB, not Wi-Fi.
- Use ADB reverse with a framed TCP stream for Quest-to-PC telemetry. Quest developer mode and USB-debugging authorization are required; wireless ADB is not used.
- Run ADB in a container with explicit USB access. The PC bridge listens on the reversed port; the Quest APK connects through its loopback address.

## Confirmed controller interaction

- Use the right controller for relative position and orientation teleoperation, enabled while its trigger is held.
- Configure trigger hysteresis: engage at value >= 0.75, release at value <= 0.25; retain the current trigger state between thresholds.
- After startup, faults, or reconnect, observe trigger release before accepting a fresh engagement. Thresholds are configurable.
- Release the trigger to pause motion and reposition the hand.
- Re-engagement anchors to the robot's current measured pose without a target jump.
- Tracking loss or disconnect disables teleoperation; recovery requires explicit re-engagement.
- Use the grip input for Franka Hand gripper control. Implement gripper actuation in milestone 1, before the first VR teleoperation hardware test.
- Toggle grasp/open on each discrete grip press; holding the grip must not repeat commands, and release does not open the hand. Sustained operator pressure is not required to maintain a grasp.
- Reject new grip commands while disconnected or controller tracking is invalid.
- Never automatically open the hand on tracking loss or disconnect.
- Ignore additional grip presses while an operation runs; do not queue presses.
- Require grip release before accepting new presses after reconnect.
- After restart or an uncertain operation result, display gripper state as unknown; the next deliberate grip press requests open to establish a known state. Do not open automatically on startup.
- Intended grasped objects include Ethernet cables. Approved configurable initial gripper speed: 0.02 m/s; grasp force: 5.0 N. Verify support on the installed Franka Hand and suitability with the actual cable/fingertips before use.
- Recommend soft, grooved fingertips and grasping the cable jacket rather than connector latches; these physical details are not yet confirmed deployment equipment.
- Configurable grip hysteresis: press at value >= 0.75, release at value <= 0.25. Require 50 ms continuously above the press threshold before issuing one toggle; require release before another toggle.
- Measure debounce duration from fresh source samples; invalid/stale input or a session change resets pending press detection.
- If Quest tracking or transport is lost while a gripper operation runs, allow the already-issued grasp/open operation to complete while rejecting new commands; do not automatically open, reverse, or retry it.
- If communication with the gripper itself fails, mark its state unknown and report the fault; do not automatically retry motion.
- Do not automatically home/calibrate the Franka Hand at startup. Provide an explicit calibration command requiring an empty hand and operator confirmation; this is separate from robot-arm homing.
- Disable grasp/open commands until gripper calibration is confirmed.
- Use the reported calibrated maximum width as the configurable default open width.
- Verify gripper calibration during the first hardware teleoperation test before testing cable grasps.
- Configure grasp target width per object from the measured cable diameter at the contact point; do not hardcode one width for all cables.
- Configure inner/outer grasp tolerances during commissioning for the actual cable and fingertip compliance; the operator must configure width/tolerances before cable-grasp testing.
- Report failed grasps; do not automatically increase force or retry.
- Object-specific width/tolerance values remain deployment decisions. The explicit calibration service/completion status is implemented and mock-verified.
- Holding the assigned right controller A or left controller X button for at least one second, then releasing it, requests only that arm's homing.
- Implement homing as a separate controller from Cartesian impedance control.
- Home is a fixed seven-joint configuration, not a startup-captured configuration or Cartesian pose.
- Home joint angles in Panda joint1–joint7 order, radians: `[0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]`.
- The selected home configuration and the path to it have not been verified collision-free for the deployment setup.
- Accept the A-button hold/release homing request only while connected, tracking is valid, and the trigger is released.
- Homing takes exclusive control ownership; Cartesian impedance teleoperation and homing must never command FCI simultaneously.
- Trigger press, tracking loss, or disconnect cancels homing through controlled deceleration, not an abrupt controller switch.
- Successful homing returns to impedance hold at the measured pose; teleoperation requires a fresh trigger release/press.
- Faults never automatically resume homing or teleoperation.
- Trajectory generation, ownership and cancellation are implemented and mock-verified. Physical home-path clearance and response verification remain deployment work.

## Container boundary

- Android APK compilation and packaging can run in a development container; the resulting APK runs on Quest Android, not inside a Linux container on the headset.
- PC containers share the deployment host kernel. Containers do not supply PREEMPT_RT or configure the host's physical USB permissions by themselves.
- USB device access, realtime scheduling permissions, memory locking, and host kernel configuration must be documented as deployment prerequisites.
- Wrap all build/runtime environments and environment-dependent software features in Docker for reproducibility across computers: ROS 2 services, controller dependencies, Android SDK/NDK/JDK/Gradle, ADB, and test/logging tools.
- Pin base images and dependency/toolchain versions; document hardware-specific settings separately. Containers cannot encapsulate the host kernel, physical USB authorization/permissions, robot firmware/Desk state, or the Quest APK runtime.
- Existing ROS Humble and pinned Android containers/toolchains are implemented. Preserve their topology and versions; host-specific deployment settings remain separate.

## Confirmed controller architecture and sequencing

- Do not modify `src/panda_bringup` or `src/panda_demos` to implement the new controller architecture.
- Copy the existing Cartesian impedance controller into `src/teleop_controllers` for further implementation.
- Keep the legacy Panda direct-libfranka approach for milestone 1, subject to compatibility verification.
- Preserve the copied controller's filtered-error impedance law: `r = measured_twist - desired_twist + Lambda * pose_error`; `tau_nominal = -J^T * Kr * r + coriolis`.
- Expose translational/rotational Lambda and Kr as configurable per-arm parameters. Select physical-test gains during commissioning rather than treating existing launch gains as approved test values.
- Existing YAML implies translational stiffness 1500 N/m and rotational stiffness 180 Nm/rad; these are reference values only, not approved commissioning defaults.
- Keep ROS and USB callbacks outside the real-time control loop; use a bounded real-time-safe command handoff.
- Add controller-side command freshness checks.
- Implement bounded-motion checks in the controller. A later CBF-QP stage will filter nominal controller torque; do not assume that future filter provides safety during the first hardware test.
- Implement the separate homing controller under the revised scope above. No physical homing test is part of this coding task; configuration and path clearance remain unverified until deployment commissioning.
- Avoid unnecessary mode-management complexity before that first test.
- Implement Franka Hand actuation outside the arm's real-time torque callback. Arm homing is also in the current revised implementation scope.
- Defer FR3 `franka_ros2` implementation; preserve robot-independent VR interfaces.
- Design controller launch configuration for multiple legacy Panda arms, with per-arm IP, namespace, frame IDs, and left/right Quest controller assignment. Milestone 1 hardware acceptance remains one arm driven by the right controller.
- Each arm has one exclusive FCI owner. Do not start multiple control processes against the same arm.
- Require an explicit operator-confirm launch argument before connecting the torque controller to FCI.
- At startup initialize impedance hold from the measured robot pose and start disarmed; do not perform arm homing or automatic gripper motion.
- Enable teleoperation only after valid tracking, fresh samples, and trigger release/re-press.

## Confirmed package boundaries

- `src/quest_teleop_interfaces`: custom raw VR state and Cartesian command messages.
- `src/quest_teleop`: USB telemetry receiver, mapping, filtering, and multi-arm launch.
- `src/teleop_controllers`: arm impedance control, homing and future controller structures.
- `src/teleop_gripper`: Hand decision core and separate non-real-time worker.
- `src/safety_filter`: CBF constraints/QP, coordinator and model/handoff utilities.
- `src/recording`: recording plans and Menu/continuous rosbag recording.
- `src/teleop_bringup`: composition with independent feature switches.
- `android/quest_teleop`: native C++ OpenXR APK project.
- Keep existing `src/panda_bringup` and `src/panda_demos` unchanged.

## Confirmed initial bounds and invalid-command policy

- Apply Cartesian workspace checks with minimal restrictions relative to the Panda's reachable Cartesian workspace; do not impose an arbitrary small operating box as the permanent policy.
- Use the Franka Emika Robot (FER) limits, not FR3 limits, from https://frankarobotics.github.io/docs/robot_specifications.html . The referenced page was retrieved and reviewed during planning; applicability to the pinned legacy stack still requires verification.
- FER joint lower limits, radians: `[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]`.
- FER joint upper limits, radians: `[2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]`.
- FER Cartesian published maxima: translation velocity 1.7 m/s, acceleration 13 m/s^2, jerk 6500 m/s^3; rotation velocity 2.5 rad/s, acceleration 25 rad/s^2, jerk 12500 rad/s^3. These are specification ceilings, not agreed commissioning speeds, and are not automatically enforced on impedance pose targets by torque-control mode.
- FER published torque maxima: `[87, 87, 87, 87, 12, 12, 12]` Nm; torque-rate maximum 1000 Nm/s per joint. The referenced specification makes torque-rate compliance a necessary condition for torque control. Verify how the pinned libfranka configuration enforces it.
- Add a configurable command-age watchdog with an approved initial cutoff of 100 ms, subject to measured USB latency/jitter and hardware commissioning.
- Check both source acquisition age after monotonic clock conversion and elapsed PC steady-clock time since the last valid sample.
- Carry Quest monotonic acquisition time separately from OpenXR pose-location time/prediction metadata.
- Synchronize Quest and PC monotonic clocks using periodic USB round-trip exchanges; retain clock-offset uncertainty and disarm when uncertainty prevents reliable freshness checks.
- Reject duplicate/out-of-order samples and reset timing/session state on reconnect; newly received delayed packets must not appear fresh solely because of their receive time.
- Use source acquisition time for the 100 ms watchdog, with OpenXR location time retained for pose/twist interpretation. The implemented USB protocol defines synchronization and the 10 ms uncertainty gate; clock assumptions still need deployment measurements.
- Watchdog timeout disables teleoperation and transitions toward measured-pose hold with zero desired velocity. Recovery requires fresh valid input and a trigger release/re-press; packets alone must not automatically resume motion.
- The 100 ms cutoff is a communications freshness threshold, independent of the 1 kHz torque-control period; it is not a verified safe motion allowance.
- On invalid or stale input, transition toward impedance hold at the current measured end-effector pose with zero desired velocity; do not continue pursuing the rejected or stale target. The control core implements a measured-pose damping hold with actuator-bounded torque transition; physical response remains unverified.
- Do not implement IK-based target reachability analysis for the initial milestone.
- Defer collision avoidance and singularity avoidance to the later CBF-QP implementation; these protections are absent from the initial hardware-test baseline.
- Initial Cartesian checks use a configurable, tool-aware outer reach envelope derived from Panda model dimensions, checked in the robot base frame. Do not add IK, inner exclusion zones, or an arbitrary small workspace box.
- This coarse envelope rejects clearly excessive targets but admits some unreachable poses; it is not proof of full-pose reachability, joint-limit compliance, collision freedom, or singularity avoidance.
- The implemented envelope is documented in `docs/IMPLEMENTATION_PROGRESS.md`; deployment tool distance and margin require confirmation.

## Confirmed ROS and OpenXR data interfaces

- Publish raw VR input as a custom `QuestControllerState` message: controller pose, tracking validity, trigger/grip/buttons, controller identity, session/sequence identifiers, and timing metadata.
- Record and transmit native OpenXR controller linear and angular velocity, obtained through `XrSpaceVelocity` chained to `XrSpaceLocation` when locating the controller action space.
- Carry independent OpenXR linear-velocity and angular-velocity validity flags. Invalid velocity must not silently become valid zero or a finite-difference substitute.
- Express native velocities in the same explicit reference space as the controller pose, in m/s and rad/s, with the associated location time and source clock semantics.
- Preserve raw native twist in received/published data for logging.
- Use containerized `rosbag2` for persistent recording, outside the real-time control loop, with output written to a mounted host directory.
- Recording topics must be fully customizable through configuration/launch arguments, not hardcoded. Provide an optional default profile for raw VR state, mapped commands, robot state, and watchdog/connection/gripper events; users may replace or disable it.
- Use custom `CartesianTeleopCommand` as the new impedance controller's ROS 2 control input, carrying target pose, filtered desired twist, motion-enable state, robot-base frame, source timestamp, session ID, and sequence number atomically.
- Use standard nested message types such as `std_msgs/Header`, `geometry_msgs/Pose`, and `geometry_msgs/Twist`; the current schema includes PC-monotonic source time and clock uncertainty for controller freshness checks.
- Retain `geometry_msgs/msg/PoseStamped` for target-pose monitoring in each arm's base frame, not as the new controller's independent control input. Existing `panda_bringup` and `panda_demos` interfaces remain unchanged.
- Isolate command and monitoring topics per arm. Validate command metadata outside the real-time loop and transfer the matched pose/twist/permission sample through a real-time-safe handoff. The controller computes torques for libfranka; libfranka does not consume ROS messages directly.
- Use native OpenXR twist as the impedance controller's desired-velocity feedforward after reference-frame conversion and teleoperation motion scaling.
- Use one shared `feedforward_scale` for both linear and angular velocity, constrained to [0, 1], defaulting to 1.0. This supersedes separate translation/rotation feedforward scales.
- Compute desired twist continuously as `low_pass((1 - feedforward_scale) * twist_pose_difference + feedforward_scale * twist_OpenXR)`; do not implement differentiation as a separate mode selected by the scale.
- Scale 0 retains pose-derived desired velocity; scale 1 uses native twist; intermediate values blend both estimates without double-counting motion.
- Both contributions must use matched location timestamps, the same robot-base reference frame, and consistent teleoperation motion scaling before blending. Use a configurable first-order low-pass filter with an approved initial cutoff of 10 Hz, applied equally to linear and angular velocity; tune during USB and hardware tests.
- Pose-difference linear velocity uses translation difference. Derive angular velocity with normalized quaternion relative-rotation expressions, shortest-arc sign handling, and a stable small-angle limit, expressed in the robot base frame; do not subtract quaternion components or differentiate Euler angles.
- Quaternion-based angular velocity avoids Euler-angle representation singularities; it does not avoid robot kinematic singularities, which remain deferred to CBF-QP.
- Use source sample timestamps with defined clock semantics, not ROS callback arrival intervals. Reset derivative history at initialization, trigger re-engagement, invalid/stale input, session/reference-space changes, and reconnect; first valid sample after reset has zero desired velocity.
- Reject nonpositive or otherwise invalid timestamp intervals; do not differentiate across discontinuities.
- Update the filter using valid source sample intervals: `a = exp(-2 * pi * twist_filter_cutoff_hz * dt)`; `filtered_twist = a * previous_filtered_twist + (1 - a) * blended_twist`, with `twist_filter_cutoff_hz: 10.0` initially.
- Derivative intervals are configurable, initially 1–100 ms. Do not advance the filter using rejected timestamps.
- Desired twist, including differentiated and native contributions, is enabled only during valid, fresh, trigger-enabled teleoperation; use zero desired velocity during pause, invalid/stale-input hold, or disarmed states. Reset filter state on these transitions so residual filtered velocity cannot persist.
- Preserve unscaled native twist for recording in both feedforward modes. Do not estimate feedforward from ROS callback arrival intervals.
- If native linear or angular velocity is invalid, fall back to the corresponding pose-derived component when pose and timestamp differentiation are valid, regardless of the configured blend weight. This is validity handling inside the continuous blend, not a separate user-selectable mode.
- If neither estimate is valid for a required twist component, disable motion and transition toward measured-pose hold with zero desired velocity.
- After derivative-history reset, the first valid pose initializes history with zero pose-derived velocity; do not differentiate across the reset.

## Confirmed pose mapping

- Track the OpenXR controller grip pose, not aim pose; trigger is motion permission.
- Use OpenXR `LOCAL` reference space.
- Capture controller and measured robot pose anchors at each trigger engagement; use relative translation and quaternion rotation, mapped through a configurable reference-to-robot basis.
- Initial translation and rotation mapping is 1:1.
- Apply consistent frame conversion and motion scaling to pose-derived and native twist before blending.
- Reference-space changes/recentering disarm motion, reset derivative/filter history, and require fresh anchors plus explicit re-engagement.
- Use documented manual reference-frame calibration for milestone 1, saving a configurable per-arm rotation from Quest LOCAL coordinates into the robot base frame; automatic calibration is not required.
- Relative translation offsets are established by engagement anchors; the saved rotation establishes axis alignment.
- Verify calibration during the first teleoperation hardware test using small controller motions by hand: isolated translation axes followed by small rotations, checking direction, sign, scale, and absence of cross-axis errors. Stop and correct calibration if motion differs from expectation.
- Transform convention and manual calibration are documented in `docs/ROBOT_FREE_MAPPING.md`; physical amplitudes/acceptance remain commissioning decisions.

## Confirmed verification sequence

- Require a robot-free verification gate before physical teleoperation.
- Build the APK and ROS packages in Docker; test physical USB streaming, timestamp handling, configurable recording, disconnect/reconnect, and session changes without connecting the arm controller to physical FCI.
- Verify pose mapping, quaternion differentiation, twist blending/filtering, trigger/grip logic, and gripper toggles against mocked hardware.
- Inject stale/invalid input and verify disarming, zero desired velocity, and measured-pose hold commands against the mock backend.
- Only after this gate perform the single-arm hardware test, including small hand motions to verify manual calibration and controlled gripper tests with the intended cables.
- Verify arm homing against mock hardware before deployment; verify physical clearance separately during deployment commissioning.
- Detailed test amplitudes, deployment measurements, and acceptance thresholds remain pending; passing mocks does not establish physical safety.

## Approved implementation order

All stages below belong to milestone 1: single-Panda VR teleoperation with impedance control and Franka Hand actuation. Multi-arm launch support is included; simultaneous multi-arm hardware validation is not required for milestone 1.

1. Reproducible environments and interfaces
   - Add pinned Android build/ADB containers and extend containerized ROS tooling without altering the existing Panda packages.
   - Verify the legacy firmware/libfranka compatibility baseline and dependency resolution.
   - Add the approved package layout, raw state and atomic command schemas, per-arm configuration, and customizable recording configuration.
2. Quest application and USB bridge
   - Implement minimal OpenXR rendering/session lifecycle, both controller action sets, grip poses, native velocity/validity, timestamps, and status display.
   - Implement bounded versioned TCP framing over ADB reverse, session/sequence validation, clock synchronization, and reconnect disarming.
   - Preserve left/right identities; assign only the right controller to the first physical arm by default.
3. Mapping and twist processing
   - Implement manual basis calibration, trigger anchors/hysteresis, quaternion pose mapping/differentiation, continuous native/derived twist blend, and 10 Hz filtering.
   - Implement the 100 ms watchdog, validity fallback, explicit re-engagement, and atomic per-arm commands.
4. New arm controller and gripper worker
   - Copy and adapt impedance code in the new package only; isolate realtime command handoff and non-realtime state publication.
   - Implement controller freshness, outer-envelope checks, measured-pose hold transitions, and torque-rate compliance verification.
   - Implement gripper toggle/debounce, explicit calibration, configurable object parameters, status reporting, and approved failure/interruption behavior.
   - Add separate per-arm homing with guarded X/A hold-release, ownership transfer and controlled cancellation, followed by Panda-adapted CBF-QP filtering.
5. Robot-free acceptance gate
   - Build all deliverables inside Docker and test the Quest USB path without physical FCI ownership.
   - Unit/integration-test quaternion edge cases, source timing, derivative/filter reset, command freshness, namespace isolation, gripper behavior, and logging configuration against mocked hardware.
   - Inject malformed, stale, duplicate/out-of-order data, lost tracking, session/recenter changes, bridge death, and reconnect; verify no automatic motion resumption.
6. First hardware teleoperation test
   - Verify host RT setup, permissions, robot firmware/FCI, payload/tool settings, physical clearance, and observer-accessible stop before arming.
   - Select commissioning gains and object-specific gripper parameters; verify empty-hand calibration explicitly.
   - Verify the reference-frame mapping with small hand motions, then single-arm position/orientation tracking and cable grasp/open toggles.
   - Test pause/re-engagement and approved failure behavior in controlled conditions; measure timing and record configurable topics.
   - Document physical acceptance results and issues before deploying arm homing or CBF-QP; FR3 remains outside this implementation scope.

## Implementation handoff

### Historical initial coding pass (superseded by current snapshot)

- Added ROS interface package `src/quest_teleop_interfaces` with `QuestControllerState` and `CartesianTeleopCommand`. `src/quest_teleop` now publishes atomic `CartesianTeleopCommand` samples; no arm-controller consumer exists yet. This publisher is robot-independent and must not be connected to physical FCI.
- `src/quest_teleop` listener defaults to `127.0.0.1:45454`; protocol is uint32 network-order length + UTF-8 JSON (max 4096 bytes). It validates fields/session/global sequence/per-controller timestamp order, replies to sync requests, converts Quest acquisition time to PC `CLOCK_MONOTONIC`, applies configurable age/future/uncertainty gates, and publishes raw state on shared `~/controller_state` plus `~/left/controller_state` and `~/right/controller_state`. `Header.stamp` is ROS receipt time; converted acquisition time is separate. Disconnect publishes invalid state for both controllers; malformed frames close the connection. Action active flags distinguish unavailable OpenXR actions from released inputs.
- Protocol in `docs/quest-usb-protocol.md` includes Quest-originated four-timestamp sync RPCs every 1 s on the same TCP stream. Estimate PC-minus-Quest offset with `((p2-q1)+(p3-q4))/2`; estimated uncertainty is RTT/2 + 0.1 ms, grows at 50 ppm, and assumes approximately symmetric path delay (not a hard bound). Initial gates: uncertainty <=10 ms, sync age <=3 s, converted acquisition time <=10 ms future and <=100 ms old. These are unmeasured provisional settings.
- `android/quest_teleop` samples both Touch grip poses, native `XrSpaceVelocity`, validity, inputs, LOCAL-space generation, focus, and Quest monotonic acquisition time. OpenXR action `isActive` is preserved separately from trigger/grip/button values. TCP transport retains queued samples rather than coalescing per hand; queue overflow drops the connection and disarms. Optional `XR_FB_passthrough` is enabled when runtime exposes it and permits creation; projection background uses alpha with opaque fallback. Headset runtime/permission behavior is unverified. GLES UI still shows only USB, clock, right tracking, and right deadman indicators; readable per-controller status/pose/twist display remains unresolved. APK is telemetry-only; no robot command path.
- ADB Android/ADB containers are in `docker/android.Dockerfile` and `docker/android-compose.yaml`. Both ROS and ADB containers use host networking; ADB server binds loopback and USB device access is privileged. Reverse tunnel and install scripts: `scripts/quest_adb_reverse.sh`, `scripts/quest_install.sh`.
- Base images and Android build toolchain are pinned. Apt transitive package versions are not locked. Android container installs NDK 26.3, but Gradle currently also installs/uses NDK 26.1 because project `ndkVersion` is not pinned; resolve this toolchain drift.
- Added `src/teleop_controllers` as a verbatim copy of the legacy controller. It still consumes `PoseStamped`, publishes in its real-time callback, and lacks the agreed safety changes. Do not use it to control hardware yet.
- `src/panda_bringup` and `src/panda_demos` remain unchanged.

### Historical verification and hardware status (superseded by current snapshot)

- ROS and Android images build; APK packages; interface and teleop packages build in Docker. Quest USB telemetry, both controller identities, tracking, and valid source timestamps were manually verified. User-provided Quest telemetry from 2026-10-10 confirms per-hand action activity and left tracking loss; button values were all false, so it did not reproduce held-button flicker. After TCP_NODELAY and single-write framing, sync RTT was 1.5–3 ms and uncertainty 0.82–3.5 ms (inside the 10 ms gate).
- Mapper builds with 16 unit tests passing; one robot-free TCP bridge→raw ROS→mapper→mock-pose integration test also passes. Integration covers separate left/right raw topics and button fields, stale/invalid-clock/tracking/reference changes, malformed/duplicate/out-of-order frames, reconnect/session changes, watchdog disarm, and release/re-engagement before motion resumes. Stale measured-pose and inactive-trigger faults also require release. No FCI consumer is involved.
- User reports button values flicker within one controller ID. The 2026-10-10 sample shows left X/Y/menu actions active and A/B inactive; right A/B active and X/Y/menu inactive, matching the suggested bindings. Button values shown are all false, so no button was held and the flicker was not reproduced. Left pose validity/tracking is false in the sample; right pose/twist are valid. Per-controller topics and role-specific button fields are implemented; alternating global sequences/IDs are expected. Meta/system button is not bound/exposed. Still require consecutive same-ID values while holding a button. No debounce/filter added.
- User reported Quest `xrEndFrame` returning -23 (`XR_ERROR_LAYER_INVALID`) after optional passthrough integration. Updated passthrough composition layer to use `XR_NULL_HANDLE` space, rebuilt APK successfully; headset retest is pending. Readable per-controller status/pose/twist UI, firmware/libfranka compatibility, full robot-free acceptance, and robot tests remain unverified. No hardware/deployment-PC inspection or FCI operation occurred; robot remains disconnected. `panda_bringup` and `panda_demos` unchanged.

### Current implementation tracking

See `docs/IMPLEMENTATION_PROGRESS.md` for the current procedure. Older baseline and verification paragraphs above describe earlier passes; they do not reopen verified Quest UI, passthrough, USB reconnect, button-schema or NDK work.

### Remaining deployment decisions

- Object-specific Franka Hand grasp width/tolerances, fingertips, physical calibration and operator procedure.
- Home path and physical clearance; table/base transforms, collision geometry/padding, payload/tool configuration.
- Commissioning gains, workspace tool offset, practical motion bounds, stop procedure, measurable acceptance thresholds.
- Verify pinned firmware/libfranka compatibility, active deployment RT kernel, permissions, scheduling, memory locking and timing. A successful compile is not physical compatibility evidence.
- Validate clock-delay assumptions and measured jitter without relaxing the 100 ms age/10 ms uncertainty gates.
- CBF feasibility, failure/stop behavior, 1 kHz worst-case timing, coherent dual-arm state age/skew, and geometric validity require commissioning measurements. CBF joint/collision rows are not a singularity guarantee.
