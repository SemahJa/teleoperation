# Revised implementation procedure

Current scope: complete the requested package reorganization before Milestone 2
asynchronous CBF-QP and performance work; see [PACKAGE_STRUCTURE.md](PACKAGE_STRUCTURE.md)
and [MILESTONE_2.md](MILESTONE_2.md).
Physical FCI/device operations remain operator actions. Preserve Docker,
`panda_bringup`, `panda_demos` and the working nominal controller baseline.

## Stages and evidence

### Hand startup calibration without a launch switch (2026-10-12)

- Operator requests that `start_hand:=true` alone enables automatic calibration.
  Remove `hand_calibrate_on_start` from combined launch and `calibrate_on_start`
  from standalone gripper launch arguments. The gripper launch always enables
  the worker's internal startup calibration behavior; no user launch toggle
  is exposed. `start_hand:=false` starts no worker and causes no calibration.
- Retain the explicit manual service for recalibration/fault recovery and no
  automatic retry or grasp. Update current guides and policy; prior entries
  below describe the superseded optional launch interface.
- Standard build/install updated; all 22 affected gripper/bringup robot-free
  cases pass. Installed launch help confirms neither launch exposes a
  calibration argument. No physical calibration was initiated by tooling.

### Automatic Hand calibration and cancelled-home measured hold (2026-10-12)

- Operator requests automatic calibration for `start_hand:=true` after successful
  manual calibration. Combined launch defaults `hand_calibrate_on_start:=true`;
  standalone gripper launch defaults `calibrate_on_start:=true`. Worker issues
  calibration once through its existing asynchronous Hand thread, retains
  hardware IP/operator confirmation, and never automatically grasps or retries.
  Manual service and opt-out remain available. Starting Hand now moves empty
  fingers; guides and policy explicitly document the changed startup behavior.
- Latest capture `teleop-diagnostics-20261012T154207Z-11260` starts home at
  ROS time 1791819731.191. At 1791819739.231, while still `homing` (elapsed
  8.042 s, home error 0.01329 rad), trigger press cancels it. No `succeeded`
  transition is captured. Owner PID 11183 later reports controlled-stop failure
  with reference error 0.01323 rad and velocity 0.00276 rad/s.
- After reference deceleration completes and measured velocity is below the
  existing threshold, cancellation captures measured joint hold once. Settling
  then checks velocity and displacement from that measured hold, rather than
  pursuing an unfinished home reference. Preserve successful-home accuracy,
  stop timeout, velocity threshold, actuator bounds and safety filter. An arm
  that fails to settle still ends control; never report cancelled home as success.
- Rebuilt standard install. All 53 affected individual robot-free cases pass:
  controllers 31, gripper 12, bringup 10. Added real-worker one-time startup
  calibration coverage and static-offset cancellation regression; moving-arm
  stop-failure test remains. Updated combined launch test to expect automatic
  calibration. No physical command was issued by tooling; hardware retest remains.

### Hardware recording launch and orphan mapper cleanup (2026-10-12)

- Operator reports hardware following stopped working after a mock trial.
  Read-only inspection finds orphan mapper PID 10491, parent PID 1, publishing
  to the same `/panda_1/cartesian_teleop_command` topic used by subsequent launches.
  Latest owner PID 10829 connected to FCI in Idle and exited on operator SIGINT;
  its log does not show a controller exception. Duplicate mappers can interfere
  with command ordering/engagement; hardware following was not observed by tooling.
- Stop only the verified orphan input mapper with SIGINT. The subsequent graph
  has no Panda teleop nodes and the command topic is absent, confirming cleanup.
  No arm owner, Hand operation or robot command was started by tooling.
- README section 5.0 now gives explicit real-hardware Menu recording commands
  using commissioned arms/runtime files and the accepted CBF-disabled baseline.
  Disable homing/Hand for this test, check single command/measured-pose publishers,
  inspect finalized bag contents and test successive episodes/shutdown finalization.
  All added shell command blocks pass syntax validation.
- The earlier `homing-continuity.txt` also contains a controlled-stop settling
  failure (reference error 0.01364 rad versus the 0.01 rad tolerance), distinct
  from the latest cleanly interrupted hardware trial. Do not treat this cleanup
  as a fix for that settling failure or relax stop tolerances. Physical following
  and recording acceptance await the operator's documented hardware trial.

### Standard workspace setup restored (2026-10-12)

- Operator confirmed hardware homing succeeds after the watchdog correction.
  Gripper operation remains untested.
- Archive previous generated `build/` and `install/` outputs under
  `.workspace-artifacts-20261012T145440Z` with `COLCON_IGNORE`, then rebuild
  libfranka, legacy workspace packages and all current teleoperation packages
  into standard `build/` and `install/` paths. Source and commissioning logs
  are preserved; protected Panda source hashes remain unchanged.
- A clean shell sourcing only `/ws/install/setup.bash` resolves all six feature
  packages plus the Panda packages to their standard prefixes. Neither the
  retired controller package nor `install/reorganized` remains in active install.
  README and package structure instructions now use the standard setup.
- All 100 individual feature-package robot-free cases pass in isolated ROS
  domain 82; colcon reports 117 tests including CTest wrapper results, with
  zero errors/failures/skips. Logs: `log/commissioning/current/build-standard-*.txt`
  and `test-standard-workspace.txt`. No physical device command was issued.
- Document the runtime executable's responsibilities and the future separation
  of common input/actuator gates from selectable nominal controller strategies;
  no new controller law or runtime selection interface is implemented here.

### Homing input watchdog clock correction (2026-10-12)

- New capture `teleop-diagnostics-20261012T144159Z-1853` shows request 1
  accepted, running for 1.638 seconds, then cancelled by `controller watchdog`.
  Quest remains valid with source/receipt sample gaps below 28 ms during home.
- The owner used the earlier RobotState timestamp for input freshness after
  consuming input received during model preparation. Such input could appear
  to have a future receipt time and spuriously cancel homing or disarm teleop.
  Check both input watchdogs using monotonic time after queue consumption;
  retain the original model timestamp for CBF age/deadline checks.
- Preserve all input freshness thresholds, trigger gates and controlled-stop
  protections. Add a regression for input received during model preparation
  and genuine subsequent expiration. Hardware retest remains an operator action.
- Rebuilt and installed `teleop_controllers` in `install/reorganized`;
  all 30 individual controller/homing/runtime tests pass in isolated ROS
  domain 81, including successful homing followed by continued owner operation.

### Homing continuity and actual-fault diagnostics (2026-10-12)

- Operator reports homing reaches home but the launch disconnects afterward.
  Captured A hold/release lasts 1.9585 seconds with trigger zero and starts
  request 1. The capture ends while homing is active, at elapsed 1.317 seconds;
  no terminal home phase is recorded. Earlier rejected attempts are not in it.
- Owner PID 469 later exits with the combined invalid-state/actuator/homing-stop
  error, code 2, which causes launch to terminate the bridge and mapper.
  CBF is disabled and all captured filter results succeed. Its last valid model
  snapshot cannot identify the subsequently rejected callback state.
- Make mock-loop and FCI connection finishing explicitly depend on shutdown
  intent plus the matched coordinator finish reply. Successful homing remains
  a transition back to disarmed Cartesian control within the same owner.
- Add specific core fault categories and post-join owner diagnostics containing
  the actual failing period, homing phase/message, home/reference errors and
  stop failure flag. Keep all invalid-state, actuator and stop protections.
  Idle homing now reports input/gesture rejection and readiness messages;
  the read-only diagnostic script includes homing status.
- Reorganized Docker build passes. All 39 affected individual robot-free cases
  pass (29 controller/homing/core/runtime, 10 bringup), including owner survival,
  continued telemetry and deliberate re-engagement after successful homing.
  A final diagnostic-format-only rebuild installs the reference-error field.
- Evidence and limits: `log/commissioning/current/homing-diagnosis.md` and the
  preserved `homing-owner-469.log`. The exact old hardware fault remains
  unresolved; an operator rerun must validate hardware continuity or capture
  the new precise `Owner fault` line. No physical command was issued by tooling.

### Package reorganization verified (2026-10-12)

- Quest transport/mapping remain in `quest_teleop`; recording implementation,
  topic planning and tests move to `recording`. Rename the arm package to
  `teleop_controllers`, retaining the impedance core, homing core and joint
  trajectory generator. Extract Hand logic/worker into `teleop_gripper` and
  CBF/OSQP/coordinator into the exported `safety_filter` library.
- Each package has feature launch files and parameters. `teleop_bringup`
  composes bridge, mapper, arm owner, guarded homing, Hand, CBF and recording
  with independent switches. Homing and safety retain the exclusive arm owner;
  no competing FCI connection or ROS torque service is introduced.
- Existing combined runtime/arms commissioning YAML schemas remain usable.
  Feature-specific files can override controller, gripper and safety settings.
  Integrated commands and recording configuration paths are migrated in the
  README/runtime guides; standalone and fresh-overlay instructions are in the
  package structure guide.
- Fresh Docker build in `build/reorganized` / `install/reorganized`, sourced
  only from Humble, passes all nine packages in the dependency graph. The six
  feature packages pass 97 individual robot-free cases: Quest 22, recording 4,
  safety 23, arm controllers 27, gripper 11, bringup 10. These include actual
  standalone launches, custom safety-file loading, single-right/left/dual,
  CBF-off, homing-off, independent feature selection, conflicting fixture
  rejection and finalized recording bags.
- Corrected standalone mapper quaternion parsing for the installed Humble
  launch API; standalone mapper/bridge launches support explicit namespaces.
  Integration tests wait for a measured pose and accepted release before
  engagement, and isolate their ROS topics/namespaces.
- SHA-256 comparison confirms all nine files in `panda_bringup` and
  `panda_demos` unchanged. No physical FCI/Hand operations or Android build.
  Milestone 2 controller/async changes remain unimplemented.

### Milestone 1 accepted; Milestone 2 scoped (2026-10-12)

- The operator reports successful single-right-arm hardware teleoperation with
  CBF disabled and explicitly accepts Milestone 1. This establishes the Quest,
  mapper and nominal controller baseline; other commissioning checks retain
  their individual status.
- Next work: asynchronous CBF-QP for single-arm experiments first, then smooth
  references and distinct controller alternatives evaluated with synchronized
  target/reference/measured-motion plots and tracking metrics.
- The implementation plan defines current-state validation of asynchronous
  results, bounded result age, nonblocking owner behavior, failure handling and
  comparative experiments. These new features are planned, not implemented or
  validated by this documentation update.

### Hardware-startup diagnostic follow-up (2026-10-12)

- User-reported hardware startup first aborted with an invalid ROS context,
  then reached a latched `invalid_model` stop after activating FCI. The latter
  cause remains unresolved: the original log does not identify the rejected
  matrix, frame, callback period or actuator interval.
- Construct the ROS executor synchronously before starting hardware; catch
  executor-thread exceptions and join through bounded `spin_once` polling and
  cancellation. Defer shared ROS-context shutdown until the main process has
  joined all owners. Early hardware errors no longer race executor creation.
- Preserve the coordinator's final consumed model snapshots and filter status.
  After owners/coordinator join, report period, finite-state flags, inertia
  symmetry/positive-definiteness, rejected frame matrices and empty actuator
  intervals. No additional logging in torque/solver callbacks and no relaxed
  validity, timing, clearance or actuator checks.
- Existing ROS container rebuild passed. Isolated robot-free regression passed
  16 individual checks across five suites: coordinator (6), controller (3,
  including five repeated first-solve failures), homing process (1), CBF
  runtime (3), and integrated single/dual launch (3). Logs:
  `/tmp/franka-controller-diagnostics-build.txt` and
  `/tmp/franka-controller-diagnostics-tests.txt` inside the container.
- No hardware session was opened by the agent. A subsequent operator-supervised
  hold attempt must provide the new diagnostic output before fixing or accepting
  the hardware model.

### Legacy end-effector rounding fix (2026-10-12)

- The user's subsequent diagnostic isolated frame 8 (end effector): inertia,
  state finiteness and joint/flange transforms passed; rotation determinant
  `0.999980746901` and orthogonality error `1.92530987919e-5` exactly match a
  45-degree rotation using float-rounded `0.7071` coefficients. This identified
  the previously unresolved numeric model rejection.
- Add bounded fixed-size orthonormalization of the measured end-effector pose
  and model end-effector frame at the hardware adapter, following the pinned
  libfranka 0.9.1+ convention. Require finite/homogeneous transforms, positive
  determinant, determinant/Gram-matrix errors at most `1e-4`, and a correction
  at most `1e-4`. Reject reflections, degenerate or substantially distorted
  transforms; preserve translation, Jacobians, joint/flange frames, and the
  CBF filter's existing `1e-8` rotation checks.
- Existing container build and 24 focused robot-free cases passed (12 filter,
  6 coordinator, 3 controller, 3 CBF runtime). Regressions reproduce the raw
  rejection then successful conversion, retain invalid-transform stops and
  demonstrate zero conversion-path allocations. Container logs:
  `/tmp/franka-legacy-pose-build.txt`, `/tmp/franka-legacy-pose-tests.txt`.
- No agent hardware session was opened. Physical hold/stop and worst-case
  callback timing after this correction still require operator commissioning.

### Hardware result-deadline investigation (2026-10-12)

- The operator's next launch progressed past model validation and stopped with
  `result_deadline`. The existing log cannot distinguish model preparation,
  coordinator scheduling, filter cost or receipt delay within the 900 us budget.
- Add fixed-size owner timing and preserve coordinator start/completion/result
  for postmortem logging after all realtime threads join. Match both cycle and
  source timestamp before displaying coordinator timings. Missing results are
  explicitly unavailable. Keep all hardware deadlines and stop behavior intact.
- Avoid a second SVD when constructing the measured orientation from the
  already validated and orthonormalized pose.
- Existing container rebuild and 25 focused robot-free cases passed: filter
  (12), coordinator (7), controller (3), CBF runtime (3). New regression verifies
  that late-result timing survives the latched stop while late results remain
  rejected. Logs: `/tmp/franka-timing-build.txt` and
  `/tmp/franka-timing-tests.txt` inside `franka-guide-ros`.
- Read-only machine inspection found PREEMPT_RT enabled, no container CPU quota,
  16 allowed CPUs and the inspected CPU governor `schedutil`. These observations
  do not establish which stage missed its deadline. No agent hardware launch
  occurred; operator timing evidence is still required to resolve the fault.

### Desk/FCI startup-state mismatch investigation (2026-10-12)

- Operator reports Desk ready before and after a custom launch, while the robot
  rejects libfranka's Move request in Reflex mode before the torque callback.
  Both commissioning configurations target `192.168.3.101`; neither controller
  calls automatic error recovery. The rejection is the robot's Move-response
  status, not an inference from the custom CBF filter.
- Read one robot state immediately before requesting control and log IP, robot
  mode, current errors and last motion errors. This read-only diagnostic occurs
  outside the realtime callback and does not clear errors or change safety gates.
  Build verification is recorded in `/tmp/franka-startup-state-build.txt` in
  the existing ROS container. No hardware session was opened by the agent.
- The reason for the Desk/FCI discrepancy remains unresolved pending the
  operator's startup snapshot. Earlier CBF deadline diagnosis is still pending.

### CBF structural sparsity timing fix (2026-10-12)

- Operator snapshot now shows Idle, no current errors, and historical
  `communication_constraints_violation`. First-cycle timing isolates filtering:
  preparation 20.359 us, coordinator delay 4.368 us, filter 2230.332 us, maximum
  iterations with a feasible iterate. Owner correctly rejects it at 908.544 us
  against the unchanged 900 us result deadline.
- Replace fully populated CSC storage (including structural zeros) with a
  configured fixed sparsity pattern. Actuator rows support one coefficient;
  same-arm rows support only their seven joints; inter-arm rows support both
  arms when both are active. Permanently disabled barriers have no structural
  coefficients. Preserve dense logical coefficients, all bounds and independent
  candidate feasibility checks. No iterations, tolerances, clearance settings,
  fallback policy or deadlines were relaxed.
- Existing ROS container build and 27 robot-free checks pass: filter 13,
  coordinator 7, controller 3, CBF runtime 3, homing process 1. New regression
  covers either active arm with table/obstacle constraints and verifies zero
  runtime allocations. Logs: `/tmp/franka-sparse-build.txt` and
  `/tmp/franka-sparse-tests.txt`.
- Same-host synthetic Panda fixture benchmark (identity inertia/zero coriolis,
  not hardware) reduced single-arm mean filter time from 286 to 86 us. A tight
  tolerance stress fixture exercising 100 iterations on its first call fell
  from 1242 to 381 us. Dual stress first-call time fell from 1193 to 788 us, but
  the subsequent run still observed a 982 us maximum; no dual timing guarantee
  is claimed. Before/after data is in container `/tmp/cbf-bench*.txt`, source in
  ignored `log/commissioning/current/cbf-bench.cpp`.
- No hardware session was opened by the agent. Real inertia, configured pose,
  CPU jitter and worst-case 1 kHz timing still require operator validation.

### Compact solver rows and real-model capture (2026-10-12)

- Operator's optimized controller accepted 16 cycles, then rejected cycle 17:
  preparation 30.087 us, coordinator delay 11.382 us, filter 987.413 us, total
  owner time 911.389 us. Solver still exhausted its iterations. The prior
  structural sparsity change improved startup but did not resolve timing.
- Remove permanently empty solver rows through a fixed configure-time mapping.
  Retain both actuator lanes, every enabled joint barrier, and all potentially
  active table/self/inter-arm/obstacle collision rows. Keep full logical rows
  and independent residual checking. Configuration changes rebuild the mapping;
  geometry activation only changes numeric coefficients and bounds.
- Build and 28 focused robot-free cases pass (filter 14, coordinator 7,
  controller 3, CBF runtime 3, homing process 1). New test verifies mapping
  rebuilds, mandatory actuator rows, and clear-to-blocked table activation.
  Logs: `/tmp/franka-compact-build.txt`, `/tmp/franka-compact-tests.txt`.
- Same synthetic fixture single-arm mean fell from 86 to 67 us, while the
  first-call 100-iteration stress case measured 408 us (prior sparse run 381 us).
  The new benchmark does not establish improved worst-case hardware timing.
  Container evidence: `/tmp/cbf-bench-compact*.txt`. Deadlines, iterations,
  feasibility tolerances and safety parameters remain unchanged.
- Add solver duration/iteration count to timing diagnostics and write the exact
  consumed failing model to container `/tmp/franka-cbf-model-arm-N.txt` after
  threads join. This allows subsequent offline diagnosis using real inertia
  and constraints rather than synthetic fixture approximations. Export-only
  follow-up build: `/tmp/franka-compact-snapshot-build.txt`. No hardware session
  was opened by the agent; real-model evidence and timing acceptance remain.

### Recorded-model replay and exact feasible-nominal optimum (2026-10-12)

- Operator's next timing stop occurs at cycle 1772: preparation 24.276 us,
  coordinator delay 19.978 us, solver 795.219 us/100 iterations, filter 886.983 us,
  owner total 919.996 us. Preserve exported model in ignored commissioning
  `cbf-failing-model.txt`. Robot-free replay source: `cbf-replay.cpp` alongside it.
- The same recorded model initially takes about 160 us for 100 solver iterations
  offline and converges toward zero intervention. This does not reproduce the
  deployment timing; snapshot replay omits prior constraint/Jdot solver history and
  scheduling/clock effects. Read-only host inspection finds `schedutil`.
  Franka recommends disabling frequency scaling for realtime control. Attempt
  to set `performance` fails because sudo requires a password; governor remains
  unchanged. README now includes operator host commands and verification.
- Add exact optimum detection: after building all rows and updating objective
  weights, check the nominal torque against every original bound with zero
  violation, zero inactive lanes and seed OSQP's primal warm start. If feasible,
  return it with `feasible_nominal`, zero iterations and zero intervention on
  active joints. Positive weighted torque-distance cost cannot improve on this
  candidate. Any violation, even below existing numerical acceptance tolerance,
  keeps the iterative QP path. No safety bounds, deadlines, solver iterations
  or tolerances changed.
- Recorded-model replay now reports zero violation and iterations, about 21 us
  mean/36 us maximum at unchanged rho=1 in this run (not a hardware timing
  guarantee). Evidence: container `/tmp/cbf-real-before.txt` and
  `/tmp/cbf-real-after.txt`.
- Build and 29 robot-free checks pass: filter 15, coordinator 7, controller 3,
  CBF runtime 3, homing process 1. Regression covers exact optimum, rejection of
  tiny actuator violations, return to iterative solving, and zero allocations.
  Existing joint/collision/fallback tests still pass. Logs:
  `/tmp/franka-feasible-build.txt`, `/tmp/franka-feasible-tests.txt`.
- No agent hardware session occurred. Host governor setup, hardware timing and
  the previously reported teleoperation engagement failure remain to validate.

### Teleoperation engagement evidence collection (2026-10-12)

- Operator reports stable custom startup but no following after right-trigger
  release/hold. Inspected launch assigns the right raw input to the mapper and
  routes its atomic command to the arm owner. Legacy `controller_enabled` and
  `operator_confirm` flags in arms YAML are not mapper engagement switches;
  hardware ownership confirmation comes from the integrated launch argument.
- Inspection shell cannot discover a live `/panda_1/controller_status`; saved
  bridge sync logs alone cannot establish valid tracking or motion engagement.
  Do not infer a cause or change gains, transforms or safety gates without
  command/status evidence from the launch's ROS environment.
- Add read-only `scripts/teleop_diagnostics.sh`: parallel bounded subscription
  capture of right raw input, atomic commands, anchor pose, owner status and
  CBF status. Save ROS discovery settings and separate stderr under a unique
  commissioning directory. No FCI connection or command publication. Shell
  syntax and invalid-argument handling checked; no hardware operation occurred.
  README section 5.2 gives the operator capture command. Engagement cause
  remains pending those observations.

### Captured engagement and subsequent deadline observations (2026-10-12)

- Inspect operator-provided topic capture and preserve/replay cycle-6438 model.
  Full findings are in ignored commissioning `teleop-diagnosis.md`. Latest
  filter exceeds both existing budgets: filter 886.111 us, owner 922.501 us.
  Recorded-model replay first call is about 194 us at 100 iterations; static
  replay is not deployment worst-case timing. Host still reports `schedutil`.
- Earlier topic trace proves mapper engagement/target changes but only one
  active owner telemetry sample, immediately followed by `safety_filter`.
  Fallback clears owner release authorization; held-trigger inputs cannot
  reauthorize motion until a fresh valid release. Invalid tracking and stale
  input also occur. No transform/gain change is supported by this evidence.
- Timing limits are provisional software budgets. Increasing only result
  timeout leaves the filter stop intact; enlarging both requires a reviewed
  complete 1 ms cycle/network budget and measured worst-case behavior. It also
  does not address mapper/controller engagement loss. Limits remain unchanged.
- Inspection/replay only; no new runtime implementation or hardware operation.

### Explicitly requested CBF-off diagnostic comparison (2026-10-12)

- User explicitly requests teleoperation without CBF to isolate the nominal
  controller. This request supersedes the earlier enabled-only coordinator
  requirement for this diagnostic mode. Add default-true launch argument
  `cbf_enabled`; pass it as `cbf.enabled` and allow coordinator configuration
  with the existing actuator-bounds-only filter branch.
- `cbf_enabled:=false` skips joint/collision CBF constraints and QP solving,
  reports `disabled_actuator_bounds_only`, and emits a startup warning. Preserve
  trigger/freshness/tracking/envelope/torque guards, model validity, realtime
  scheduling and all coordinator deadlines. Manufacturer protection settings
  are untouched. No automatic robot recovery or hardware launch was performed.
- Build and 30 robot-free checks pass: filter 15, coordinator 8, controller 3,
  integrated launch 4. New coordinator regression retains actuator slew and
  invalid-model stops despite disabled CBF; new launched mock comparison verifies
  actual engagement and disabled-mode telemetry. Logs:
  `/tmp/franka-cbf-disabled-build.txt`, `/tmp/franka-cbf-disabled-tests.txt`.
- README section 5.2 and CBF design document distinguish this reduced-protection
  diagnostic from normal hardware/collision acceptance. Physical comparison
  remains an operator action; default runtime files continue to use CBF.

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
ros2 run teleop_controllers cartesian_impedance_controller --ros-args \
  -r __ns:=/panda_1 -p backend:=mock
```

In a **second sourced ROS container terminal**, reuse the validated Quest USB
bridge and launch the mapper without another synthetic pose publisher:

```bash
ros2 launch teleop_bringup robot_free.launch.py start_bridge:=false mock_measured_pose:=false
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
ros2 run teleop_gripper franka_hand_worker --ros-args \
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
ros2 run teleop_controllers cartesian_impedance_controller --ros-args \
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
colcon build --symlink-install --packages-select quest_teleop_interfaces quest_teleop teleop_controllers \
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
`build/{quest_teleop,teleop_controllers}/test_results/`: 27 Quest and 54
controller cases, zero failures/errors. CTest wrapper counts are not additional
individual cases. Relevant builds/tests were batched; only failures/substantive
changes prompted focused reruns. These results establish software acceptance,
not physical dynamics, clearance or deployment timing.
