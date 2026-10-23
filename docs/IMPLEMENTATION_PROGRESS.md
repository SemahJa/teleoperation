# Implementation record

Current implementation: separated runtime/manager/torque laws, persistent local
joint position hold, shared CBF-independent actuator caps, homing, Hand control,
recording, full-cycle tracking telemetry and MATLAB report tools. The owner
still waits for the current cycle's safety reply; asynchronous reuse remains
planned in [MILESTONE_2.md](MILESTONE_2.md). Optional pose/velocity filtering and
interpolation are implemented in the mapper and disabled/enabled per arm.

Use the [documentation guide](README.md) for current operating instructions.
The dated entries below are historical evidence; older paths, test counts and
descriptions of release behavior do not override the current implementation.
Physical FCI/device operations remain operator actions.

## Stages and evidence

### Right-Hand opening lockout and late stop acknowledgement (2026-10-23)

- Preserved four recent dual Hand logs under
  `log/diagnostics/right_hand_open_review_20261023/`. In the latest right-Hand
  log (PID 28902, robot `192.168.3.101`), startup calibration succeeded. A
  subsequent stop took 9124.666 ms inside the SDK, then completion reported
  `communication_ok=0` and cleared calibration. Later joystick presses and
  side-grip inputs were fresh/valid but ignored because `calibrated=0`.
  Manually reopening the fingers cannot clear this software fault latch.
- Reproduced a backend race before fixing it: a successful blocking stop
  acknowledgement exceeded the two-second retry window, and command completion
  during the final wait was discarded as failure. The regression now passes;
  successful acknowledgement plus command completion permits opening and a
  later close. The retry window still faults if completion remains outstanding;
  it does not interrupt blocking SDK calls or guarantee physical stop latency.
- Retained distinct unsuccessful responses, command exceptions, device/
  communication errors and retry exhaustion instead of overwriting them with
  one generic timeout. Genuine stop faults are not retried. A later transport
  fault cannot be hidden by an earlier successful acknowledgement; ordinary
  redundant command rejection after acknowledgement remains tolerated while
  waiting for command completion. New logs expose `stop_acknowledged` and
  `stop_retry_exhausted`; `/gripper_events.message` retains the fault detail
  until explicit recalibration starts. Error logs name the affected calibration
  service. Motion remains serialized and genuine faults require explicit
  empty-hand calibration; there is no automatic reconnect/calibration.
- The older hardware logs discarded the actual stop response, so they cannot
  prove that the reproduced race caused this specific trial's fault. No device
  latency or physical gripping result is inferred from the software fix.
  Updated the runtime guide with diagnosis and per-arm recovery instructions.
- Release build succeeded with tests explicitly enabled (the container cache
  had `BUILD_TESTING=OFF`). Gripper: 52 cases (31 core, 17 backend, 4 ROS);
  bringup: 15 cases including 6 actual mock launch modes. All 67 passed without
  failures/errors/skips. Test log: `/tmp/hand-stop-fix-tests.log` inside the
  existing container. APK, runtime values, Docker definitions and third-party
  code were unchanged; no physical FCI/Hand/headset operation was performed.

### Review of calibration timing and engaged overshoot (2026-10-23)

- Inspected the latest dual hardware trial (owner PID 24113, Hand PIDs
  24115/24117). Both calibration requests were dispatched within 0.1 ms.
  Left completion took 9203.199 ms: SDK command 9166.626 ms and readback
  35.979 ms. Right completion took 6598.295 ms: SDK command 6551.431 ms and
  readback 46.169 ms. Both report success/communication OK and calibration
  ready. The difference is inside the SDK homing call, not sequential ROS
  dispatch; these metrics cannot distinguish physical homing from firmware/
  network acknowledgement time. `homing()` exposes no motion-speed argument.
  Desk's indicator during FCI and its disappearance afterward match the
  [maintainer explanation](https://github.com/frankarobotics/franka_ros/issues/118).
  Preserved owner/Hand logs in `log/diagnostics/overshoot_review_20261023/`.
- Audited the current engaged Cartesian law and dual YAML. `D = gain` and
  `K = gain * lambda`, giving translational K=900 N/m and D=30 N s/m, and
  rotational K=75 Nm/rad and D=2.5 Nm s/rad for the current file. Gains are
  not inertia-normalized/adapted, and no explicit nullspace damping or posture
  term is implemented. Hold/homing damping is selected separately; changing
  hold gains does not damp continuously engaged Cartesian tracking.
- Both dual mappings currently use unshaped pose, 10 Hz low-pass velocity
  and no interpolation. Their `feedforward_scale: 0.5` blends native and
  pose-derived velocities; it is not an amplitude multiplier. Zero still
  supplies pose-derived reference velocity. Independent pose/twist processing
  can leave residual twist when pose stops. Actuator caps/rate limiting can
  also change braking, and the latest owner log contains a stale-release
  sample; no tracking bag from the reported overshoot was available to
  identify which effect dominated. No controller gains, filters, torque caps,
  calibration policy or hardware behavior were changed based on speculation.
- Updated architecture and monitoring documentation with the exact damping
  semantics, missing nullspace behavior and diagnostic distinctions. Corrected
  stale text claiming mapper pose interpolation was absent. This review is
  read-only apart from documentation/evidence preservation; no build/test or
  physical operation is needed for these changes.

### Joystick opening and responsive Hand command path (2026-10-23)

- Replaced side-grip double-click opening with one press of each assigned
  joystick. Opening uses the press edge, not release/duration recognition.
  Each joystick needs fresh active release after calibration/reconnection/fault;
  startup-held/unavailable buttons cannot open, and holding cannot repeat.
  Opening cancels held-close demand and waits for any preceding stop; an
  already-held side grip cannot reverse it without a fresh release/press.
  Side-grip hold/release closing and persistent refreshes remain in place.
- Native OpenXR already sampled joystick clicks locally. Added
  `thumbstick_click`/`thumbstick_click_active` to the APK's JSON serializer and
  a named `thumbstick` button in the bridge. Increased the raw message button
  bound to four for left joystick/X/Y/Menu. Existing APK packets without the
  optional pair remain accepted for arm/closing input; partial pairs are
  rejected. Joystick opening requires the updated APK and rebuilt ROS nodes.
- Closing debounce defaults to zero; opening has an independent
  `open_speed_mps: 0.1`, retaining operator closing speeds, widths and refresh
  period in both overall configs. Completion/watchdog polling defaults to
  1 ms on a separate thread, avoiding competition from an always-ready ROS
  timer. Input/core synchronization never holds a lock across SDK, DDS or
  console I/O. A dedicated reporting thread has latest-only status and bounded
  diagnostic backlog; pending motion dispatch precedes completion reporting.
- Cancellation distinguishes command return from final UDP readback, so it
  does not repeatedly stop an already-ended command while readback is pending.
  A possibly overtaken stop can still retry while the command is outstanding;
  condition-variable wake-up follows command return without a lost-notification
  window. Final readback/stop serialization and communication-fault calibration
  requirements remain. Added captured dispatch/collection monotonic timestamps,
  SDK command/readback/stop durations and stop-call count. Deferred console
  prefix timestamps must not be used as physical event timestamps.
- Nine-package ROS Release build and offline Gradle debug APK build succeed.
  All 152 affected cases pass: 30 Quest, 52 controller, 48 Hand, 15 bringup and
  seven recording. New cases cover both-hand USB joystick forwarding, malformed
  pairs, press edges/release gates, held-opening priority, cancellation/readback
  races, 400 ms delayed status publication and independent dual-Hand operation.
  Corrected old debounce/config fixtures and an existing rosbag test using a
  non-Humble `TopicMetadata(id=...)` argument. An initial mixed stress run missed
  a short joystick press with source age near 100 ms; the final separate-polling
  path passes the same test without relaxing freshness. It does not establish
  guaranteed timing on a loaded non-realtime host or physical Hand.
- Documentation, formatting and scoped whitespace checks pass. Software tests
  use localhost ROS domain 137 and mock/injected devices; no FCI, Hand or headset
  operation occurred. The new APK is at
  `android/quest_teleop/app/build/outputs/apk/debug/app-debug.apk`; install it
  with `bash scripts/quest_install.sh` on the host, then restart the sourced ROS
  pipeline. Build/test logs are `/tmp/joystick-hand-*.log` in the ROS container;
  final evidence includes `final-build`, `final-tests`, `timing-build` and
  `verified-tests`. Hardware response remains to be checked by the operator:
  firmware acknowledgement, valid readback and finger travel still take time.

### Separate Desk status from input starvation and Hand readback rejection (2026-10-22)

- The operator identified the warning as Desk's Hand/end-effector panel.
  [Franka's maintainers document this indicator as expected during FCI control](https://github.com/frankarobotics/franka_ros/issues/118).
  It does not establish that our Hand connection failed. The latest hardware
  trial (16:36:10 UTC, owner PID 14043) reports accepted torque cycles for both
  arms, but mapped command source ages reach 1398 ms and motion stays disabled.
  Both firmware calibrations return success; only the right Hand reaches our
  calibration-ready state. Earlier trials show software readback rejection on
  either side. Preserved latest owner/Hand logs under
  `log/diagnostics/dual_desk_20261022/`. No physical operation was performed.
- Reproduced a concrete execution-path defect without hardware: injecting a
  400 ms stall into `robot_state` publication starves the shared ROS executor's
  Quest callbacks and pose anchors, preventing engagement. Split bulk telemetry
  into its own callback group/executor. Input and small pose feedback retain
  a separate executor and fixed-size latest-only handoff; anchors use
  best-effort, depth-one QoS with a matching mapper subscription. Full-cycle
  tracking keeps its existing dedicated queue/publisher. Torque laws, gains,
  actuator caps, deadlines and 100 ms freshness gates are unchanged. The same
  injected-stall regression now passes sustained engagement, release, fresh
  pose feedback and clean shutdown. This reproduces the observed symptom; old
  logs do not identify the exact middleware call that delayed the hardware run.
- The old Hand state check rejects measured width above the estimated maximum
  by more than 1 micrometre, or any negative endpoint noise. Added a shared
  10 micrometre software readback allowance for completion and live closing;
  accepted internal widths are clamped to calibrated travel. Commanded travel
  is not expanded. Larger/invalid readback and transport/stop faults still
  revoke calibration. Raw width, maximum, endpoint excess and communication
  status are now logged together, with an immediate error on rejected
  completion. Old logs omit maximum width, so endpoint discrepancy remains a
  plausible cause of those calibration rejections, not a confirmed hardware
  diagnosis. No automatic calibration/reconnect was added. Input transition
  logging is rate-limited to reduce callback overhead.
- Release builds pass; all 136 affected cases pass: 29 Quest, 52 controller,
  41 Hand and 14 bringup cases. The new regression failed before production
  changes and passes after them. Corrected its test-only CMake environment
  assignment before the final run. The publisher-stall preload library is
  built only for testing, never installed or enabled in hardware launch.
  Formatting, scoped diff whitespace and documentation links pass. Tests use
  localhost ROS domain 137 and mock/injected devices. Container logs:
  `/tmp/desk-repro-build.log`, `/tmp/desk-stall-repro.log`,
  `/tmp/desk-fix-build.log`, `/tmp/desk-fix-tests.log` and
  `/tmp/desk-isolation-test.log`. Physical validation of both arms and Hands
  remains an operator action; further rejection now has explicit readback
  evidence rather than only a Desk indicator.

### Distinguish right Hand calibration from arm/input readiness (2026-10-22)

- Investigated the next dual trial (16:04:58 UTC, owner PID 12066). The operator
  confirmed physical right IP `192.168.3.101`, Desk readiness and fresh right
  trigger/grip release followed by continued nonresponse. The right Hand log
  nevertheless reports calibration success after 6693.558 ms with maximum
  gap 0.0802295655 m. Left calibration reports success after 11122.715 ms.
  No ordinary right-Hand request follows calibration. The arm log contains
  startup `Idle` reads and no controller fault; launch ultimately escalated
  Ctrl+C to SIGKILL. Existing logs do not establish whether the right owner
  entered its first accepted torque cycle or which input gate blocked commands.
  This is not treated as a confirmed calibration failure or a resolved hardware
  control issue. Preserved evidence under
  `log/diagnostics/right_readiness_20261022T160458Z/`.
- Added non-realtime `Arm control ready` logging after the first accepted
  safety-checked torque cycle, and a diagnostic warning if no accepted cycle
  appears after five seconds. The warning distinguishes whether control was
  requested but does not stop/recover/restart hardware. Added raw Quest and
  mapped validity/release/request transition logs. Hand logs now identify
  backend/IP/input assignment, calibration-ready readback and input/release
  gates. No arming, tracking, torque or calibration policies were bypassed.
- Release builds pass. All 18 selected process cases pass: four controller/
  mapper, six CBF runtime, three Hand and five combined-launch cases. One launch
  fixture exposed a DDS readiness race: its observer saw poses before the
  mapper subscription matched. It now waits for both subscribers and a newly
  published pose before trigger engagement; production release gates remain
  unchanged. Formatting, scoped diff whitespace and documentation links/anchors
  pass. Used localhost ROS domain 137 and mock processes; no physical FCI/Hand
  commands. Logs: `/tmp/readiness-*` in the ROS container. A running-trial
  capture of right controller status, Hand status and raw Quest state remains
  necessary to identify the actual right-side failure.

### Independent dual actuator-only lanes after startup deadline stop (2026-10-22)

- Inspected the actual PID 9927 ROS log from the dual hardware launch at
  15:51:09 UTC. Both robots reported `Idle`; the first fatal cause was
  `result_deadline` on cycle 1 with CBF disabled. Arm 0 spent 81.945 us preparing
  and 833.379 us waiting, totaling 915.324 us against its 900 us deadline, with
  no matching result. The coordinator had consumed arm 0's model but had no
  paired result. Arm 1 then observed the shared stop on its first cycle.
  Subsequent Hand stop requests, bridge disconnect and SIGINT messages were
  normal launch teardown. Preserved the controller/launch logs and saved model
  under `log/diagnostics/dual_startup_20261022T155109Z/` before mock testing.
- Removed the unnecessary peer-state barrier only for dual actuator-only
  filtering. Each arm now receives an independently validated current-cycle
  result using its own model, nominal torque, previous robot torque and period.
  Existing effective caps, model/age/filter/result deadline guards and shared
  fatal-stop propagation remain. Both owners must still request finishing.
  CBF-enabled dual filtering retains coupled fresh-state/skew checks; no
  asynchronous reuse or deadline increase was introduced. Timing diagnostics
  now select the saved reply for the corresponding lane.
- Added regressions for absent/delayed peer startup, actuator bounds, continued
  operation with an old unused peer snapshot, coordinated finishing, own-state
  age/model/deadline faults and shared stop. Added actual mock dual bringup with
  CBF disabled. Its safety status intentionally carries only the current arm's
  source timestamp; the unused entry is zero rather than a claimed paired state.
- Nine-package Release build and all 92 affected cases pass: 27 safety, 51
  controller and 14 bringup, with zero failures/errors. Corrected a new test
  fixture that initially applied 0.35 twice to already-scaled defaults; runtime
  caps were unchanged. Source formatting, scoped diff whitespace and local
  documentation link/anchor checks pass. Used isolated localhost ROS domain 137
  and robot-free fixtures; no physical arm/Hand commands or operator YAML edits.
  Build/test logs are under `/tmp/dual-actuator-*` in the ROS container.
  Physical dual-arm callback timing still requires operator verification.

### Persistent held-close demand independent of results (2026-10-22)

- The owner clarified that a held grip must keep the closing demand active
  independently of ordinary attempt success/failure. Refresh deadlines now
  start at close dispatch, not after a failed/unfinished completion. The
  existing `close_retry_interval_ms` setting is retained as the held-close
  refresh period. A running move remains uninterrupted; once it ends, an
  overdue refresh dispatches on the next worker check, without replaying missed
  periods or overlapping calls.
- Successful closure to zero no longer ends the held demand. Periodic live
  readback skips redundant moves at the bound, but closes again if the gap
  reopens during the same hold. Release, invalid/stale input and faults retain
  cancellation behavior. Transport/state/stop errors do not become automatic
  motion restarts. Position refreshes still do not command a specified force;
  this change does not add force-based `grasp()` control.
- Release build and all 50 affected cases pass: 37 Hand (24 core, 10 backend,
  three ROS process) and 13 bringup, with zero failures/errors. Regressions
  cover dispatch-based cadence, slow in-progress moves, healthy and failed
  completion, zero-width refresh, a reopened gap under the same hold and
  release cancellation. Source formatting, scoped diff whitespace and local
  documentation links/anchors pass. Operator values, Android artifacts and
  arm control remain unchanged. Used mocks/injected devices with ROS domain
  137 and localhost-only discovery, without physical FCI/Hand access. Logs:
  `/tmp/grip-persistent-*` in the ROS container. Sustained physical clamping
  force is not established by these tests.

### Repeated held-grip closing and command timing (2026-10-22)

- Added repeated position-closing attempts while a fresh grip remains held and
  the measured gap is above the lower width bound. The configurable
  `hand.close_retry_interval_ms` defaults to 200 ms after completion; zero
  disables retries. Attempts reuse width/speed and never overlap an active
  move or stop. Release, input faults and shutdown cancel pending retries;
  reaching the lower bound ends repetition. Existing fresh-release/repress and
  double-click opening behavior remains available.
- Unsuccessful close results and libfranka command exceptions may retry after
  valid state readback. The API does not identify contact, so these are not
  claimed as contact detections. Transport/protocol/unknown failures remain
  faults even if a later state read succeeds. Failed stop acknowledgement
  still invalidates calibration. Position retries do not command or measure
  clamping force, and do not automatically increase speed or force.
- Changed grip subscription depth from 20 to one latest queued sample. Release
  dispatch precedes request logging and bypasses press debounce. Completion,
  retry and watchdog polling defaults to 5 ms (previously 10 ms), configurable
  with `hand.worker_poll_ms`. Added request source-age and operation/stop
  completion durations to logs. These distinguish software delivery from the
  downstream path without claiming physical stopping latency or a fixed-rate
  Hand command stream. Documented timing checks and future sensor-based grip
  regulation separately from operator haptic cues.
- Nine-package Release build and all 78 affected cases pass: 36 Hand (23 core,
  10 backend, three ROS process), 29 Quest/configuration and 13 bringup, with
  zero failures/errors. Tests cover multiple blocked retries, release/fault/
  shutdown cancellation, lower-bound termination, disabled retries, new-press
  restart and driver error classification. Formatting, diff whitespace in the
  changed source/docs and local documentation links/anchors pass. Existing
  unrelated operator YAML values and Android artifacts were preserved. Used
  injected devices/mocks, ROS domain 137 and localhost-only discovery; no
  physical arm/Hand commands or APK rebuild. Logs: `/tmp/grip-repeat-*` in
  the ROS container. Physical contact forces and stopping delay remain for
  operator measurement.

### Repeatable Hand gestures and persistent pause holding (2026-10-22)

- Reproduced discarded grip presses arriving while a previous stop was still
  finishing and a stale zero-width completion snapshot rejecting a new close
  despite a nonzero live finger gap. Closing now reads current hardware width
  before each move, never commands opening, and does nothing if already at the
  configured lower bound. A new held press survives pending cancellation only
  while it remains fresh and held; release, faults and watchdog expiry cancel
  it. A new stable hold can interrupt automatic opening. Contact still does
  not trigger automatic retry, force increases or movement through an object.
- Grip buttons no longer depend on Cartesian pose tracking. Connection, focus,
  grip-action availability, clock/freshness/order/session gates remain required.
  Successful stop acknowledgement is retained if a subsequent redundant stop
  throws, but subsequent movement still waits for both workers to finish.
  Missing stop acknowledgement remains a fault requiring recalibration.
  Added an injectable device adapter to verify these hardware-driver paths
  without connecting to a physical Hand.
- Extended the startup hold latch to pauses: ordinary Quest input faults keep
  the parked joint reference and stiffness ramp. An input fault during engaged
  teleoperation captures measured joints once and selects local holding.
  Fresh release/re-engagement is still required. Safety fallback, invalid-state
  termination, homing priority and shutdown remain unchanged. Gains, actuator
  limits, payload settings and operator YAML values were not changed.
- Nine-package Release build succeeds. All 120 affected robot-free cases pass:
  31 Hand, 51 controller, 25 safety and 13 bringup, with zero failures/errors.
  The new launch readiness check verifies a valid trigger release consumed by
  the owner, since `position_hold` also applies before Quest is ready. Used
  ROS domain 137 and localhost-only discovery; no physical FCI/Hand commands.
  Logs are under `/tmp/pause-*` in the ROS container. Hardware stopping latency,
  finger contact behavior and physical holding performance need operator
  verification; mocks do not establish zero physical drift under all loads.

### Preserve startup joint hold before Quest engagement (2026-10-22)

- Reproduced a startup mode-selection defect: the production mapper publishes
  invalid/disarmed watchdog commands before Quest is ready. `fault()` revoked
  holding permission and reset the hold law, so an owner that initially selected
  joint holding switched to Cartesian damping with no static restoring torque.
  First-session/freshness faults could also reset the stiffness ramp or capture
  a later, displaced configuration. Two new deterministic regressions fail on
  the previous implementation, including a seven-joint synthetic displacement.
- Added a separate local startup-hold latch. Before the first accepted motion
  command or active homing, input validity/order/session/queue/freshness faults
  still revoke engagement permission but preserve the first local joint anchor
  and the progressing stiffness ramp. Fresh startup release messages also keep
  that anchor. Invalid state, safety fallback and shutdown exit this startup
  behavior; post-engagement input-fault damping remains unchanged. No gains,
  torque limits, payload settings or operator YAML values were changed.
- Added production mapper/mock-owner coverage for invalid tracking before
  trigger release/engagement and updated transition tests to distinguish local
  startup hold from post-engagement holding. Documented the startup exception
  and expected `reason: position_hold`, `motion_enabled: false` status.
- Release builds and all 88 affected robot-free cases pass: 50 controller,
  25 safety and 13 bringup, with zero failures/errors. Formatting and local
  documentation link/anchor checks pass. Safety and launch tests were explicitly
  enabled because their existing operator build caches had `BUILD_TESTING=OFF`.
  Development uses mock backends, ROS domain 137 and localhost-only discovery;
  no physical arm/Hand connection is made. Logs: `/tmp/startup-hold-*` in the
  ROS container. Robot-free tests establish mode selection and restoring torque,
  not zero physical motion under arbitrary load/model errors or saturation.

### Hold-to-close and double-click Hand control (2026-10-22)

- Replaced the grip grasp/open toggle with debounced hold-to-close using
  `move(close_width_m, speed_mps)`, immediate stop request on fresh release,
  and automatic move to `open_width_m` after two short clicks. All widths are
  full finger gaps. Default click duration is 50--200 ms and the first-release
  to second-release interval is at most 350 ms; these settings are configurable
  globally and per arm. Preserved the operator's speed/open widths and unrelated
  runtime settings. Legacy grasp force/width/tolerance keys remain accepted for
  old configs but no longer affect position closing.
- Kept transport-neutral gesture handling in `hand_core`; extracted blocking
  hardware/mock motion and cancellation into `hand_worker`. The stop worker
  runs independently of the blocking move and ROS executor. Stop requests are
  serialized against subsequent motion; repeated stop dispatch covers the
  race where cancellation reaches the device before a just-starting move.
  A failed/timed-out stop clears calibration and cannot launch deferred opening.
  Modelled partial travel in the mock rather than teleporting on completion.
- Invalid/stale/out-of-order/session/reference input and watchdog expiry stop
  ordinary movement and clear pending gestures; held input cannot resume it.
  Startup empty-hand calibration remains independent of Quest input. Shutdown
  requests cancellation and joins both workers. Closing at or below the lower
  bound cannot turn into opening; no automatic retry or force increase occurs.
- Updated runtime schema, standalone/single/dual YAML, operation/state reporting,
  README and architecture/runtime guides. Runtime configuration tests now use
  a temporary mock copy instead of assuming operator-edited dual IPs are empty.
  Initial tests exposed calibration being stopped by a stale pre-calibration
  input; fixed the policy and added dedicated coverage.
- Nine-package Release build passes. All 65 affected robot-free cases pass:
  23 Hand (16 core, 4 backend, 3 ROS process), 29 Quest/configuration and
  13 bringup; zero failures/errors. Used ROS domain 137 and localhost-only
  discovery. Checked C++/Python formatting, local documentation links/anchors
  and diff whitespace. No physical Hand/arm operation or APK rebuild occurred.
  Real input/network/device stopping latency and position-closing contact
  behavior remain operator validation; no exact force or instantaneous stop
  guarantee is claimed. Logs are under `/tmp/grip-*` in the ROS container.

### Maintenance cleanup, formatting and documentation (2026-10-16)

- With explicit owner approval, removed all existing raw episode bags,
  diagnostic captures, build/test logs, exported CSVs and MATLAB reports under
  `log/`: 802,822,317 file bytes, approximately 803 MB. Retained the ignored
  output root and reusable MATLAB source. Validation logs for this pass are in
  `/tmp`, so the repository output directory remains empty apart from
  `COLCON_IGNORE`. Removed two obsolete generated XML results for the deleted
  controller cores to prevent double-counting in future test summaries.
- Formatted maintained ROS C++/Python and scripts with clang-format 14 and
  Black 24.10.0; expanded MATLAB compound statements and normalized indentation
  with MISS_HIT 0.9.44. Added scoped `scripts/format_code.py`, check mode and
  pinned optional developer dependencies. Android, vendored dependencies,
  legacy Panda packages and existing Docker definitions were left outside the
  code cleanup. Operator YAML values were preserved.
- Cached constant Cartesian gain vectors at controller construction, flattened
  each exported message once, and reused request energy in MATLAB lag searches.
  Removed the unused lag sample counter. These are small efficiency/readability
  changes; no hardware timing improvement is claimed.
- Added a documentation index and architecture diagram. Reworked package and
  development guides; corrected old configuration paths, combined-launch flags,
  release behavior, backend assumptions, recording/export instructions and
  container build steps. Marked proposed asynchronous CBF work and historical
  stage records explicitly.
- Nine-package Release build passes. All 132 current robot-free cases pass:
  28 Quest, 7 recording/export, 25 safety, 47 controller/runtime, 12 Hand and
  13 bringup; zero failures/errors. Used ROS domain 137 and localhost-only
  discovery. Colcon's aggregate is 152 including test-wrapper suites; the
  individual case count is 132. Scoped formatting check and MATLAB syntax/style checks pass.
  MATLAB numeric/figure tests remain unexecuted by the agent because the
  existing separate batch process could not obtain a license in the preceding
  stage. No physical robot/Hand operation was performed.
  Shell syntax, all local documentation links/heading anchors, and single/dual
  PlotJuggler layout generation also pass; the single-arm generator reproduces
  the checked-in XML exactly.

### CBF scale configuration and explicit MATLAB legends (2026-10-16)

- Moved both actuator scales from the separate `safety` YAML section/ROS prefix
  to `cbf.torque_limit_scale` and `cbf.torque_rate_limit_scale` in standalone
  safety configuration and both combined runtime templates. Updated shared
  schema validation, launch routing, runtime parameters, tests and current
  documentation. The 0.35 defaults and CBF-independent enforcement are unchanged.
  Old YAML sections are rejected; old ROS scale overrides produce a migration
  error before control rather than silently dropping an intended stricter cap.
- Added visible, explicit legends to all 38 MATLAB plot panels, including every
  pose/velocity/derivative axis, each joint's four torque stages, single-signal
  diagnostics and Joint 1--7 in both residual plots. Residual joint colors are
  consistent. Empty requested trajectories retain an unavailable placeholder
  so their legend entry remains distinct from measured/reference curves.
- Extended the MATLAB report round-trip self-test to inspect the legends in
  all saved FIG panels and exercise unavailable torque/readback/request cases.
  Documented that existing figures must be regenerated. Clarified that grasp
  width is the full gap between opposing fingers, epsilon permits width
  deviation, and force is sent in the grasp request rather than in a second
  command after a success flag. No Hand operation behavior was changed.
- The nine-package Humble build and all 132 current robot-free cases pass:
  28 Quest, 7 recording/export, 25 safety, 47 controller/runtime, 12 Hand and
  13 bringup. Runs used ROS domain 137 and localhost-only discovery. The eight
  MATLAB source files pass syntax parsing. Attempted MATLAB self-tests using a
  separate batch process in the existing R2024b container; license checkout
  failed with error -1 before executing tests. Numeric/render/export validation
  remains for the licensed browser session; its authentication was not modified.
  No physical robot/Hand command was issued, and operator values/generated
  Android files, existing Docker definitions and legacy packages were preserved.

### Editable 35%-of-FCI actuator limits and Hand grasp diagnostics (2026-10-16)

- Added `safety.torque_limit_scale` and `safety.torque_rate_limit_scale`, both
  defaulting to 0.35, in standalone safety configuration and both combined
  runtime templates. These scale the published Panda FCI ceilings once:
  `[30.45,30.45,30.45,30.45,4.2,4.2,4.2]` Nm and `350` Nm/s. Additional controller
  absolute caps remain minima, including the existing stricter shared dual-arm
  cap policy. The scales are independent of CBF enablement and validated in
  `(0,1]` before startup; changes require a launch restart.
- Centralized the defaults/helpers in `safety_filter/actuator_limits.hpp` and
  applied them to filter defaults, runtime setup, manager preflight and reported
  actuator limits. Startup logs show the effective bounds. The same magnitude/
  rate intersections constrain Cartesian, holding, homing, damping and fallback
  torque, including actuator-only clipping with CBF disabled. Gravity compensation,
  Hand grasp force, freshness gates and geometric/kinematic CBF thresholds are
  not rescaled. No manufacturer protection settings were modified.
- Added Hand startup/request/result logs showing requested width, force, speed,
  epsilon tolerances, completion width, success and `is_grasped`. Force is still
  passed unchanged to `Gripper::grasp`; there is no force readback or continuous
  slip detector. No grasp parameters, retry behavior or operation semantics were
  changed. Added setup/diagnostic guidance for thin cables and parameter reloads.
- Read only the existing `quest_20261016T142129Z_2bfae179` bag's Hand events:
  one grasp failed at 17.509 s with measured gap 9.8244 mm; a later grasp
  completed at 34.480 s with gap 8.1085 mm. The successful gap is incompatible
  with a 1 mm target and the narrow tolerances in the edited YAML, so that older
  recording does not verify that the current settings were loaded. The bag
  contains no issued-force field, and no physical cause of slipping is inferred.
- Added regression coverage for default/custom scales, invalid scale rejection,
  positive/negative magnitude and rate bounds with CBF enabled/disabled, actual
  single/dual YAML routing, full-rate bound telemetry, stricter user caps and
  propagation/logging of a 30 N Hand request.
- The nine-package Humble Docker build passed. All 131 current robot-free cases
  pass: 28 Quest, 7 recording/export, 25 safety, 46 controller/runtime, 12 Hand
  and 13 bringup. Tests ran in ROS domain 137 with localhost-only discovery.
  Two coordinator assertions initially compared the computed per-cycle slew
  directly to the decimal literal 0.35; they now compute the same 350 Nm/s *
  1 ms bound, avoiding a floating-point representation mismatch. The eight
  affected coordinator cases passed on rerun; no production limit was relaxed.
- Syntax/configuration checks and `git diff --check` pass. No physical robot or
  Hand commands were issued. Existing robot Docker definitions, legacy packages,
  Android generated changes and operator Hand values were preserved, including
  the epsilon edits made during implementation. Physical holding/grasp behavior
  under these new arm limits remains unverified.

### Joint position holding after trigger release (2026-10-16)

- Replaced normal released-trigger damping with `PositionHoldController`, a
  separate seven-joint impedance law using the common torque interface. The
  manager captures measured joints once on entry, sets zero desired joint
  velocity and preserves that reference across repeated release messages.
  This restores arm posture as well as the end-effector location under finite
  stiffness. It never pursues an old Quest target or re-anchors every cycle.
- Startup captures the initial configuration. Re-engagement selects the
  unchanged Cartesian law and the mapper's existing measured-pose anchors.
  Homing retains exclusive priority and returns to measured joint hold when
  input is fresh/released. Input faults, stale commands, safety fallback and
  shutdown retain damping; homing arbitration cannot erase the fault gate.
- Added configurable seven-element `hold_stiffness` and `hold_damping`, plus
  `hold_stiffness_ramp_seconds` in standalone and single/dual runtime configs,
  including per-arm override routing. Default stiffness is
  `[600,600,600,600,250,150,75]` Nm/rad, damping
  `[50,50,50,50,20,15,10]` Nm s/rad and the smooth stiffness ramp is 0.25 s.
  Physical gain/transition performance remains unverified and requires
  commissioning; actuator/CBF bounds remain authoritative.
- Status/tracking report `reason: position_hold` with Quest
  `motion_enabled=false`, preserving ROS message schemas and old recording
  compatibility. MATLAB's diagnostic mode plot includes holding; pose/velocity
  and torque still record while active teleoperation metrics exclude holds.
  The dual-arm filter retains parked-arm priority for a holding arm.
- Added robot-free checks for one-time capture, displacement of the redundant
  seventh joint, release/re-engagement ownership, stiffness ramp/reset, invalid
  gains/references, homing return and fault-triggered reference removal. Updated
  the actual mapper/mock process and single/dual launch acceptance checks.
- The nine-package Humble Docker build passed. All 127 current robot-free cases
  pass: 27 Quest, 7 recording/export, 23 safety, 45 controller/runtime, 12 Hand
  and 13 bringup. Tests used isolated ROS domain 137 and localhost-only discovery.
  Corrected a launch-test readiness assumption: startup hold alone does not
  prove an accepted trigger release; readiness now also requires a consumed
  source command. The affected four launch cases passed on rerun.
- MATLAB source syntax parsing, Python/configuration checks and `git diff
  --check` pass. No physical FCI/device operation occurred. Existing robot
  Docker definitions, legacy Panda packages, Android generated changes and
  operator configuration values were preserved; only the requested hold keys
  were added to the editable runtime YAML. Physical hold gains/transitions and
  the updated MATLAB mode plot have not been exercised by this agent.

### Trajectory monitoring and licensed MATLAB reports (2026-10-16)

- Added coherent per-arm `TrackingSample` messages: mapped requested,
  controller-reference and measured pose/twist; four torque stages; effective
  gains, modes, safety intervention, clock identities and sample-loss counters.
  Requested twist is captured before mapper low-pass filtering; measured twist
  uses the same-state `J*dq`. No Android change or raw IMU claim is involved.
- The torque callback hands fixed-size samples to a bounded SPSC queue. A
  separate non-realtime consumer publishes full-rate `tracking_samples` for
  recording and latest `tracking_status` for the 100 Hz live dashboard. Invalid
  Cartesian references and unavailable mock torque readbacks are NaN. Hardware
  desired/measured torque retain their distinct gravity conventions and incoming
  state timestamps; previous outputs are alignment candidates, not acknowledgements.
- Extended the default recording profile and added a tested rosbag2-to-CSV
  exporter with integer-clock preservation, per-arm files and a schema manifest.
  Added generated single/dual-arm PlotJuggler layouts for pose, velocity, all
  seven joints' four torque stages and diagnostic context.
- Added base-MATLAB experiment reports with vector PDF, 600 DPI PNG and editable
  figures; time-weighted tracking metrics; descriptive velocity-lag analysis;
  and supporting local-fit acceleration/jerk estimates. Analysis separates
  engagement/session/reference changes and gaps, and excludes idle, homing and
  fallback from tracking metrics. It does not alter control/filter settings.
- Added a separate optional official MATLAB R2024b Compose environment, preserving
  existing robot Docker definitions. The [monitoring guide](TRAJECTORY_MONITORING.md)
  documents browser first use, account/network licensing, batch execution,
  release entitlement and handover after personal license expiration. No ROS or
  Signal Processing Toolbox is required; credentials are never packaged.
- The Humble Docker build passed. All 121 current robot-free cases pass: 27 Quest,
  7 recording/export, 23 safety, 39 controller/runtime, 12 Hand and 13 bringup.
  Coverage includes real mapper/mock-controller monitoring, full/live streams,
  torque conventions, inactive validity, clock precision and two-arm bag export.
  Runs used isolated ROS domain 137 and localhost-only discovery. An initial
  test run overlapped the schema rebuild; affected failures passed after the
  rebuild completed. The editable runtime YAML's hardware choice was preserved.
- MATLAB's eight source/test files pass syntax parsing; Compose configuration,
  Python/shell syntax and generated layout structure pass checks. MATLAB numeric
  tests and figure export have **not** run here because no activated MATLAB
  runtime/license is available. Seven MATLAB self-tests, including synthetic CSV
  analysis and headless figure export, are provided for the documented licensed
  first run. The PlotJuggler GUI and physical robot behavior remain untested.
  Existing operator YAML and Android generated-file changes remain untouched.

### Shared controller runtime and torque-law architecture (2026-10-16)

- Moved ROS/configuration, threading, telemetry, model/backend adapters and
  permanent FCI ownership to `controller_runtime.cpp`. Retained the existing
  executable, ROS node names, launch interfaces and YAML keys for compatibility.
- Replaced `ControlCore` with `ControllerManager`: command validity/freshness,
  session/order and release/engagement gates, state/actuator-interval checks,
  lifecycle reset and exclusive nominal-source selection now live in
  `controller_manager.cpp`. Homing/controlled stopping, armed teleoperation and
  disarmed damping select exactly one torque computation per cycle.
- Added a fixed-size `TorqueController` interface with typed Cartesian/joint
  references, seven nominal joint torques, lifecycle reset and explicit faults.
  Separate Cartesian impedance, damping and homing implementations preserve the
  baseline formulas. Homing advances references independently of torque
  computation; the synchronized `joint_motion_generator` remains unchanged.
- Removed the discarded intermediate torque clamp. Every nominal law still
  passes through the existing shared safety coordinator, which owns final
  actuator bounds with CBF enabled or disabled. Retained `tau_J_d` anchors,
  controlled homing cancellation, fresh release/press after faults/home and
  one permanent libfranka owner per arm. Torque limit telemetry uses the final
  safety layer's shared limits.
- One Docker build of `safety_filter`, `teleop_controllers` and `teleop_bringup`
  passed. All 72 individual robot-free cases passed: 23 safety/coordinator,
  36 controller/law/homing/runtime and 13 bringup cases. Coverage includes
  injected-law selection/reset/faults, homing priority and cancellation,
  actuator-only CBF-disabled execution, mapper watchdog/re-engagement,
  single-left/right and dual launches, recording and standalone compatibility.
  Tests ran in isolated ROS domain 137 with localhost-only discovery.
- Updated README and package architecture/extension documentation. No physical
  FCI/device operation, Android rebuild, Docker change or legacy Panda edit.
  Existing operator changes in the single-arm YAML and Android generated files
  were preserved.

### Mode-specific template cleanup (2026-10-13)

- Dual template omits `single_controller`; its left/right assignments remain
  explicit under `arms`. Single template keeps the pipeline hand selector and
  omits the redundant per-arm selector and inactive inter-arm clearance.
- Both templates omit fixed false mapper activation/confirmation flags. Single
  omits the empty per-arm override block, with a note for optional surveyed
  external-world transforms; dual retains both required transform placeholders.
  Reformatted dual arrays inline for consistent editable templates.
- Five focused configuration checks pass, including left selection from the
  reduced single template, dual assignments, missing physical geometry and
  unchanged confirmation guards. No builds or hardware operations were needed.

### Mandatory runtime selection and two overall arguments (2026-10-13)

- Operator requests mandatory `runtime_config` with no default file and only
  `operator_confirm` as the second overall argument. Removed every overall CLI
  feature/mode/recording override; values come exclusively from the chosen YAML.
  Standalone launch interfaces are unchanged.
- Replaced the generic template with `config/single_panda.yaml` and
  `config/dual_panda.yaml`. Both are complete mock-default templates; dual has
  distinct left/right namespaces/frames and deliberately unset hardware IPs/
  surveyed transforms. Hardware cannot use the mock's internal spacing implicitly.
- `operator_confirm` defaults false and is required true only for hardware.
  It authorizes real connections and enabled Hand startup calibration; it does
  not arm the trigger or certify physical settings. Updated README, package
  reference, mock guide and policy with explicit file selection.
- One two-package build passed; all 40 affected robot-free cases pass (27 Quest,
  13 bringup). Actual launch help exposes exactly the two arguments; omission of
  runtime_config fails, obsolete overrides are rejected, YAML-only single/dual
  execution and standalone compatibility pass. No Docker/legacy/C++ controller
  edits, Android rebuild or hardware/USB operation occurred.

### Unified overall configuration (2026-10-13)

- Implemented the operator's requested separation: standalone launches retain
  package-local defaults, while combined bringup defaults to one complete
  `teleop_bringup/config/teleoperation_runtime.yaml`. It contains feature switches,
  arm identities/mapping, bridge settings, controller/Hand/CBF settings, per-arm
  overrides and recording profile/Menu settings. CLI switch overrides remain
  optional; hardware operator confirmation remains explicit and launch-only.
- Removed combined `arms_config`, `recording_config`, `controller_config`,
  `gripper_config` and `safety_config` file arguments. Strict readers select
  their portions of the unified document while preserving standalone schemas.
  Namespaced bridge input derivation, all exposed bridge settings and configurable
  Menu debounce reach their nodes. Empty optional obstacle arrays are omitted
  from ROS parameter declarations when unused.
- README now edits the unified source YAML directly, with no workspace-level
  config copies. Updated package/runtime guides and policy. Installed config is
  symlinked to source; `runtime_config` can select a different complete YAML.
- One focused Docker build passed for the four changed launch/config packages.
  All 43 affected robot-free cases pass (26 Quest, 5 recording, 12 bringup),
  including YAML-only feature selection and node parameter values, removed-file
  rejection, hardware confirmation rejection, standalone defaults, mock
  single/dual control and bags. One test-only ROS API compatibility fix was
  checked by rerunning only launch-options acceptance. Verified installed
  argument help/default path. No Docker/legacy Panda/C++ controller changes,
  Android rebuild, USB/device operation or physical FCI operation occurred.

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
