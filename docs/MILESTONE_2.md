# Milestone 2: asynchronous CBF and teleoperation performance

This is a design plan, not a description of available runtime switches.
As of 2026-10-16, the controller-law separation, release position holding,
pose/velocity/four-torque telemetry, PlotJuggler layout and MATLAB report tools
are implemented. Asynchronous CBF result reuse, pose interpolation and
controller-comparison experiments remain pending. The owner still waits for
the current cycle's safety result. Use [PACKAGE_STRUCTURE.md](PACKAGE_STRUCTURE.md)
for the implemented architecture and [TRAJECTORY_MONITORING.md](TRAJECTORY_MONITORING.md)
for available analysis. Earlier requirements to preserve trial logs below are
historical; the owner authorized removing existing logs and recordings during
the 2026-10-16 maintenance cleanup.

## Milestone 1 acceptance

The operator accepted Milestone 1 on 2026-10-12 after successful single-right-arm
hardware teleoperation with `cbf_enabled:=false`. The existing Quest transport,
mapping and nominal impedance controller form the working baseline.

Keep the synchronous CBF faults as evidence: intermittent 100-iteration solves,
filter/result deadline stops, and CBF fallback disarming the owner prevented
sustained following. This acceptance does not assert that CBF-enabled hardware,
dual-arm collision handling, Hand, homing or every commissioning checklist item
passed. Those records retain their individual status.

The operator's next requested work is:

1. Decouple CBF-QP computation from the 1 kHz torque callback.
2. Improve smoothness and tracking, implement controller alternatives, and
   compare target and measured motion quantitatively.

The first asynchronous implementation is **single-arm experimental operation**,
as selected by the operator. Keep existing dual-arm behavior unchanged. New
physical trials remain operator actions; development and automated tests are
robot-free. Preserve Docker, dependencies, baseline configurations and logs.

## Task 1: single-arm asynchronous CBF-QP

### Required architecture

The present coordinator already runs on another thread, but the owner waits for
the exact current-cycle result. It is therefore synchronous from the torque
callback's perspective. Remove that wait in an explicit experimental execution
mode, rather than simply increasing its timeout.

- **1 kHz owner:** acquire current state, compute current nominal torque,
  publish a latest-only immutable worker request, consume available results
  without waiting, perform bounded current-state acceptance/actuator checks,
  and return torque. No QP iteration, allocation, ROS call or blocking wait.
- **Independent worker:** take the latest snapshot, construct constraints and
  solve the QP with a separately bounded workload. Intermediate pending requests
  may be superseded; there is no unbounded request backlog. Derivative/history
  calculations must use the actual interval between consumed model timestamps,
  rather than pretending every worker snapshot is one millisecond apart.
- **Result handoff:** fixed-size, preallocated publication of source model time,
  source sequence, command/session/reference identity, controller mode, solve
  start/end, feasibility and the result's validity information. Compute age
  from source acquisition time, not only solver completion time.
- **Acceptance:** an older result must not override a trigger release,
  tracking/session/reference fault, homing transition or shutdown. Reject
  out-of-order results, invalid/nonfinite results and results outside their
  permitted age/state envelope. Recheck actuator magnitude and slew against
  current `tau_J_d` on every callback, including reused results.
- **Startup and degradation:** define bounded behavior before the first valid
  result and on worker starvation/failure. No automatic motion engagement,
  unbounded reuse or claim that damping alone preserves geometric clearance.
  Persistent inability to produce an acceptable command must stop/disarm.

An asynchronous result is not automatically safe for the current state. A
timestamp limit alone cannot certify an old torque. The implementation must
choose and test one of the following before enabling physical CBF claims:

1. Rebuild current single-arm constraints without solving and accept a cached
   candidate only if it satisfies those constraints and current actuator bounds.
   Benchmark that verifier separately within the callback budget. When nominal
   torque satisfies all current constraints, retain the exact-optimum shortcut.
2. Publish a conservative constraint certificate valid over a bounded state and
   time envelope, with sampled-data/delay margins, and check the current state
   against that envelope in the owner.

**Initial implementation preference:** current-state verification, because the
existing constraint builder and independent residual checker can be reused and
measured. If its worst-case cost exceeds the callback budget, move to certified
constraint envelopes; do not silently replace verification with raw torque hold.
Both retain the limitations of the underlying model/CBF approximation.

Sampled-data CBF literature explicitly addresses between-update evolution and
delays; it does not justify indefinite reuse of a continuous-time solution:
[Breeden, Garg and Panagou](https://arxiv.org/abs/2103.03677),
[Singletary, Chen and Ames](https://arxiv.org/abs/2005.06418).

### Controls and telemetry

- Keep `cbf_enabled:=false` for the accepted baseline and existing synchronous
  mode for comparison. Add an explicit execution selector for asynchronous
  single-arm experiments; reject unsupported asynchronous dual mode.
- Separate callback budget, worker solve budget, worker update interval and
  accepted result age. Determine age/update defaults from replay/benchmarks and
  the verifier design; a slower QP is still bounded, not unlimited.
- Record worker/source/result age, execution time, accepted/rejected result
  counts and reasons, superseded requests, constraint violation, intervention,
  fallback, and callback execution/jitter. Retain the original source timestamp
  when reusing a result.
- Preserve exclusive FCI ownership, explicit hardware confirmation, the input
  gates, workspace envelope and torque guards in every execution mode.

### Robot-free acceptance

Inject solves longer than 1 ms and confirm callback progress does not wait for
them. Exercise startup with no result, delayed/stale/out-of-order results,
changing geometry, trigger release during a solve, reference/session changes,
changing nominal commands, current torque anchors and worker failure. Verify
the current-state checker rejects a formerly feasible candidate after the state
changes. Measure allocations, backlog/drop behavior, callback worst cases and
worker timing. Use the recorded failing Panda models as regression inputs; static
replay alone is insufficient to establish temporal behavior or physical safety.

## Task 2: controller performance and comparison

### Instrumentation first

Record these distinct signals on a common timeline and in the same robot frame:

1. Raw operator pose and mapped robot target.
2. Effective position/orientation/velocity reference actually consumed after
   smoothing, engagement and safety decisions.
3. Measured robot position/orientation, joint position/velocity, nominal torque
   and commanded torque.
4. Controller structure and full settings; engagement, saturation, CBF
   execution mode, result age, intervention, fallback and dropped trace samples.

Use callback model timestamps for robot/reference pairs; ROS publication times
alone may hide telemetry delay. Reference and command transport timestamps must
remain distinguishable. Record outside the torque callback through bounded
handoff and report trace losses. The current fixed-kinematics mock is useful for
logic tests, but it cannot validate physical tracking dynamics or controller
stability.

### Controller candidates

| Candidate | Purpose | Additional requirements |
| --- | --- | --- |
| Existing filtered-error impedance | Preserve the successful baseline exactly | Current gains, mapping and nominal law |
| Inertia-aware Cartesian impedance | Compare tracking and damping with task-space dynamics compensation | Actual mass/Jacobian, regularized task-space inertia, singularity handling and bounded commands |
| Resolved-rate reference with joint torque tracking | Compare a velocity-based structure and explicit reference limits | Damped inverse kinematics, joint-reference limits, null-space policy and continuity on engagement |

Do not label an algebraic rewrite as a new controller. The current law
`tau = -Jᵀ G (v - v_ref + Lambda * error) + coriolis` already equals Cartesian PD
with `Kp = G * Lambda` and `Kd = G`. Document each candidate's actual equation,
units, compensation terms and limits. Keep gravity compensation consistent with
Franka's torque interface; do not add it a second time.

Add a shared reference generator so pose sampling and feedforward do not create
unbounded target velocity/acceleration. Treat translation and rotation
consistently; use quaternion/SO(3) orientation errors, not Euler-angle subtraction.
Reset reference state on release/fault/session change, anchor engagement at the
measured pose, and avoid a lagging trajectory continuing after disarm. Compare
baseline direct mapping with reference shaping independently of controller
structure so the source of improvement is identifiable.

### Plots and metrics

Produce an offline comparison command that reads recorded bags/traces and emits
CSV, a machine-readable metrics report and standalone PNG/PDF plots:

- Overlay mapped target, effective reference and measured x/y/z versus time.
- Plot translation error norm and quaternion geodesic orientation error.
- Plot target/measured velocity, acceleration and suitably filtered jerk;
  report filter settings because differentiation amplifies measurement noise.
- Plot nominal/commanded torque, saturation and CBF intervention/result age.
- Mark engagement, invalid input, CBF fallback and disarmed intervals.
- Compare RMS, median, 95th-percentile and maximum translation/orientation
  errors, phase/lag where the trajectory supports estimation, overshoot/settling,
  smoothness, torque saturation and time spent disarmed or in fallback.

Segment by session/reference and valid engagement. Do not interpolate across
disarm, recenter or missing-data gaps, or score long stationary/disarmed periods
as good tracking. Raw-target versus effective-reference errors must both be
reported: smoothing can reduce jerk by adding latency rather than improving the
controller itself. Report timing/drop statistics and exclude runs with inadequate
data from quantitative comparisons.

### Comparison protocol

Use repeatable slow ramps, sinusoids and smooth point-to-point references before
freehand operator trials. Keep starting pose, payload/tool, axis mapping,
translation/rotation scale, trace rate, reference generator, CBF mode and torque
limits fixed across controller comparisons. Match stiffness/damping objectives
where meaningful and record every parameter and repository revision.

First evaluate with deterministic reference replay and an appropriate dynamics
simulation or recorded-model tests. Then the operator can perform the same
reviewed physical motions for each candidate. Gain tuning, singularity handling
and physical results must not be inferred from identity-inertia fixture tests.

## Implementation order and completion criteria

1. Preserve Milestone 1 evidence; introduce effective-reference and timing
   telemetry plus trace/bag plotting and metrics. Establish baseline plots.
2. Extract constraint verification from iterative solving; build and test the
   single-arm nonblocking worker/owner contract with delayed-result regressions.
3. Validate asynchronous age/state policies and worst-case callback timing in
   robot-free tests. Keep execution modes explicit and independently comparable.
4. Add shared reference shaping and the alternative controller structures, one
   at a time; test continuity, singularities, numerical validity and torque guards.
5. Generate repeatable comparisons and record operator hardware outcomes.

Task 1 is complete only when a QP exceeding one callback period cannot stall the
owner, stale/invalid results cannot authorize motion, and validation/fallback
behavior and measured timing are documented. Task 2 is complete only when the
candidate controllers, reproducible plots/metrics and a comparison report exist.
Improved physical smoothness/tracking is a measured outcome, not a code-completion
claim. These two tasks are currently planned, not implemented or validated.
