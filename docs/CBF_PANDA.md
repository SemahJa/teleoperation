# Panda adaptation of the thesis_ws CBF-QP

Current status: the filter is connected to the actual single/dual arm executable.
Robot-free core/coordinator/process tests pass. Hardware callbacks collect one
RobotState's mass, coriolis, poses and Jacobians; physical execution and timing
commissioning have not been performed.

## Formulation and adaptation

The filter retains the thesis_ws joint/collision exponential CBF formulation
and torque-tracking QP: minimize weighted squared deviation from nominal
torque, subject to joint position, velocity, acceleration, actuator magnitude/
rate, table, same-arm, inter-arm and optional spherical obstacle constraints.
Franka's torque interface compensates gravity; the modeled acceleration is
`M^-1 (tau - coriolis)`. Cartesian impedance remains the agreed filtered-error
law, and the separate joint homing law supplies another nominal torque source.
Both pass through the same filter before torque is returned.

- Uses FER/Panda joint/velocity/acceleration/torque ceilings, not FR3 values.
  Initial `7 rad/s²`, torque `[80,80,80,80,10,10,10] Nm` and `900 Nm/s` settings
  are software reference bounds, not physical commissioning approval.
- Rate constraints intersect magnitude limits using each arm's actual
  `tau_J_d`, rather than the filter's previous requested output. Each slew
  interval is capped at one millisecond even after a delayed callback.
- Single-arm masking disables inactive-arm barriers and forces its dummy
  torques to zero. Dual mode retains coupled collision rows; independent
  per-arm filters cannot substitute for that coordinator.
- Geometry transcribes the pinned Panda URDF coarse cylinders/spheres as
  capsules, including the Panda flange geometry and hand mounting yaw. The
  same-arm exclusion list matches the pinned Panda arm/hand SRDF. Hand logical
  link 9 remains distinct from flange link 8. Clearance padding is added once.
- Invalid models, empty actuator intervals and nonfinite active rows are
  reported as fatal input faults. Invalid rows are never silently disabled.
  Solver failure returns a bounded fallback and explicit constraint violation;
  fallback is not presented as geometrically safe.
- Legacy hardware end-effector rotations may include float-rounded `0.7071`
  coefficients. The hardware adapter orthonormalizes only near-rigid measured
  end-effector/model end-effector poses (finite homogeneous matrix, positive
  determinant, determinant/orthogonality errors and correction bounded by
  `1e-4`). Translation and Jacobians are preserved; joint/flange poses remain
  raw. The strict filter rotation checks remain `1e-8`. Reflections, degenerate
  and substantially distorted transforms still stop control. This follows the
  pinned libfranka 0.9.1+ orthonormalized-rotation convention; it does not change
  Desk's configured tool transform or load.
- Fixed solver dimensions and numeric storage are allocated during configure.
  The CSC matrix includes only structurally possible coefficients: one per
  actuator row, seven for same-arm dynamics/geometry, and fourteen for coupled
  inter-arm rows. Configured inactive-arm/disabled barriers have no solver
  coefficients and are omitted from the solver's fixed row mapping. Logical
  rows and their original bounds remain available for
  independent feasibility checking; distance-activated rows keep their full
  possible support. The pattern never changes in the realtime path.
  Runtime results use constant status strings, fixed model arrays and no heap
  allocation. Tests instrument both C++ allocation and solver malloc/calloc/
  realloc paths. OSQP signal handling/printing/MKL loading are disabled.

Collision rows use closest capsule points, interpolated point Jacobians,
finite-difference `Jdot*dq` history and curvature terms. Coincident points
retain the previous separation direction or choose a controllable axis.
No slack silently relaxes actuator/collision constraints. A fixed-iteration
solver candidate is accepted only after an independent check of all current
constraints within configured numerical tolerance. Parked-arm weighting
changes the soft torque objective; it does not relax collision rows.

After constructing and validating every current constraint, the filter first
checks the nominal torque (with inactive-arm torques forced to zero). If its
maximum violation is exactly zero, it is the exact optimum of the positive
weighted squared-distance objective. Report `feasible_nominal`, zero solver
iterations and zero violation, and seed OSQP's primal warm start. This path
still validates the model, actuator bounds and all collision/joint rows and
remains subject to the same runtime deadlines. Even violations smaller than
the solver's feasibility tolerance require the normal iterative path.

## Runtime ownership, handoff and failure behavior

The integrated launch defaults to `cbf_enabled:=true`. The explicitly requested
diagnostic option `cbf_enabled:=false` sets `cbf.enabled=false`; the coordinator
then uses the filter's actuator-bounds-only branch, without joint/collision
barriers or QP iterations. All model/actuator validity, exact-cycle handoff,
age/skew/deadline checks and consumer engagement gates remain. It reports
`disabled_actuator_bounds_only` and a startup warning. This removes software
collision and joint CBF protection; it is not normal commissioning acceptance
and does not alter the robot's own protections or start physical control
automatically. Runtime YAML cannot silently change this launch selection.

One permanent owner per arm computes either Cartesian or homing nominal torque.
A single coordinator owns the 14-variable QP and consumes fixed-size model
snapshots through independent SPSC latest-snapshot lanes. Each owner waits only
for a reply carrying its exact cycle and model timestamp; earlier-cycle results
are never applied. The coordinator requires a new snapshot from each active arm
for every coupled solve. Single mode masks the inactive arm completely.

Hardware provisional limits: model age 2 ms, cross-arm timestamp skew 1 ms,
callback/model/solve/result deadline 0.9 ms, and filter computation 0.8 ms.
These are enforced software limits, not measured timing guarantees. Snapshot
timestamps are local callback-entry times; they do not establish synchronized
physical acquisition across robot clocks. Bound network/receive delay and its
variation during commissioning. The hardware
coordinator requires maximum SCHED_FIFO priority and memory locking; libfranka
retains its own enforced realtime configuration. A bounded 25 microsecond wait
lets the equally prioritized coordinator run even on a shared CPU. Thread
placement, wakeup jitter and worst-case time must be measured on the deployment
PC. No mutex or ROS work enters the torque/model/coordinator path.

On a result/filter deadline stop, postmortem `CBF timing` lines report each
owner's final cycle: callback-to-submit preparation, submit-to-owner-return wait,
total elapsed time and the unchanged result deadline. A matching coordinator
result adds submit-to-coordinator delay, coordinator work, filter duration and
solver status. Missing or different-cycle results are marked unavailable (`-1`
for times). Coordinator completion can occur after the owner has timed out;
postmortem availability does not mean the torque was accepted. All formatting
and logging occurs after the owner and coordinator threads have joined.
Timing stops also report solver duration/iteration count and export a matching
consumed model to `/tmp/franka-cbf-model-arm-N.txt` in the controller's container.
The `CBF_MODEL_V1` text format contains arm index, cycle, period, then q, dq,
coriolis, previous desired torque, nominal torque, mass, and each frame's pose
and Jacobian. Matrices are column-major at 17-digit precision. Export is after
all realtime threads have joined and never clears the stop. Each subsequent
timing failure replaces that arm's file; copy it before another trial.

The non-RT mock uses age/result limits of 100 ms, skew/filter limits of 50 ms.
These fixture limits never apply to hardware. Its collision model follows Panda
URDF kinematics, while inertia is identity, coriolis zero, joint state fixed and
Cartesian telemetry synthetic; it does not simulate physics or certify motion.

An invalid model/transform/actuator interval, state age/skew fault, missing or
late matching result, or filter deadline failure latches a shared stop. Solver
infeasibility returns the explicitly reported bounded fallback, disables
teleoperation and cancels homing; 20 consecutive failed solves by default latch
a stop. A successful solve resets that consecutive counter, but never resets a
latched stop. Owners terminate through libfranka exceptions without automatic
reconnect, recovery or restart. Fallback has no geometric guarantee.

Normal shutdown uses filtered measured-pose damping and controlled homing
cancellation. Both owners request finishing; the coordinator grants finishing
in the same paired reply, preventing one owner from disappearing while its peer
still awaits a coupled solve. `/ARM/cbf_status` reports source timestamps, cycle,
skew, solver/filter time, failure count, constraints, clearances, intervention,
filtered torque and stop reason. Fatal stop reasons also appear in process logs.

Tests cover complete Panda model fixtures, independent command activity, paired
results and finishing, stale/skew/deadline/model faults, repeated infeasibility,
actual dual process filtering and actual process termination. Collision-coupled
torque signs are exercised in the filter's controlled geometry test. None of
these robot-free checks establishes physical 1 kHz compatibility.

## Physical limitations and commissioning boundary

Coarse geometry is not a mesh collision certificate. Finger motion, altered
fingertips/tools, cables, the measured table plane and both base transforms
require physical survey and conservative padding validation. Capsules must be
compared with detailed geometry over the joint domain and all gripper widths.
Closest-point changes and sampled `Jdot` are approximation limitations.

No formal singularity CBF is implemented in thesis_ws or this adaptation.
Quaternion handling and a coarse target envelope do not provide singularity,
reachability or collision guarantees. A formal dexterity barrier needs valid
gradients/Hessians and joint-domain/numerical/feasibility validation before
hardware use; it must not be advertised as an existing protection.

Before deployment: verify firmware/tool/payload and torque convention, measure
worst-case 1 kHz update/solver time and jitter with all collision classes,
bound dual-arm state skew, check zero unexpected infeasibility, test the
independent stop/recovery path, validate home clearance, and perform staged
physical acceptance with an observer-accessible stop. The bounded fallback
has no geometric guarantee. These commissioning operations are outside the
current coding task.

Sources: local thesis_ws `quest_fr3_controllers/{cbf_safety_filter,CBF_DESIGN}`,
the pinned legacy `franka_description`/`franka_moveit_config` geometry and SRDF,
and OSQP 0.6.3. Source copyright/license headers are retained. Packaged licenses
and dependency notices are in `src/teleop_controllers/THIRD_PARTY_LICENSES`.
