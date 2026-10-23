# Trajectory monitoring and MATLAB experiment reports

Use PlotJuggler for live pose/velocity feedback and MATLAB for repeatable
experiment summaries and vector/high-resolution figures. MATLAB runs in a
separate Docker container; no host MATLAB installation, ROS Toolbox or Signal
Processing Toolbox is required. The Android/OpenXR application is unchanged.

PlotJuggler also supports offline recordings through its ROS data-loader
plugins. Opening a live dashboard does not record an experiment. To generate
MATLAB reports later, record the tracking topics into a bag first, then export
the finalized bag to CSV. Existing trial bags and reports were removed during
the requested cleanup; record a new episode before following the export steps.

## Data and clocks

Each arm publishes two `quest_teleop_interfaces/TrackingSample` topics:

| Topic | Use |
| --- | --- |
| `<arm namespace>/tracking_status` | Latest coherent sample at up to 100 Hz, for live plots |
| `<arm namespace>/tracking_samples` | Every successful control cycle, normally about 1 kHz, for recording |

The torque callback copies fixed-size snapshots into a 1024-slot SPSC queue.
A separate thread converts/publishes ROS messages; a slow monitor never blocks
control. Overflow drops monitoring samples and increments `monitor_queue_drops`.
The 100 Hz status publishers also run separately from Quest input and the
mapper's measured-pose feedback. That pose feedback uses best-effort, depth-one
QoS; recording and analysis still use the full-cycle `tracking_samples` stream.
The default rosbag profile includes `tracking_samples`. Gaps in recorded cycle
numbers expose missing samples; explicitly exporting the 100 Hz topic naturally
omits intermediate control cycles. Neither ROS nor a bag is a lossless guarantee.

Each sample includes:

- Requested pose: mapped Quest pose in the robot base frame, before reference
  shaping. Optional pose filtering/interpolation runs in the mapper; the
  current bringup templates select no pose shaping.
- Requested twist: mapped native/pose-derived blend **before** the mapper's
  low-pass filter. Reference twist: the filtered velocity supplied to the law.
  These are kinematic signals, not raw accelerometer/gyroscope readings.
- Reference pose and measured pose; measured Cartesian twist computed as `J*dq`
  from the same robot state/Jacobian; joint state and homing references.
- Pose/velocity errors, operating mode, command age, safety/limit flags and
  effective impedance gains. Cartesian-reference fields are NaN when inactive
  or homing, avoiding artificial zero tracking errors. Measured motion remains
  available. Older/custom command producers without the new optional requested
  twist have `requested_twist_valid=false` and NaN monitoring values.
- Four torque stages and their validity/convention metadata.

`state_time_pc_monotonic_ns` timestamps the received state snapshot;
`control_time_pc_monotonic_ns` is after command-queue consumption;
`command_issue_time_pc_monotonic_ns` timestamps output preparation just before
returning from the callback. It is **not** the exact network transmission time.
The command's acquisition time, receipt time, sequence, session, reference-space
identity and OpenXR location time are retained separately. ROS header stamps are
for plotting/discovery; MATLAB uses integer monotonic timestamps. The exporter
subtracts integer clocks before conversion to seconds to preserve precision.

MATLAB plots the request at its source-acquisition time, reference at its
output-ready time, and measured motion at state time. It preserves response
lag. Source-aligned requested pose uses zero-order hold without extrapolation;
reference tracking errors use the reference/state pair used by the controller.
OpenXR location-time intervals are used for requested velocity differentiation
when available. Prediction/location time and acquisition time remain distinct.

## Four torque conventions

| Field | Meaning |
| --- | --- |
| `nominal_torque` | Current controller's requested output |
| `commanded_torque` | Current callback output after our actuator/CBF safety layer |
| `robot_desired_torque` | Incoming libfranka `tau_J_d`, excluding gravity |
| `measured_torque` | Incoming link-side torque sensor `tau_J`, including gravity |

The first two belong to the current output; readbacks precede that output taking
effect. Previous callback nominal/commanded torque and timestamps are also
recorded as alignment candidates, **not** acknowledgements. Packet loss or robot
processing can invalidate a simple one-cycle correspondence. No measured/desired
residual is claimed to isolate friction or contact force. Model gravity is
recorded for an explicitly approximate gravity-corrected residual; it depends on
configured tool/payload. None of these signals is a motor-current measurement.

The existing `robot_state` publisher now also fills `tau_j_d` and `dtau_j`.
Actuator limits default to 35% of Panda FCI torque/rate ceilings, independently
of CBF enablement. `torque_limits` and `maximum_torque_rate` record the effective
bounds after additional controller caps, not merely the YAML's raw caps. See
[editable actuator settings](CBF_PANDA.md#editable-actuator-limits-including-cbf-disabled-operation).
Mock torque readbacks/gravity are NaN with validity false: the mock is a fixed
fixture and does not simulate robot dynamics, sensors or downstream processing.

Normal trigger release now captures the measured joint configuration once and
holds it with configurable joint stiffness/damping. `reason: position_hold`
identifies that mode; `motion_enabled=false` still means Quest teleoperation is
disengaged. Startup and pauses retain a fixed local anchor through ordinary
Quest input faults. A fault during engagement captures measured joints once
and switches to local holding, with fresh release/re-engagement still required.
This appears as `reason: position_hold` while input is unavailable. Safety
fallback, shutdown and invalid-state termination retain their separate stop
policy. The hold reference is fixed until a mode transition; it does not follow
each new measured sample. All laws share the same final safety path.
The `controller.hold_stiffness`, `hold_damping` and
`hold_stiffness_ramp_seconds` settings are in both runtime templates.

Measured pose, twist and all four torques continue recording during position
holding. Cartesian requested/reference fields and active tracking metrics remain
unavailable/excluded outside engaged teleoperation; they do not describe a joint
hold target. MATLAB's mode plot identifies position-hold intervals separately.

## Build and live dashboard

**ROS container terminal**, from a Humble shell:

```bash
cd /ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install --packages-up-to teleop_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
```

Rebuild/restart the ROS pipeline because the command interface gained optional
monitoring fields. No APK build or headset reinstall is needed. Existing
controller laws use the shared runtime and safety path. Current configuration
uses the `cbf` actuator scales described above.

Start the normal teleoperation launch using the selected runtime YAML. In a
**second host terminal**, open another shell in the existing ROS container:

```bash
docker exec -it franka-teleop bash
cd /ws
bash scripts/plotjuggler_tracking.sh /panda_1
```

In PlotJuggler, select **Streaming -> ROS2 Topic Subscriber**, select
`/panda_1/tracking_status`, use message header timestamps, and start streaming.
If the layout is opened before data arrives, retain the missing-series
placeholders. Alternatively connect first and load `config/plotjuggler_tracking.xml`.
The generated layout has Pose, Velocity, Four torques and Diagnostics tabs.
Requested/reference gaps are intentional when those signals are invalid.
Arrays use PlotJuggler's `.0`-based element names. Pose units are m, rotation
error rad, linear velocity m/s and angular velocity rad/s.

For dual arms pass the configured namespaces, for example:

```bash
bash scripts/plotjuggler_tracking.sh /panda_left /panda_right
```

Subscribe to both `tracking_status` topics. To save a permanent custom layout:

```bash
python3 scripts/create_tracking_layout.py --namespace /panda_left \
  --namespace /panda_right --output /ws/log/dual_tracking.xml
```

## Record and export

Keep `recording.include_default_topics: true` in the runtime YAML, or add each
arm's `tracking_samples` to a replacement topic profile. Menu recording starts
only on a deliberate left Menu press; continuous mode remains available through
YAML. Stop/finalize the episode before exporting it.

**ROS container terminal:** replace the bag-directory placeholder with an actual
recorded episode under `/ws/log/quest_bags`:

```bash
cd /ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run recording export_tracking /ws/log/quest_bags/EPISODE \
  /ws/log/analysis_export/EPISODE
```

`EPISODE` means the directory name that contains `metadata.yaml`, for example
`quest_YYYYMMDDTHHMMSSZ_RANDOMID`, not the metadata filename or an individual
`.db3` file. Use `ls /ws/log/quest_bags` to find a completed recording. Substitute
the same name in the export and MATLAB report paths to keep results grouped.

This creates one CSV per arm/topic and `manifest.json` with clocks, schema and
torque conventions. Existing exports are never overwritten. Example filename:
`panda_1_tracking_samples.csv`. Use `--topic /panda_left/tracking_samples` to
select an arm. Exporting `tracking_status` is supported explicitly, but cannot
recover omitted control cycles. Preserve the original bag and the actual YAML
used for the experiment; the CSV records the effective impedance gains.

## First-time MATLAB Docker setup and licensing

The optional `docker/matlab-compose.yaml` uses MathWorks' official
`mathworks/matlab:r2024b` image. This release is the reproducible default; select
another eligible release with `MATLAB_RELEASE`, and record it with results.
The analysis uses base MATLAB only and is written for R2024b. Consult the
[official container requirements](https://www.mathworks.com/help/cloudcenter/ug/matlab-container-on-docker-hub.html)
for licensing eligibility. Container use requires an appropriate valid license
linked to a MathWorks account and enabled for cloud/container use. Individual
and Campus-Wide licenses are described there as cloud-enabled; ask the license
administrator about other license types and release entitlement.

### Account-based first use

**Host terminal**, from the repository root:

```bash
mkdir -p log/analysis_export log/analysis_reports
docker compose -f docker/matlab-compose.yaml pull
docker compose -f docker/matlab-compose.yaml run --rm --service-ports matlab
```

Open the URL printed by the container in the host browser. Authenticate with
your own eligible MathWorks account when prompted, then open MATLAB. The browser
port is bound to localhost. Use `MATLAB_BROWSER_PORT=8889` if 8888 is occupied.
This is a local containerized MATLAB session, not a host MATLAB installation.
The Compose file mounts only analysis code (read-only) and the workspace `log`
directory (read/write). Do not store credentials or license files in the repo.
Authentication is not packaged into this image or shared with collaborators;
expect to sign in when starting a new disposable container.

In the **MATLAB command window in the browser**:

```matlab
addpath('/work/matlab');
results = runtests('/work/matlab/test_teleop_analysis.m');
assertSuccess(results);
[summary, data] = analyze_teleop( ...
    '/work/log/analysis_export/EPISODE/panda_1_tracking_samples.csv', ...
    '/work/log/analysis_reports/EPISODE/panda_1', ...
    'DerivativeWindow', 0.08, 'DPI', 600);
```

Results are in the host's `log/analysis_reports/EPISODE/panda_1`. Browser sessions
can open a saved figure with `openfig(pathToFigure,'new','visible')` for interactive
inspection. PDF/PNG files can be opened without MATLAB. Stop the disposable container with Ctrl+C in its host terminal when finished; result files persist in the bind mount.

If a separate `docker exec ... matlab -batch ...` reports License Manager Error
`-1` (license file not found), run the self-tests and analysis in the MATLAB
browser command window above. Do not assume browser sign-in also licenses a
separate batch process. For batch use, configure a supported licensing workflow
such as the network-license example below.

### University/network license and batch execution

Ask your administrator for the license-manager port/hostname, allowed release
and any named-user requirements. Connect the required university network/VPN.
Network Named User licensing can require a matching container username; follow
MathWorks' [custom-container recipe](https://www.mathworks.com/help/cloudcenter/ug/create-a-custom-matlab-container.html)
with the administrator if the default image user is unsuitable.
Supply the **actual** server address through the shell environment, for example:

```bash
export MLM_LICENSE_FILE='27000@YOUR_LICENSE_SERVER'
docker compose -f docker/matlab-compose.yaml run --rm matlab -batch \
  "addpath('/work/matlab'); r=runtests('test_teleop_analysis.m'); assertSuccess(r); analyze_teleop('/work/log/analysis_export/EPISODE/panda_1_tracking_samples.csv','/work/log/analysis_reports/EPISODE/panda_1');"
```

The server name above is a placeholder. The license server must be reachable
from Docker. Licensing/network failures happen before analysis; they do not
change the recordings. Batch execution with account-based activation should
follow your organization's supported MathWorks workflow; the example above
uses a network license and does not embed account passwords or activation keys.

### Handover and expiration

A Docker image does not transfer your personal MATLAB entitlement. If your
access expires after graduation, a collaborator can still use the scripts with
their own eligible account or the lab's valid network license. Data, source,
CSV, PDFs and PNGs remain available independently. Record the MATLAB release
and retain base-MATLAB-only code to avoid hidden toolbox requirements. Give the
next maintainer this document, the Compose file, representative bags/CSVs and
expected test results. Have the lab administrator own the shared license-server
arrangement. Never hand over personal login credentials or an activated home
folder as part of a Docker image.

No MATLAB container is automatically pulled/activated by teleoperation. The
image download and licensed execution are explicit operator actions; monitoring,
recording, CSV export and viewing finished PDFs/PNGs do not require MATLAB.

### Validation status

The ROS build and robot-free tests pass, including actual mock-controller
monitoring and two-arm bag/CSV round trips. MATLAB source files pass syntax
parsing, and the Compose configuration validates. MATLAB numeric tests and
figure export have not been executed by this agent: a separate batch process in
the existing MATLAB R2024b container failed license checkout with error `-1`.
Run the self-tests above in your licensed browser session before analyzing
an experiment. The live PlotJuggler GUI and physical robot behavior also require
operator validation; mock results do not establish tracking quality.

## Report interpretation

`analyze_teleop` writes:

- `pose_velocity.pdf/png/fig`: requested/reference/measured position and linear/
  angular velocity, orientation tracking error, command age and intervention.
- `four_torques.pdf/png/fig`: seven joints, four raw torque stages with explicit
  gravity conventions and their own timestamps.
- `diagnostics.pdf/png/fig`: operating modes, actuator-pressure flags, recording
  gaps, queue drops and carefully labelled approximate torque residuals.
- `estimated_linear_derivatives.*` and `estimated_angular_derivatives.*`:
  supporting acceleration/jerk estimates from velocity.
- `summary.json`, `summary.csv`, `analysis.mat`, and three derivative CSVs.

Every panel has an explicit visible legend, including each position/velocity
axis, all seven four-torque panels, every acceleration/jerk panel, single-signal
diagnostics and the seven joints in each residual plot. Joint colors are
consistent across the two residual panels. Legends also identify unavailable
(NaN) signals, so missing readbacks are not silently relabelled or hidden.
PDF, PNG and FIG exports retain the legends. Previously generated figures do
not change automatically: rerun `analyze_teleop` on the existing exported CSV,
preferably into a new report directory to retain the previous report.

Metrics exclude idle, homing and fallback intervals. Derivative estimates use a
centered local quadratic fit with actual times; they are noncausal analysis,
not robot feedback. Requested commands are de-duplicated before fitting.
Windows do not cross engagement/session/reference changes or large sample gaps;
boundaries/insufficient support are NaN. The default full window is 80 ms,
configurable by `DerivativeWindow`. Compare matched tasks with identical
windows and inspect sensitivity to that choice. CSV derivative columns carry
`estimated_` names and validity flags. No raw acceleration is claimed.

The velocity-lag estimate is a descriptive cross-correlation search over
0--250 ms using a common support interval. Stationary/short segments return
unavailable estimates. A maximum-boundary result means the search is inconclusive.
It does not identify one universal system delay or establish causality.

Use pose/velocity as the main usability evidence. Use torque intervention and
derivative estimates to explain events, alongside task time, corrective hand
movements and operator feedback. Accurate tracking of a filtered reference can
coexist with poor responsiveness to the operator. Smoothing or gain changes are
not performed automatically by this pipeline.

For overshoot while the trigger remains held, inspect only intervals with
`motion_enabled=true`, `cartesian_reference_valid=true` and no fallback. Compare
the requested, reference and measured pose around each movement stop, then
reference/measured twist, command age and torque magnitude/rate flags. A
stationary reference pose with a still-moving reference twist indicates
reference-path lag or inconsistency; oscillatory measured pose after both
references settle is evidence for investigating task damping/dynamics. Joint
motion with little Cartesian motion is a separate nullspace diagnostic. The
current Cartesian law has no explicit nullspace damping or posture term. See
[controller damping](PACKAGE_STRUCTURE.md#damping-and-nullspace-behavior-during-engagement).
