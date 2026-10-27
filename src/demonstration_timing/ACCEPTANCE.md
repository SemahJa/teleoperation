# Phase 3 acceptance report — 2026-10-23

> **Completion update:** Phase 3 infrastructure is complete with explicitly documented hardware-validation limitations. See [PHASE3_COMPLETE.md](PHASE3_COMPLETE.md) for the 60-second camera and live Quest/mock checks. Earlier results below remain historical evidence.

Status: software infrastructure and bounded camera diagnostics validated. Physical
camera/robot capture synchronization accuracy is NOT established. Phase 2 is
postponed and has not passed. Live robot/action alignment remains unvalidated
because no relevant telemetry was running; no controller or gripper was started.

## 1. Timestamp semantics audited

README.md contains the field-by-field origins/units/precision/uncertainty table.
Current source and message definitions were read, including latest recording and
Hand contracts. TrackingSample already preserves host state, control, output,
source and controller-receipt times. Quest carries converted acquisition time
with transport uncertainty and separate OpenXR time. GripperStatus lacks source,
operation-start and readback timestamps and is latest-only reporting; no lossless
action journal or continuous finger state is assumed. Camera metadata retains
SDK clock domain, native frame timestamp, firmware readout/exposure midpoint,
per-stream counters and arrival fields. All source timing fields remain intact.

## 2. Common-clock implementation and measured relationships

Added a new standalone demonstration_timing package, independent of controllers.
Reused Phase 1 camera-only launch and installed ROS Humble, wrapper 4.56.4 and
pinned SDK 2.56.5 RSUSB library (process-local /tmp extraction). No host package or
driver installation/binding was changed. Default installed V4L2 SDK enumerated
no cameras; /dev/video* was absent before Phase 3 testing. USB still showed wrist
5000 Mbps and scene 480 Mbps. The isolated RSUSB backend enumerated both serials.

Host ROS/system-to-monotonic brackets were measured at 10 Hz over each 12-second
observation. Integer-origin affine fits exported bounded ClockModel objects:

| Run | Fitted monotonic/ROS drift | Empirical mapping envelope |
|---|---:|---:|
| target | -0.112402 ppm | 29218 ns |
| mixed | -0.275794 ppm | 38515 ns |

The envelope is max fit residual plus half the maximum clock-read bracket, inside
the observed interval only. It is an empirical host-clock relation, NOT camera
capture uncertainty or cross-device synchronization accuracy. SDK metadata
reported global_time on all four sensor streams. Camera hardware/exposure-to-host
relations, robot-device-to-host relations and gripper readback ages were not
measured. No estimate from capture-to-callback arrival was used as a clock map.

Explicit clock IDs, segments, observed validity intervals and uncertainty are
required. Maps reject extrapolation, absent/ambiguous relationships and invalid
uncertainty. Local-publisher provenance must be explicitly asserted. Reset/step,
invalid-state, session and reference-space boundaries prevent interpolation.

## 3. Short camera diagnostics

All runs used native ROS on localhost-only domain 137 and independent camera
processes. Startup waited for both camera nodes to report readiness. Measurements
were 12 seconds, with no pixels or complete RGB-D bag saved.

| Run / camera | Requested FPS | Image RGB / aligned depth | Metadata color / depth | Exact shared image headers |
|---|---:|---:|---:|---:|
| target: wrist | 30 | 359 / 359 | 360 / 360 | 359 |
| target: scene | 30 | 359 / 355 | 360 / 360 | 355 |
| mixed: wrist | 30 | 358 / 359 | 359 / 359 | 358 |
| mixed: scene | 15 | 179 / 179 | 180 / 180 | 179 |

The target run's scene aligned-depth header subscriber saw four internal frame-gap
estimates. Metadata counters were consecutive in all streams in both final runs;
no duplicate/non-monotonic header or metadata-counter reset was observed. These
short observations do not localize every missing image to sensor, wrapper, DDS
or collector. Boundary counts can differ by one because callbacks straddle the
observation window. Missing-data masks remain essential.

RGB/aligned-depth header differences were zero for exact matches. Nearest
substitution for scene target missing frames produced approximately +/-33 ms
header differences. Equal synchronized headers can hide different native sensor
times. Within-camera firmware exposure-midpoint depth-minus-color medians:
- target wrist: -2.778000 ms.
- target scene: -2.549000 ms.
- mixed wrist: -0.923000 ms.
- mixed scene: -2.369000 ms.

These are device metadata observations, not independent exposure calibration.
Scene-minus-wrist nearest RGB-header phase (50 ms pairing tolerance):
- target: median -5.218506 ms; range [-11.522216, -1.343261] ms.
- mixed: median 0.475830 ms; range [-32.633057, 38.889893] ms.

This phase includes independent sampling, SDK clock estimates and missing frames;
it is NOT the measured offset between camera hardware clocks. The mixed run can
reuse a scene frame because 15 FPS is slower than 30 FPS. Original per-frame
header/arrival/native metadata, frame numbers and exact metadata JSON are retained
in compact evidence/target.json and mixed.json. timing_summary.json includes
all gap, arrival-minus-header and native timing statistics; no pixel data exists.
The compact files preserve parsed values and raw metadata strings exactly.

## 4. Offline robot/action alignment

Implemented normalized JSONL/diagnostic-JSON input and source-preserving mapped
records, nearest-neighbor joins with uncertainty-aware tolerance, optional bounded
joint-position interpolation, reset/validity/session masks, labelled JointState
fallback and causal past-only gripper status. Output is observation_t/action_t
plus explicit missing_data_mask/missing_reasons/provisional flags.

A teleoperation action requires a recorded source command and exact matching
controller application identity, with source <= receipt <= host output issuance
<= observation. Homing/fallback/invalid applications cannot establish an action.
The output execution field names its host-output semantics and never asserts
physical actuator acknowledgement. Matching raw Quest grip/button inputs are
retained when present. No gripper action or trajectory is fabricated.

Strict joins of the real camera reports intentionally mask all camera observations:
capture uncertainty is unknown. The explicit provisional mode filled all 359
target and 358 mixed wrist-anchor rows with nearby camera references. This includes
reuse/substitution of neighboring frames; it does NOT mean every frame arrived.
Robot/gripper/action fields stayed missing in both modes because they were not
recorded. No missing telemetry was synthesized. See evidence/offline-results.txt.

The deterministic synthetic fixture produced ten aligned anchors, six interpolated
joint states and one causally observed action, while uncertain gripper state stayed
masked. Its known clock models include an offset and drift and are labelled
synthetic. The exact result is evidence/synthetic-alignment.json.

## 5. Automated and hardware checks

23 pure synthetic cases pass, covering 30/15 FPS, known offsets and drift,
loss/duplicates/reordering, clock jumps and resets, missing/incompatible maps,
unknown/invalid uncertainty, sparse state interpolation and gap limits,
session/disconnect boundaries, JointState fallback, raw CDR header parsing,
unavailable/uncertain/future gripper observations, causal actions, future-output
rejection, and homing/fallback exclusion. Colcon's total includes one pytest
wrapper suite in addition to individual cases. No robot control tests were run.

Camera diagnostics exited successfully for both final runs with no parsing errors.
Both camera processes shut down cleanly after each test. Native wrapper warnings
included RSUSB startup/interface contention, firmware power-line-frequency default
range errors and USB2 reduced-performance notices; startup/transfer warnings are
not evidence of stability. Launch logs are retained in evidence. A passive check
of the default ROS graph found no Panda/Quest topics; isolated-domain subscriptions
also found none. Physical robot/gripper alignment was therefore not attempted.

## 6. Files added, commands and review boundary

Only src/demonstration_timing was added during Phase 3:
- package.xml, CMakeLists.txt
- demonstration_timing/__init__.py, clocks.py, metrics.py, adapters.py,
  diagnostics.py, alignment.py
- scripts/camera_timestamp_diagnostics, scripts/align_demonstration
- test/test_timing.py; examples/synthetic.jsonl and clock_models.json
- README.md, ACCEPTANCE.md and evidence/*

README.md gives exact build, pytest/colcon, short native RSUSB, mixed-FPS and offline
alignment commands. Build/test output lives under /tmp/phase3-*. Tests and clock
models are independent of rosbag2/MCAP and can be reused by a future recorder.
No episode recording, MCAP writer, extrinsics, clock discipline, hardware sync
cable, full dataset validation or long-duration stability test was implemented.

Unresolved accuracy limits: unknown camera exposure-to-host uncertainty, SDK
global-time estimation error, unmeasured robot sensor/network latency, unknown
gripper readback age, no live robot/action replay, short drift-estimation windows
and untested physical reconnect/clock-reset behavior. Strict masks prevent treating
these unknowns as verified aligned training data. Provisional output is exploratory.

No robot/Hand/safety/recording/Quest source or existing runtime configuration was
modified; no Panda controller, gripper operation or teleoperation was launched.
The original checkout and existing ADB container were preserved. No commit or merge.
Stop here for acceptance review before further phases or physical teleoperation.
