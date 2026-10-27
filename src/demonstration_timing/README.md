# Demonstration timestamp infrastructure — Phase 3

> **Completion update:** Phase 3 infrastructure is complete with explicitly documented hardware-validation limitations. See [PHASE3_COMPLETE.md](PHASE3_COMPLETE.md) for the 60-second camera and live Quest/mock checks. Earlier results below remain historical evidence.

This package adds passive diagnostics and offline joins. Phase 2 long-duration
stability testing is postponed, not passed. It does not change or start robot,
gripper, Quest, recorder, torque-control or safety code. No control publisher,
service client or action client exists in this package.

## Timestamp audit and interpretation

Audited worktree `franka_teleoperation_cameras`, branch
`feat/dual-realsense-cameras`, HEAD e682b17716a3a246a981a64e9e85d28b103a5f97.
Phase 1 files were still uncommitted and were preserved. The previous/original
checkout was only read. No fetch, merge, switch, reset or commit was performed
in Phase 3. Exact evidence and measured results are in ACCEPTANCE.md.

Precision below describes representation or quantization, not synchronization
accuracy. ROS Header has no clock-domain identifier: a numerical match alone
cannot prove clock compatibility.

| Source / fields | Origin and semantics | Units / representation precision | Uncertainty / restrictions |
|---|---|---|---|
| Camera color and aligned-depth `header.stamp` | Wrapper 4.56.4 `frameSystemTimeSec`: HARDWARE_CLOCK is anchored once to wrapper ROS-now; GLOBAL_TIME/SYSTEM_TIME uses SDK timestamp converted to epoch ns | ROS sec + nanosec, nominal 1 ns; SDK double milliseconds is coarser (about 244 ns double spacing at this epoch) | SDK clock-conversion/capture uncertainty not supplied. Same frameset can reuse one header across streams. Never infer simultaneous exposures from equal headers |
| `/cameras/ROLE/color/metadata`, `/depth/metadata`: `header.stamp`, `json_data.frame_timestamp`, `clock_domain` | Header is wrapper publication frame stamp; JSON frame_timestamp is SDK `get_timestamp()` with its reported domain | JSON frame_timestamp double milliseconds; original JSON retained byte-for-byte as string; Decimal conversion to integer ns | GLOBAL_TIME is SDK-estimated host/system time, not measured absolute capture accuracy. HARDWARE_CLOCK is device-local and can reset. SYSTEM_TIME is host/backend-derived, not sensor exposure time |
| Metadata `hw_timestamp` | Device clock at readout/transmission start (SDK FRAME_TIMESTAMP renamed by wrapper) | Integer microseconds | Device-local; possible wrap/reset; not directly comparable across cameras |
| Metadata `sensor_timestamp` | Device-calculated exposure midpoint | Integer microseconds | Device-local; firmware metadata, not independently calibrated exposure time |
| Metadata `frame_number`, `frame_counter` | Sequential per-stream indices | Integer frame count | RGB/depth counters may have different starting values. Gaps are observed stream-delivery gaps; they do not alone identify sensor/USB/DDS loss |
| Metadata `time_of_arrival`, `backend_timestamp` | SDK/backend arrival measurements, distinct from this subscriber | Arrival system-clock value (observed integer ms here); backend timestamp SDK-defined microseconds, observed zero with RSUSB | Zero backend timestamp is unavailable; arrival is not capture. Unrecognized metadata fields remain raw |
| `/panda_1/tracking_samples.header.stamp` | ROS-now minus age of the state callback monotonic timestamp, constructed off realtime | ROS sec/nanosec | Projection error from sequential clock reads; use native state timestamp when publisher host is verified |
| TrackingSample `state_time_pc_monotonic_ns` | Host CLOCK_MONOTONIC read at FCI callback entry | uint64 ns | Host callback observation, NOT robot encoder acquisition; sensor/network age unmeasured |
| TrackingSample `control_time_pc_monotonic_ns` | Host input/freshness check within that cycle | uint64 ns | Different point in the same cycle, not interchangeable with state time |
| TrackingSample `command_issue_time_pc_monotonic_ns` | Host timestamp after output calculation/before monitor queue push | uint64 ns | Output-issuance observation; not physical torque execution or acknowledgement |
| TrackingSample `command_source_time_pc_monotonic_ns`, `command_receipt_time_pc_monotonic_ns` | Retained requested-command source time and controller subscription receipt time | uint64 ns | Receipt is host-local. Source inherits Quest synchronization uncertainty, which TrackingSample itself does not carry; join to source command |
| TrackingSample `previous_command_issue_time_pc_monotonic_ns`, previous control cycle | Previous callback output association | uint64 ns and integer cycle | Alignment candidate, not causal proof that an output produced the next readback |
| `/panda_1/joint_states.header.stamp` | Same ROS projection of original host state snapshot, published by approximately 100 Hz non-real-time telemetry timer | ROS sec/nanosec; sampled/downsampled | q/dq are actual recorded snapshot values. No original monotonic field here; prefer TrackingSample. Do not treat publication cadence as controller cadence |
| `/panda_1/robot_state.header.stamp`, `.time` | Header is same state projection; time is native libfranka RobotState.time.toSec() | Header ns; time double seconds from millisecond-backed franka::Duration | Robot-relative monotonic time, epoch not bound to host here. No robot-relative-to-host calibration measured. Native robot_state is audited but not a required diagnostic subscription |
| CartesianTeleopCommand `header.stamp` | Mapper projects converted source acquisition time into ROS clock; disarm path can use current ROS time | ROS sec/nanosec | Input/motion/clock flags must be checked; header alone cannot identify actuation |
| CartesianTeleopCommand `source_acquisition_time_ns`, `openxr_location_time_ns`, `source_time_pc_monotonic_ns`, `clock_uncertainty_ns` | Original Quest acquisition clock, OpenXR location clock, existing acquisition-to-host estimate and uncertainty | Integer ns | OpenXR location time and acquisition time are distinct; only acquisition conversion is explicitly supplied. Preserve validity/session/reference-generation flags |
| QuestControllerState `header.stamp` | USB bridge ROS receipt/construction time | ROS sec/nanosec | Not acquisition time |
| QuestControllerState `acquisition_time_ns`, `openxr_location_time_ns` | Quest CLOCK_MONOTONIC sample time and OpenXR XrTime location time | uint64 ns | Distinct semantics; never add host offset to XrTime without an independently verified relation |
| Quest `clock_offset_pc_minus_quest_ns`, `clock_uncertainty_ns`, `clock_sync_age_ns`, `source_time_pc_monotonic_ns` | Existing transport clock exchange estimates host minus Quest offset, age and uncertainty; converted acquisition is acquisition + offset | Signed/unsigned integer ns | Use only valid/fresh clock and source flags; uncertainty is inherited from transport, not newly measured by this package |
| `/panda_1/gripper_events.header.stamp` | ROS time when status is constructed before latest-only reporting thread | ROS sec/nanosec | Readback age and request/start/physical-completion timestamps absent. Width is sparse feedback, not continuous measured motion. Counters are not a lossless event journal |
| Collector `arrival_ros_ns`, `arrival_monotonic_ns` | Local callback arrival, bracketed around a local ROS-clock read | Integer ns; actual clock resolution recorded by OS, not assumed sensor accuracy | Transport, executor, deserialization/serialization and scheduling delays remain. Never substitute these values for capture timestamps |

Source inspection: controller_runtime.cpp and tracking_message.hpp;
quest_teleop/pose_mapper.py and quest_usb_bridge.cpp; GripperStatus.msg and
franka_hand_worker.cpp/hand_reporting.hpp. Camera definitions were read from the
pinned wrapper's base_realsense_node.cpp and SDK include/librealsense2/h/rs_frame.h
inside the Phase 1 image. No external version assumptions were used.

## Common-clock strategy

The preferred reference is `host_monotonic:HOSTNAME:BOOT_ID`. All source timestamps
remain integers in their original units/fields. Clock models add `common_ns`;
they never overwrite timestamp_ns, header_ros_ns or native metadata JSON.
Cross-host or cross-boot clocks require explicit measured relationships. Time
namespaces and publisher provenance must also be verified; the collector records
its time-namespace offsets. A ROS graph topic name is not a host identity.

The collector brackets local ROS-clock reads with CLOCK_MONOTONIC reads at 10 Hz.
For bracket [m0,m1], it pairs ROS time r with midpoint (m0+m1)/2 and records
half-bracket uncertainty. `fit_clock` estimates an affine relation using integer
origins to avoid epoch-float precision loss:

`common_ns = reference_origin_ns + round((native_ns - source_origin_ns) * rate)`.

The empirical uncertainty envelope is maximum fit residual plus maximum supplied
pair-measurement uncertainty. It applies only inside the observed anchor interval;
extrapolation is rejected. Offset steps above 2 ms, non-monotonic host anchors or
reset clocks split epochs. Smaller/unobserved steps between samples cannot be
ruled out: this is an empirical mapping, not a hard synchronization guarantee.
Models with excessive residuals are rejected. Original anchor brackets are saved.

The reusable fit_clock API requires >=3 simultaneous clock pairs and a known
pair-measurement uncertainty. Device capture versus subscriber arrival is NOT a
simultaneous clock pair and must never be fed to it. No camera hardware-clock
or exposure-to-host offset/drift was calibrated in these tests. SDK GLOBAL_TIME
is retained as an estimate with UNKNOWN capture uncertainty. The observed
header/inter-camera phase statistics do not estimate true capture-clock offsets.

`--local-camera-clock` explicitly asserts that the camera wrappers are using this
host/system ROS clock. Without it, image-header clocks stay unverified and cannot
be joined to the local reference. Use it only when publisher provenance is known.
`--telemetry-host-clock-id host_monotonic:HOST:BOOT` is likewise an explicit
verified binding for robot/Quest monotonic fields. Without it, telemetry clocks
remain unverified and are rejected by common-clock joining. Other-host ROS headers
are not assigned to this host automatically. Simulation clocks are rejected.

Strict alignment defaults to rejecting UNKNOWN timestamp uncertainty, including
camera capture uncertainty and gripper readback age. `--allow-unknown-uncertainty`
enables an explicitly provisional timing join; masks/provisional flags distinguish
this exploratory output from calibrated synchronization. It never permits an
unmapped/incompatible clock. Known uncertainty from both endpoints is included in
the matching tolerance, not discarded. Do not quote the host-clock fit uncertainty
as camera/robot synchronization accuracy.

## Diagnostics and small-data contract

The utility subscribes to color and aligned-depth images with `raw=True`, reads
only the CDR Header, then releases the serialized buffer. It does not deserialize
or copy pixel arrays in application code. RMW/rclpy still transfers serialized
image bytes; this is not zero-copy transport. `--metadata-only` avoids image
subscriptions entirely, but cannot check aligned-image headers. Metadata JSON is
small and parsed separately. Every retained image is a topic/header/frame-id
reference, ready for a future recorder to resolve to stored pixels.

Reports include per-stream header gaps, duplicate/non-monotonic stamps, expected
FPS gap estimates, metadata frame-counter gaps/duplicates/resets, native hardware
timestamp decreases, paired RGB/aligned-depth header differences, native SDK,
readout and exposure-midpoint metadata differences, inter-camera nearest-header
phase, arrival-minus-header statistics, raw metadata and host-clock brackets.
Different expected FPS can be supplied for wrist and scene. Nearest pairing uses
a tolerance and can reuse a slower-camera frame; it is not one-to-one acquisition
synchronization. Missing-frame estimates distinguish observation gaps from proof
of a sensor dropping a frame.

With `--observe-robot`, only already-discovered matching types on these topics are
subscribed: /panda_1/tracking_samples, /panda_1/joint_states,
/panda_1/cartesian_teleop_command, /quest_usb_bridge/right/controller_state,
/panda_1/gripper_events. The utility creates no publisher, service/action client
or hardware owner. It does not start a missing telemetry source. Missing custom
message installations are reported. Recorded messages retain all original integer
timing fields and payloads; non-finite numeric values are represented by JSON null
for invalid/unavailable data, never treated as real measurements.

## Offline alignment

`align_demonstration` accepts the diagnostic JSON directly, or normalized JSONL
plus an explicit clock-model JSON list. It is storage-independent: future MCAP
readers can reuse adapters.adapt and clocks/alignment without importing ROS.
No bag writer, episode lifecycle or MCAP plugin is implemented here.

Each normalized record requires record_id, stream, timestamp_ns (integer native
ns), clock_id, clock_segment, segment (session/validity segment), valid,
uncertainty_ns (integer or null), and payload. Camera payloads retain frame_ref and a metadata_event_record_id referencing the
exact metadata_json stored once in events (older reports may inline it). Other original timing fields remain in payload.
Clock models require source, reference, segment, source_origin_ns,
reference_origin_ns, rate, uncertainty_ns, valid_from_ns, valid_to_ns, method.
The reference must be host_monotonic because current action receipt/issuance
contracts are explicitly host-monotonic.

Outputs have observation_t={wrist_rgb,wrist_depth,scene_rgb,scene_depth,robot_state,
gripper_state}, action_t, missing_data_mask, missing_reasons and provisional flags.
The anchor is a configurable camera stream, wrist_rgb by default. Observations
are at wrapper header timestamps, not independently measured exposure midpoints.

- Camera/state matching uses nearest neighbors inside the configured tolerance.
  Slow-camera frames may be reused, with exact source IDs retained.
- Robot state prefers TrackingSample, falling back to recorded JointState only
  when suitable tracking data is absent; the source stream is labelled.
- Optional interpolation covers joint positions only, between consecutive valid
  recorded samples with the same session/epoch, a bounded gap and both endpoints
  within tolerance. It never interpolates pose/quaternion, gripper, images or
  actions. It is offline interpolation and can use a future state endpoint.
- Resets, non-monotonic time, invalid samples, session/reference-generation and
  mode boundaries split segments before time indexing. Overlapping numerical
  timestamps from reset epochs are ambiguous and masked. Original order is kept
  for interpolation adjacency; sorting never hides a reset. Long gaps are gated.
- Gripper uses the latest past status inside tolerance, never a future status or
  interpolated width. Busy/uncalibrated/unknown states are invalid; readback-time
  uncertainty remains unknown even for a calibrated, completed status.
- action_t requires an exact session/sequence/reference-generation command match
  to an observed TrackingSample application, with source <= controller receipt
  <= output issuance <= observation, including available known uncertainty.
  The latest invalid/faulted application blocks reuse of an earlier action.
  Homing/fallback/non-Cartesian samples do not establish a teleoperation action.
  `applicable_execution_timestamp` explicitly means observed host output issuance,
  not physical execution. Missing application telemetry leaves action masked.
  Original Quest grip/button inputs are attached only on an exact matching source
  key. There is no fabricated gripper action/request/execution timestamp.

## Reproduce build and synthetic checks

```bash
cd ~/aljabnouni/franka_teleoperation_cameras
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/phase3-colcon-log build \
  --base-paths src/camera_bringup src/demonstration_timing \
  --build-base /tmp/phase3-build --install-base /tmp/phase3-install
source /tmp/phase3-install/setup.bash
PYTHONPATH=src/demonstration_timing python3 -m pytest -q src/demonstration_timing/test
colcon --log-base /tmp/phase3-test-log test \
  --base-paths src/demonstration_timing --build-base /tmp/phase3-build \
  --install-base /tmp/phase3-install --packages-select demonstration_timing
colcon test-result --test-result-base /tmp/phase3-build/demonstration_timing
ros2 run demonstration_timing align_demonstration \
  src/demonstration_timing/examples/synthetic.jsonl \
  --models src/demonstration_timing/examples/clock_models.json \
  --reference host_monotonic:synthetic:boot --tolerance-ms 40 \
  --interpolate-state --output /tmp/phase3-synthetic-aligned.json
```

## Reproduce short native diagnostics

The installed V4L2 SDK currently cannot enumerate the cameras: their UVC driver
interfaces are unbound and /dev/video* is absent. USB devices remain present.
No driver binding or host installation was changed. An isolated SDK 2.56.5 RSUSB
library from the already-built pinned Phase 1 image was reused for native ROS
processes, via process-local LD_LIBRARY_PATH only. This is distinct from claiming
that the default installed V4L2 backend was validated in Phase 3.

```bash
# Extract only the library into /tmp, with no host SDK installation.
docker run --rm --network none --entrypoint bash \
  franka-dual-realsense:4.56.4-sdk2.56.5 -lc \
  'tar -C /usr/local/lib -chf - librealsense2.so.2.56' > /tmp/phase3-rsusb.tar
mkdir -p /tmp/phase3-rsusb
tar -xf /tmp/phase3-rsusb.tar -C /tmp/phase3-rsusb
source /tmp/phase3-install/setup.bash
export ROS_DOMAIN_ID=137 ROS_LOCALHOST_ONLY=1
# Camera terminal only: use the isolated backend for these processes.
LD_LIBRARY_PATH=/tmp/phase3-rsusb:$LD_LIBRARY_PATH \
  ros2 launch camera_bringup cameras.launch.py cameras:=both
# Wait for both wrappers to report readiness. In a second equally sourced terminal:
ros2 run demonstration_timing camera_timestamp_diagnostics \
  --seconds 12 --local-camera-clock --observe-robot --output /tmp/phase3-target.json
# Mixed FPS: stop the camera launch, then restart it with scene_profile:=usb2_15fps.
# Repeat diagnostics with --scene-fps 15 and --output /tmp/phase3-mixed.json.
```

Do not start teleoperation to obtain missing topics. A passive default-domain graph
check found no Panda/Quest telemetry during this task, so live robot/action
alignment was not tested. Panda 109's safety issue remains outside this work.

```bash
REFERENCE=$(python3 -c 'import json; print(json.load(open("/tmp/phase3-target.json"))["reference_clock"])')
ros2 run demonstration_timing align_demonstration /tmp/phase3-target.json \
  --reference "$REFERENCE" --output /tmp/phase3-strict.json
# Exploratory header-based join with UNKNOWN capture uncertainty explicitly allowed:
ros2 run demonstration_timing align_demonstration /tmp/phase3-target.json \
  --reference "$REFERENCE" --allow-unknown-uncertainty \
  --output /tmp/phase3-provisional.json
```

No camera-to-camera trigger synchronization, camera/robot hardware-clock
calibration, extrinsic calibration, real robot/gripper movement, long stability
test or full episode recording is part of Phase 3. Review ACCEPTANCE.md before
using provisional matches for training.
