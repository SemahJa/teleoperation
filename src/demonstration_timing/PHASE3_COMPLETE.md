# Phase 3 infrastructure complete

Completion check on 2026-10-23: the existing infrastructure passes its scoped
software and 60-second camera tests, with the hardware-validation limitations
below. No redesign or Phase 3 code fix was required. Recording development may
proceed using provisional alignment and explicit uncertainty/missing-data flags;
unknown exposure synchronization is not a reason to block that infrastructure.
This does not mark Phase 2 long-duration stability as passed or begin Phase 5.

## Measured camera results

Both serials verified: wrist D435 934222072657 (5000 Mbps), scene D435i
238222073675 (480 Mbps). Both opened 640x480 RGB8/Z16 at 30 FPS with aligned depth
and enable_sync. This run used the native installed SDK 2.56.5, without the earlier
process-local RSUSB override. Observation duration: 60.017 seconds.

| Camera | Observed RGB / aligned depth Hz | Image counts RGB / depth | Internal header-gap estimates RGB / depth | Metadata Hz |
|---|---:|---:|---:|---:|
| wrist | 29.93 / 29.91 | 1794 / 1793 | 3 / 4 | 29.98 |
| scene | 29.69 / 29.23 | 1780 / 1753 | 17 / 45 | 29.98 |

Rates use first-to-last subscriber arrival extent; full-window rates are also in
completion-summary.json. Each metadata stream had 1,798 samples, consecutive
frame counters, no duplicates, resets or non-monotonic timestamps. Image gaps
remain observations of the wrapper/DDS/collector path, not proof of sensor loss.
The wrist emitted a startup depth-stream-failure notification, but subsequently
streamed throughout the measured window. No stream setting was silently lowered.

Exact shared RGB/depth headers: wrist 1,791/1,794 RGB headers (99.83%), scene
1,751/1,780 (98.37%). Missing counterparts can be replaced only provisionally by
nearby frames, not fabricated. All metadata domains reported global_time.
Native SDK depth-minus-color timestamp medians were -17.090 us (wrist) and
-58.105 us (scene); firmware exposure-midpoint metadata was unavailable in this
backend/run. Equal ROS headers do not establish simultaneous exposures.

Scene-minus-wrist nearest RGB-header offset: median -4.026002 ms, range
[-16.230713, +31.311767] ms. This is sampling/header phase, NOT a measured physical
camera-clock offset. The host ROS-to-monotonic affine fit had a 37.264 us empirical
envelope and fitted relative drift -0.004185 ppm, inside its 60-second anchor
interval. Exact model/offset/anchors are preserved. Neither this envelope nor
SDK global_time certifies camera capture-time accuracy; capture uncertainty is
still unknown.

At the declared 50 ms tolerance, provisional alignment covered all 1,794 wrist
anchors with four camera references. Maximum absolute match deltas: wrist depth
33.357178 ms, scene RGB 31.311766 ms, scene depth 36.529296 ms. Neighbor reuse is
explicit; 100% matching coverage does not mean 100% frame delivery. Strict mode
continues to mask unknown capture uncertainty. Provisional flags and original
source timing are preserved in the losslessly compressed JSON evidence.

## Live Quest with mock measured pose

After the operator authorized ADB, a separate 20-second native check ran cameras,
Quest USB bridge, mapper and mock_measured_pose. No arm/Hand owner was launched.
It received 1,347 right-controller samples and 2,296 mapped-command messages.
All 1,347 acquisition+offset conversions and 2,295 source-field comparisons
matched exactly; there were zero timestamp mismatches. Quest-reported clock
uncertainty was 0.402764–0.621697 ms
(median 0.485489 ms). These are existing transport estimates, not camera accuracy.

Pose-validity remained false, with zero motion-enabled commands; the mapper stayed
disarmed. Thus source-time propagation is verified, active motion mapping and
controller execution are not. All 592 live-test wrist anchors had provisional
camera matches; robot/gripper/action execution slots stayed missing. No trigger,
pose-validity flag, robot measurement or gripper state was fabricated.

The first container harness lacked its camera-message environment; after correcting
that environment, live Quest messages arrived but native camera data was not visible
there. The final native check above passed. These were test-harness/environment
issues; no Phase 3 implementation change was needed. The temporary ADB reverse
mapping and all test camera/mock processes/containers were removed. The previously
running Quest app and ADB server were left running.

## Tests and artifacts

- 23 existing Phase 3 synthetic cases passed.
- 5 camera configuration cases passed; colcon reports 30 total including two wrappers,
  zero failures/errors/skips.
- 4 existing USB-free Quest bridge/mapper integration cases passed.
- Final 60-second camera and 20-second native camera/Quest/mock diagnostics exited 0.
- Real Panda telemetry, continuous gripper feedback, motion-enabled Quest commands,
  physical execution timing, actual exposure synchronization and long stability
  remain unverified. No physical robot control or gripper operation was started.

Only this report, the completion note in README.md/ACCEPTANCE.md, and evidence files
were added/updated. Existing implementation/configuration and the original worktree
were preserved. No commit, merge or Phase 5 work was performed.

Evidence: [completion-summary.json](evidence/completion-summary.json), compressed
original reports/provisional alignments, and test/launch logs in evidence/completion-*.
Gzip compression is lossless; SHA256 hashes of decompressed originals are recorded.
No image pixels or full demonstration episode were recorded.

## Reproduction commands

```bash
cd ~/aljabnouni/franka_teleoperation_cameras
source /tmp/phase3-install/setup.bash
export ROS_DOMAIN_ID=137 ROS_LOCALHOST_ONLY=1
ros2 launch camera_bringup cameras.launch.py cameras:=both
# Second sourced terminal, after both cameras report readiness:
ros2 run demonstration_timing camera_timestamp_diagnostics \
  --seconds 60 --wrist-fps 30 --scene-fps 30 --pair-tolerance-ms 50 \
  --local-camera-clock --observe-robot --output /tmp/phase3-60s.json
PYTHONPATH=src/demonstration_timing python3 -m pytest -q src/demonstration_timing/test
colcon --log-base /tmp/phase3-finish/test-log test \
  --base-paths src/camera_bringup src/demonstration_timing \
  --build-base /tmp/phase3-build --install-base /tmp/phase3-install \
  --packages-select camera_bringup demonstration_timing
colcon test-result --test-result-base /tmp/phase3-build
# Reproduce provisional joins using the saved original 60-second report:
gzip -dc src/demonstration_timing/evidence/completion-60s.json.gz > /tmp/phase3-60s.json
REFERENCE=$(python3 -c 'import json; print(json.load(open("/tmp/phase3-60s.json"))["reference_clock"])')
ros2 run demonstration_timing align_demonstration /tmp/phase3-60s.json \
  --reference "$REFERENCE" --tolerance-ms 50 --allow-unknown-uncertainty \
  --output /tmp/phase3-provisional.json
```

The live check's exact temporary harness and native build logs remain in
/tmp/phase3-finish/run-native-live.py and native-mock-build.txt. It launches only
quest_teleop.launch.py with start_bridge:=true start_mapper:=true
mock_measured_pose:=true and standalone arms.yaml (no hardware owner). A matching
native Quest/message install is at /tmp/phase3-finish/native-install. The bridge
and cameras shared verified host monotonic/system clocks. The existing diagnostic
used --telemetry-host-clock-id with the current host/boot identity, not an assumed
remote clock. See README.md for build and clock-domain contracts.
