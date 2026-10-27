# Real measured-state + camera recording — 2026-10-26

**The requested bounded read-only validation passed.** No robot control loop, commands, error recovery, unlocking, homing, controller setting changes or gripper operations were performed. Operator manually enabled FCI after an earlier read-only connection refusal. The reader leaves FCI configuration unchanged. Original checkout and uncommitted work preserved; no commit or merge.

## Evidence

Native Panda192.168.3.101,libfranka0.9.2,server protocol5. All1,459 recorded state samples report **User stopped**. Reader requested100Hz; native sample interval median10ms (range6..14ms), host receipt interval median9.9997ms. This is bounded observation-loop jitter and deliberate decimation, not a1kHz control stream. Native timestamps and sample sequences are strictly increasing with sequence steps1 throughout the recorded window.

Episode:
`log/robot_telemetry_validation_20261026/camera_state_episode_retry1/bags/quest_20261026T113227Z_60a5e460`

Requested15seconds; actual bag receipt duration14.679513052seconds after discovery/startup. MCAP1,364,375,357bytes (1.36GB). Readable with zero validation errors. Robot data available=true; gripper data available=false. Expected missing topics TrackingSample and gripper_events remain documented; the episode is not represented as a full physical-control demonstration.

| Data | Recorded messages / result |
| --- | --- |
| Panda JointState | 1,459; q,dq,measured effort exactly equal native metadata |
| Measured end-effector Pose | 1,459; all translations exactly equal raw O_T_EE |
| Native/receipt metadata | 1,459; original SDK q/dq preserved exactly in full-precision SDK text |
| Wrist RGB / aligned depth | 440 / 440,29.978FPS |
| Scene RGB / aligned depth | 441 / 441,29.977FPS |
| Camera diagnostics | Zero header/metadata gaps, duplicates or timestamp reversals; common-window observer/bag differences0 for all four images |
| Sparse raw pixel checks | 32compared,0mismatches |
| Provisional robot alignment | 437/440 wrist-camera anchors matched state within50ms (99.32%); first3 remain missing |
| Gripper/action alignment | Unavailable and masked, not invented |

First recorded wrist header1793014347671632080ns precedes first recorded joint header1793014347794146279ns by122.514ms. First three camera anchors are therefore outside50ms state tolerance; later437match. There are no internal robot sample-sequence gaps. A misleading generic missing-reason string in existing Phase3 code was corrected to recognize recorded JointState fallback outside tolerance; matching, masks, source timestamps and clock models are unchanged. A regression test covers this case. Derived JSON was revalidated; raw MCAP unchanged.

Host ROS/system-to-monotonic fit empirical envelope38,966ns. Converted ROS observation stamp minus stored read-return monotonic receipt:median10,846ns,range7,867..44,403ns. These quantities describe host observation/serialization timing, not robot encoder acquisition age. Robot-native milliseconds since start are preserved as a separate clock; no capture-time offset/drift bound has been measured. Camera exposure and robot acquisition uncertainty remain unknown/null.50ms matching is explicitly provisional.

## Implementation and failures handled

Added separate `panda_state_bringup` package: only SDK Robot construction,serverVersion,readOnce; publishes existing standard JointState/Pose/String messages. No new controller or episode recorder. Distinct measured_ee_pose topic avoids feeding mapper anchor state. Startup checks refuse duplicate measurement publishers and simulated time; invalid/nonfinite data, native timestamp duplicate/reset, conflicting motion/guiding/recovery modes and network errors stop the reader without recovery or robot-state modification. No automatic SDK retries.

Synthetic tests first caught double quoting of SDK robot-mode text; fixed before live acquisition. First live attempt read state but rejected its rotation-matrix rounding (raw orthogonality residual approximately2.7e-5Frobenius norm), before any episode was recorded. It remains preserved under camera_state_episode. Retried using the repository's bounded1e-4 legacy rotation representation convention; raw O_T_EE is unchanged. Maximum recorded elementwise raw orthogonality residual approximately1.92531e-5; derived quaternion rotation correction approximately9.57382e-6. Both transformations/units and provenance are explicit, rather than silently rewriting the SDK measurement.

Added offline inspector and `scripts/record_read_only_panda.py`, reusing existing MenuRecorder. Added measured_ee_pose/read_only_state to the opt-in demonstration profile and sensor QoS. No changes to controller/Hand behavior or operator YAML. No new TrackingSample/control-cycle or execution timestamps. Existing Phase3 matching algorithms reused; only fallback missing-reason metadata corrected.

Tests: **55 relevant cases passed**:9new reader tests (6C++/3Python),17recorder,24timing (including the missing-reason regression),5camera. Build, Python/shell syntax and diff whitespace checks passed. Test logs/build logs and exact executed acquisition scripts are saved in `log/robot_telemetry_validation_20261026`. All task reader/camera/recorder processes stopped after validation; no background FCI reader remains.

## Files

Added `src/panda_state_bringup/{CMakeLists.txt,package.xml,src/read_only_state.cpp,include/panda_state_bringup/snapshot.hpp,scripts/inspect_read_only_episode.py,test/test_snapshot.cpp,test/test_read_only_guards.py,README.md,VALIDATION_REPORT.md}` and `scripts/record_read_only_panda.py`.

Modified task-owned `src/recording/recording/profiles.py`; existing Phase3 `alignment.py` missing-reason expression and its regression test. Existing controller, gripper, Quest behavior and native SDK untouched. All evidence and raw failed/successful runs retained.

See [README.md](README.md) for exact build, bounded acquisition, camera recording and inspection commands. Gripper measurement, real control receipt/output, physical execution alignment, physical demonstrations, exposure synchronization and extrinsics remain unverified/deferred. Robot User-stopped status and error-free state snapshots do not establish readiness for motion or resolution of the earlier joint-limit safety issue. Physical teleoperation requires a separate operator-reviewed task.
