# Read-only Panda observation

`read_only_state` uses only `franka::Robot` construction, `serverVersion()` and `readOnce()`. It never starts a controller, changes robot settings, unlocks joints, recovers errors, homes or connects to the gripper. The reader exits on network errors, non-increasing native robot time or a mode transition to Move/Guiding/AutomaticErrorRecovery. It performs no automatic reconnection or mode changes. It checks for pre-existing publishers on its three measurement topics before connecting.

This package is separate from the existing controller/Hand launch. **Do not use the hardware `single_panda.yaml` launch for this test:** that configuration enables controller/homing/Hand behavior. Keep Desk in the operator-selected stopped state. The reader does not establish safety or readiness for motion.

## Build using the verified SDK

Native libfranka0.9.2 at `/usr/local` was already installed and successfully read this Panda (192.168.3.101,server protocol5). CMake requires this exact SDK version; no SDK upgrade or alternate SDK installation is performed.

```bash
cd ~/aljabnouni/franka_teleoperation_cameras
source /opt/ros/humble/setup.bash
source install/demonstration/local_setup.bash
colcon --log-base log/read_only_state_build build \
 --base-paths src/panda_state_bringup \
 --build-base build/read_only_state --install-base install/read_only_state \
 --cmake-args -DBUILD_TESTING=ON -DFranka_DIR=/usr/local/lib/cmake/Franka
source install/read_only_state/local_setup.bash
```

First build the existing six-package camera/recording workspace using its documented procedure if `install/demonstration` does not yet exist. This read-only package adds no runtime dependency on controller or gripper owners.

## Bounded reader only

With FCI already manually enabled and the robot stopped:

```bash
ROS_DOMAIN_ID=139 ROS_LOCALHOST_ONLY=0 \
FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/camera_bringup/config/fastdds_local_large_frames.xml" \
ros2 run panda_state_bringup read_only_state --ros-args \
 -r __ns:=/panda_1 -p robot_ip:=192.168.3.101 \
 -p sample_hz:=100.0 -p duration_seconds:=15.0
```

`robot_ip` has no default. Sample frequency1..200Hz, duration(0,300]seconds. This is a non-real-time observation loop, deliberately subsampling the SDK state stream. Do not interpret skipped native1kHz updates as recorded-message losses. No Quest input subscription, control service or torque publisher exists. RealtimeConfig::kIgnore disables only the client's realtime scheduling requirement; it changes no robot safety setting.

## Camera + measured-state recording

This starts cameras and the read-only reader, then calls the existing MenuRecorder for a bounded episode. It starts no Quest mapper, robot controller or Hand worker. It requires no headset interaction and creates no second recorder implementation.

```bash
export ROS_DOMAIN_ID=139 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/camera_bringup/config/fastdds_local_large_frames.xml"
python3 scripts/record_read_only_panda.py --robot-ip 192.168.3.101 \
 --seconds 15 --sample-hz 100 --output-root "$PWD/log/read_only_panda_episodes"
```

Both camera profiles stay explicitly640x480,30FPS. Do not run another camera launch against these serials simultaneously. All launched processes are bounded and stopped on exit. Existing operator-selected FCI mode is left unchanged. Failed attempts/logs and partial recordings are preserved; no auto error recovery or settings changes are attempted.

## Recorded contracts

| Topic | Type | Timestamp meaning |
| --- | --- | --- |
| `/panda_1/joint_states` | sensor_msgs/JointState | ROS observation immediately after readOnce returns; measured q,dq,tau_J (link-side sensor torque), not desired actions |
| `/panda_1/measured_ee_pose` | geometry_msgs/PoseStamped | Same observation stamp, translation from native O_T_EE; bounded rotation projection for quaternion representation |
| `/panda_1/read_only_state` | std_msgs/String JSON | schema panda_read_only_state_v1, explicit robot address/native clock identity, session/sequence, robot-native milliseconds since robot start, host read-before/receipt CLOCK_MONOTONIC nanoseconds, matching ROS header, original q/dq/tau_J/O_T_EE and SDK state text |

All three are standard ROS messages with best-effort volatile QoS. The opt-in demonstration recorder profile includes these topics. The dedicated pose topic does not supply the mapper's mock/teleop anchor input.

No `TrackingSample` is fabricated: control/receipt/output issue timestamps are unavailable without a real controller. Native robot time remains a distinct device clock; no offset/drift model from device acquisition to host capture time is asserted. Read-call duration includes waiting/transport, and is not an acquisition-age uncertainty bound. Capture uncertainty and execution timestamp remain null.

The robot's legacy SDK matrix showed small non-orthogonality. As in the repository's existing `safety_filter/legacy_pose.hpp`, a1e-4 rounding bound is checked before converting to an orthonormal rotation/quaternion. This affects only the derived ROS pose; raw O_T_EE and full-precision native state remain unchanged. The correction and residual are recorded explicitly. Large/nonfinite/malformed transforms are rejected.

## Offline inspection and alignment

```bash
ros2 bag info PATH_TO_EPISODE
python3 -m recording.validate_episode PATH_TO_EPISODE
ros2 run panda_state_bringup inspect_read_only_episode PATH_TO_EPISODE
```

The existing Phase3 aligner uses measured JointState fallback; no algorithm or clock-model redesign was made. Its derived `robot_state` observations remain provisional because capture uncertainties are unknown. The inspector verifies exact native-to-standard-message values, timing continuity, source clock identities and pose representation provenance. Raw MCAP is never modified. `read_only_state_validation.json`, manifest, timing_records.jsonl and alignment_provisional.json are derived artifacts.

Robot state outside the configured50ms window remains missing. Gripper state and action/execution fields remain masked. Matching within50ms is not encoder/camera exposure synchronization or actuator acknowledgement.

## Tests

```bash
ROS_DOMAIN_ID=142 ROS_LOCALHOST_ONLY=1 colcon --log-base log/read_only_state_tests test \
 --base-paths src/panda_state_bringup --build-base build/read_only_state \
 --install-base install/read_only_state --event-handlers console_direct+
colcon test-result --test-result-base build/read_only_state --verbose
```

Nine cases passed: six C++ projection/provenance tests and three startup-refusal tests (invalid parameters, sim clock, existing measurement publisher). Tests use no live robot. ROS/ament reports additionally count wrapper test entries. Relevant existing recorder/camera/timing tests passed too; see [VALIDATION_REPORT.md](VALIDATION_REPORT.md).

Primary SDK reference: [Franka state-only read/readOnce documentation](https://support.franka.de/docs/libfranka.html). Actual0.9.2 installed headers were used for compatibility. Native acquisition is tested; this package has not been added to or hardware-tested in the optional Docker image.
