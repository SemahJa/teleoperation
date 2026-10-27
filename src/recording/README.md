# Demonstration recording (Phase 5)

The existing `MenuRecorder` owns the rosbag2 subprocess. Camera subscriptions, clock observations, manifests and offline validation run outside real-time robot control. Original image bytes, message fields and timestamps are recorded without resampling. The existing sqlite3 configuration remains supported; `config/demonstration.yaml` explicitly selects MCAP and late discovery.

## Lossless storage optimization

The demonstration profile now selects `mcap_compression: Zstd` and
`mcap_compression_level: Fast`. CRCs and message indexes remain enabled.
Existing sqlite3 and unspecified legacy compression settings are unchanged.
The prepared hardware runtime uses the same options; its earlier YAML is saved as
`log/real_demonstration_20261026/hardware.pre_zstd.yaml`.

Both native and prepared Docker overlays were rebuilt. No new SDK/image installation
or camera/controller change is required. For a newly built optional Docker image,
rebuild from the updated source as usual. Settings apply when starting the next
recorder; an already running node is not reconfigured.

See [compression benchmark](COMPRESSION_RESULTS.md): the144.30s episode shrank
from13.61GB to3.11GB (77.2%), with all337,880messages verified byte-identical.
Plain timing/alignment exports remain available for Phase3 compatibility; their
verified `.zst` archival copies are much smaller. Originals have not been deleted.

## Verified native startup

From this camera worktree, after authorizing USB debugging:

```bash
cd ~/aljabnouni/franka_teleoperation_cameras
ROS_DOMAIN_ID=99 scripts/start_demonstration_recording.sh
```

The script builds six packages from this worktree into `build/demonstration` and `install/demonstration`, forwards Quest TCP45454, foregrounds the installed app and launches both RealSense cameras, Quest USB bridge, mapper, fixed mock measured pose and the existing Menu recorder. It launches no Panda or physical gripper owner. Do not source another checkout's overlay first.

Wear/focus the telemetry app and wake controllers. Release left Menu before pressing it once to start. Press it again to stop. Wait until `/quest/recording_status` says `idle` and `episode validation complete`; presses during finalization/validation are ignored. In mock mode, release the right index trigger, then hold it to test enabled mapper commands. Ctrl-C gracefully stops the pipeline and removes forwarding added by this invocation. The ADB server and persistent keys are retained.

Environment options: `DEMONSTRATION_OUTPUT_ROOT` (default this worktree's `log/demonstrations`), `QUEST_SERIAL`, `QUEST_ADB_CONTAINER`, `ROS_DOMAIN_ID`. Keep `QUEST_PORT=45454` for the installed app's compiled endpoint. Launch arguments can be appended, e.g. `cameras:=wrist`, `cameras:=scene`, `scene_profile:=usb2_15fps`; see camera_bringup for valid profile names. Profiles never fall back automatically. The tested target is RGB8 and aligned16UC1,640x480,30FPS, wrist934222072657 and scene238222073675. Scene still negotiates USB2.

Native prerequisites already installed for this validation: ROS2 Humble, wrapper4.56.4/runtimeSDK2.56.5, MCAP storage0.15.17, nlohmann-json3-dev3.10.5-2, numpy and colcon. Install a missing MCAP plugin with `sudo apt-get install ros-humble-rosbag2-storage-mcap`; it does not install a RealSense SDK. Current native wrapper emits a compile/runtime SDK warning retained in validation logs. Do not replace SDK libraries merely to suppress it.

## Follow-up: tested camera transport

The loss investigation found a process-local DDS buffer configuration that passed a five-minute native recording with zero detected steady-window gaps or observer/bag mismatches. Explicitly select it with:

```bash
CAMERA_TRANSPORT=large_shm ROS_DOMAIN_ID=99 scripts/start_demonstration_recording.sh
```

This chooses64MiB SHM buffers and loopback-only UDP in XML; it makes no global sysctl or robot-control change. Default startup remains unchanged. See [five-minute results](../camera_bringup/STABILITY_TEST_RESULTS.md) for measured rates, recovery tests, startup warnings and remaining hardware limits. Validation of a five-minute raw bag took about176seconds; wait for completion before pressing Menu for the next episode.

## Topics and QoS

The opt-in profile includes both camera RGB/aligned-depth images, corresponding CameraInfo, color/depth metadata, timing clock/health topics, active arm joint states/TrackingSample/command/pose/status topics, Quest left/right states, gripper events and TF. Actual types/counts and missing topics are written in each manifest. No dedicated physical gripper command or continuous measured-state contract was discovered; raw Quest grip/buttons and existing events are preserved. Never infer a measured gripper position from a requested command.

Sensor subscriptions use best-effort volatile QoS; `/tf_static` and existing transient-local gripper events use reliable transient-local. The recorder retains requested topics even if absent at start and leaves discovery enabled. One missing camera does not prevent recording other streams. Storage plugin availability is checked before start; no silent format fallback. Existing legacy profiles keep sqlite3 and conservative clock provenance defaults.

Health messages contain disk space, file growth and independent subscriber counts. Humble has no portable recorder lost-message counter here; `storage_drop_count=null` means unknown. Common-window image identity comparison can demonstrate recorder-path misses. Independent subscribers can also lose data: shared gaps do not prove a camera publication gap. Logs and partial raw files remain preserved on failures.

## Inspect and validate

```bash
source /opt/ros/humble/setup.bash
source install/demonstration/local_setup.bash
ros2 bag info log/demonstrations/EPISODE
python3 -m recording.validate_episode log/demonstrations/EPISODE
ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 ros2 topic echo /quest/recording_status
```

Each episode has `episode_manifest.json`, `timing_records.jsonl` and `alignment_provisional.json`, plus adjacent recorder/validation logs and QoS/storage configuration. Validation reads raw MCAP/sqlite3, parses image layouts, decodes representative pixels/depth, verifies sparse observer payload fingerprints, counts messages, retains metadata and invokes existing Phase3 alignment. Revalidation overwrites derived JSON, never raw bag data. Interrupted MCAP can retain only a readable prefix; inspect status and counts even if `readable=true`.

The manifest marks fixed mock pose, missing real robot/gripper measurements, uncalibrated extrinsics, original clocks and measured host clock models. `clock_provenance: same_host` is an explicit assertion for this native composition; independent hosts/simulation clocks require separate verification. Host ROS/monotonic bracket fits are not camera exposure calibration. Unknown capture uncertainty remains null; a provisional50ms join does not establish exposure synchronization.

Enabled command intent can be associated with recorded Quest samples using session, sequence and reference-space generation, with original acquisition/location/host-source timestamps intact. Existing Phase3 `action_t` requires actual recorded controller receipt/output observations; mock mapping alone leaves it masked. Do not invent execution timestamps. Offline source-intent evidence for the live test is under `log/phase5-validation/enabled_actions_20261026`.

## Isolated Docker deployment

`docker/recording.Dockerfile` derives from the pinned camera image, adds MCAP and builds the same six packages. It installs no additional RealSense SDK and does not alter the working Panda image. Build from this worktree:

```bash
docker build -f docker/recording.Dockerfile -t franka-demonstration-recording:phase5 .
```

With no native pipeline already using the cameras/45454, configure the existing authorized ADB server:

```bash
docker exec docker-quest-adb-server-1 adb reverse tcp:45454 tcp:45454
docker exec docker-quest-adb-server-1 adb shell am start -n org.franka.questteleop/android.app.NativeActivity
mkdir -p log/demonstrations
docker run --rm --name franka-demonstration-recording \
 --network host --ipc host \
 --device-cgroup-rule='c 189:* rmw' \
 -v /dev/bus/usb:/dev/bus/usb -v /run/udev:/run/udev:ro \
 -v "$PWD/log/demonstrations:/recordings" \
 -e ROS_DOMAIN_ID=99 -e ROS_LOCALHOST_ONLY=1 \
 franka-demonstration-recording:phase5
```

The container command is the same camera/mock Menu launch. Record all three sources inside this container to avoid relying on unvalidated native/container DDS visibility. Afterwards remove forwarding only if you added it: `docker exec docker-quest-adb-server-1 adb reverse --remove tcp:45454`. Container image build/readback validation and live native tests are reported separately; a successful image build does not prove live container USB/DDS performance.

## Automated checks

Run packages separately to avoid the installed launch-testing plugin's duplicate module-name collision:

```bash
export ROS_DOMAIN_ID=138 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=src/recording:src/quest_teleop:src/camera_bringup:src/demonstration_timing:$PYTHONPATH
for package in recording demonstration_timing camera_bringup quest_teleop; do
 python3 -m pytest -p no:cacheprovider -q src/$package/test
done
python3 -m pytest -p no:cacheprovider -q src/teleop_bringup/test/test_robot_free_launch.py
```

See [PHASE5_REPORT.md](PHASE5_REPORT.md) for acceptance limits before real demonstration collection.
