# Panda / Quest 3: setup, mock checks and first hardware test

This workspace targets **legacy Panda (FER), Ubuntu 22.04 and ROS 2 Humble**.
It includes the original direct-libfranka `panda_bringup` controller and
`panda_demos` publisher, plus Quest USB input, pose mapping, single/dual
impedance control, separate Franka Hand workers, homing, coupled CBF-QP and
Menu-triggered recording. This is not an FR3 setup guide.

The implementation handoff passed 81 individual robot-free cases; see
[IMPLEMENTATION_PROGRESS.md](docs/IMPLEMENTATION_PROGRESS.md). Physical
compatibility, clearance and realtime timing remain deployment commissioning
checks. Follow [docs/AGENTS.md](docs/AGENTS.md) for further development.

The operator accepted Milestone 1 on 2026-10-12 after successful single-arm
teleoperation with CBF disabled. The next work is single-arm asynchronous CBF-QP
and controller tracking/smoothness comparisons; see the
[Milestone 2 plan](docs/MILESTONE_2.md).

Package responsibilities, standalone launches and migration instructions are in
[PACKAGE_STRUCTURE.md](docs/PACKAGE_STRUCTURE.md). The combined entry point is
`ros2 launch teleop_bringup teleoperation.launch.py`.

## Reading order, terminals and evidence

1. [New-PC installation and build](#1-new-pc-installation-and-build).
2. [Quest USB and headset checks](#2-quest-usb-and-headset-checks).
3. [Mock acceptance](#3-mock-acceptance).
4. [Physical preparation and verification](#4-physical-preparation-and-verification).
5. [Legacy Panda connection test](#15-legacy-panda-controller-and-demo-connection-test),
   then [first single/dual teleoperation test](#5-first-real-fci-teleoperation-test).

Section 1.5 completes the first-time setup, but **requires section 4 first**.
`panda_bringup` opens a real FCI session; its fake-hardware visualization does
not make its controller a mock.

| Name | Where | Purpose |
| --- | --- | --- |
| **H1** | PC host, `~/franka_teleoperation` | Installation, Docker, ADB/APK, editing and archiving |
| **C1** | ROS container, `/ws` | Build, then the single active controller launch |
| **C2** | Another shell in the same container, `/ws` | Topic checks, services, legacy demo |
| **C3** | Another shell in the same container, `/ws` | Additional observations and bags |
| **Headset** | Quest 3 | Display, buttons and deliberate input gestures |
| **Operator** | Beside the robot with its manufacturer stop accessible | Desk activation, physical observations and stopping |

An ongoing launch/echo occupies its terminal until stopped. Run commands within
one block sequentially. All paths below use the clone directory
`~/franka_teleoperation`, regardless of your username. It is mounted at `/ws`:
`/ws/log/commissioning/current` is the host's
`~/franka_teleoperation/log/commissioning/current`.

Each stage names evidence files and successful signs. Record **PASS / FAIL /
NOT RUN**, date, operator, measurements and file/bag paths in `verification.md`.
Missing checks are not passes. `log/` is Git-ignored; archive evidence separately.

## 1. New-PC installation and build

### 1.1 Host prerequisites and Docker

Use native Linux Docker Engine. Docker shares the host kernel; the image does
not supply PREEMPT_RT. Docker Desktop is not the FCI deployment path. Have a
wired robot-network interface, USB data cable, Quest developer mode and access
to the robot's manuals/Desk. Host ROS/Android SDK installations are unnecessary.

**H1, host:**

```bash
cat /etc/os-release
uname -m
sudo apt-get update
sudo apt-get install -y git curl ca-certificates nano python3 python3-yaml \
  iproute2 iputils-ping ethtool usbutils rt-tests
```

For a fresh Ubuntu installation without Docker, follow the official
[Docker apt installation](https://docs.docker.com/engine/install/ubuntu/):

```bash
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "$VERSION_CODENAME")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
```

Log out/in to apply membership. Docker group membership grants administrative
access. If Docker already works, retain it; resolve conflicting packages using
the linked Docker instructions rather than installing a second daemon.

**H1, after login:**

```bash
docker version
docker compose version
docker run --rm hello-world
```

**Pass:** Ubuntu 22.04, working Docker daemon/Compose v2, `Hello from Docker!`.
Save versions in `host.txt` after creating the directory below.

### 1.2 Clone the implementation and create the record

**H1:**

```bash
cd ~
git clone https://github.com/dp-tumLi/panda_ros2_to_go.git franka_teleoperation
cd ~/franka_teleoperation
git status --short
git rev-parse HEAD
test -f docs/AGENTS.md
test -f src/teleop_bringup/launch/teleoperation.launch.py
test -f android/quest_teleop/app/src/main/cpp/openxr_program.cpp
```

Use the branch/revision supplied by the repository owner containing this
implementation. **Local uncommitted/untracked files do not transfer through
`git clone`.** If source checks fail, obtain that branch or the complete workspace
snapshot before continuing; do not substitute a current upstream FR3 driver.

**H1:** create this run folder once. For another session, archive/rename
`current` first so previous evidence is preserved.

```bash
mkdir -p log/commissioning/current
git rev-parse HEAD > log/commissioning/current/revision.txt
git status --short > log/commissioning/current/worktree.txt
cat /etc/os-release > log/commissioning/current/host.txt
uname -a >> log/commissioning/current/host.txt
docker version >> log/commissioning/current/host.txt
docker compose version >> log/commissioning/current/host.txt
cat > log/commissioning/current/verification.md <<'EOF'
# Commissioning record
Date / operator / observer:
Panda model / serial(s), Desk system / robot / gripper server versions:
Git revision / local changes / APK hash:

| Check | PASS / FAIL / NOT RUN | Measurements / evidence filenames |
| --- | --- | --- |
| Host / clone / dependencies / build | NOT RUN | |
| Quest USB / display / buttons / reconnect | NOT RUN | |
| Mock mapping / watchdog / impedance / CBF | NOT RUN | |
| Mock Hand / home / Menu / single and dual | NOT RUN | |
| Firmware and pinned library compatibility | NOT RUN | |
| PREEMPT_RT / FIFO / memory locking / network | NOT RUN | |
| Base / table / tool / payload / geometry survey | NOT RUN | |
| Reviewed gains / scales / test bounds / home path | NOT RUN | |
| External stop and deliberate recovery | NOT RUN | |
| Legacy FCI hold and zero-radius demo | NOT RUN | |
| Single-arm measured hold and timing | NOT RUN | |
| Single-arm small motion and loss/recovery | NOT RUN | |
| Physical Hand and homing | NOT RUN | |
| Dual transforms / timing / independent control | NOT RUN | |

Approved displacement / rotation / speed / duration / stop conditions:
Timing method / load / CPU placement / worst cases / trace files:
Physical minimum clearances / survey uncertainty / padding:
Tool and payload configuration / evidence:
Hand object width / tolerances / force / speed / calibration maximum width:
Homing swept-path review / cancellation stop distance and outcome:
Finalized bag paths:
Outstanding failures / next permitted stage:
EOF
```

**Pass:** required source exists, revision/local changes recorded. Update the
record after every stage in **H1**:

```bash
nano log/commissioning/current/verification.md
```

### 1.3 Build the existing ROS environment

**H1:**

```bash
cd ~/franka_teleoperation
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml run --rm --name franka-guide-ros panda-ros2
```

That terminal is now **C1**. Keep its shell open. If you exit it, repeat
`compose run` before using `docker exec` later.

**C1, container `/ws`:**

```bash
cd /ws
source /opt/ros/humble/setup.bash
set -o pipefail
bash scripts/fetch_teleop_dependencies.sh
apt-get update
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y \
  --skip-keys ament_cmake_clang_format
colcon build --base-paths src --symlink-install --packages-select libfranka \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
  2>&1 | tee /ws/log/commissioning/current/build-libfranka.txt
source install/setup.bash
colcon build --base-paths src --symlink-install \
  --packages-skip quest_teleop_interfaces quest_teleop recording safety_filter teleop_controllers teleop_gripper teleop_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
  2>&1 | tee /ws/log/commissioning/current/build-workspace.txt
source install/setup.bash
colcon build --base-paths src --symlink-install \
  --packages-select quest_teleop_interfaces quest_teleop recording safety_filter teleop_controllers teleop_gripper teleop_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  2>&1 | tee /ws/log/commissioning/current/build-teleoperation.txt
source install/setup.bash
ros2 pkg executables panda_bringup
ros2 pkg executables panda_demos
ros2 pkg executables teleop_controllers
ros2 pkg executables teleop_gripper
ros2 pkg executables recording
ros2 pkg executables quest_teleop
git -C src/third_party/libfranka_0_9_2 rev-parse HEAD \
  > /ws/log/commissioning/current/dependencies.txt
git -C src/third_party/libfranka_0_9_2 submodule status --recursive \
  >> /ws/log/commissioning/current/dependencies.txt
git -C src/third_party/franka_arm_ros2 rev-parse HEAD \
  >> /ws/log/commissioning/current/dependencies.txt
git -C src/third_party/osqp_0_6_3 rev-parse HEAD \
  >> /ws/log/commissioning/current/dependencies.txt
```

The helper imports `third_party.repos`, initializes libfranka/OSQP submodules and
excludes standalone OSQP colcon discovery. The controller builds its pinned
solver. No workspace patch is required. The skipped rosdep key is legacy lint
only. Legacy packages are built with tests off because their lint configuration
requires that unavailable key; the new packages have acceptance tests enabled.
LCAS `lcas_0.9.2` and the pinned ROS driver are intentional, not interchangeable
with current upstream libfranka. Verify physical protocol compatibility in 4.1.

ROS builds discover packages under `src` explicitly. The tracked
`android/COLCON_IGNORE` also excludes the Android tree from default colcon
discovery. If a ROS build starts `quest_teleop_android` and fails to find
`/sources/android/native_app_glue/android_native_app_glue.c`, use the build
commands above with `--base-paths src`. The APK requires the separate Gradle/NDK
container in section 2.1; the ROS container does not build it.

`apt-get update` is required inside each fresh container: the Docker image
removes `/var/lib/apt/lists/*` after installing its preinstalled packages.
`rosdep update` refreshes ROS dependency mappings, not APT's package indexes.
If `rosdep install` reports `Unable to locate package
ros-humble-warehouse-ros-sqlite`, run `apt-get update` successfully in **C1**,
then repeat the same `rosdep install` command. This dependency is declared by
the bundled `franka_moveit_config`; do not add it to `--skip-keys`.
If it still fails, check `cat /etc/os-release`, `dpkg --print-architecture`
and `apt-cache policy ros-humble-warehouse-ros-sqlite`. Use Ubuntu 22.04
(Jammy) and ensure `apt-get update` fetched the configured ROS 2 repository
without repository, signature or network errors before continuing.

**Pass:** no failed build packages. Executable lists include
`cartesian_impedance_controller` in both controller packages,
`cartesian_circle_publisher`, `franka_hand_worker`, and `quest_usb_bridge`.
Retain build/dependency files; record the result. Build once here, then rebuild
only after relevant source changes. Documentation edits do not require rebuilding.

### 1.4 Open C2 and C3

In **two additional host terminals**, run this block separately. They become
**C2** and **C3**, sharing C1's container, files and host network:

```bash
docker exec -it franka-guide-ros bash
cd /ws
source /ws/install/setup.bash
set -o pipefail
```

`install/setup.bash` is the standard workspace entry point and also sources
ROS Humble. The temporary `install/reorganized` overlay is retired; open a new
shell after migration so an earlier overlay does not remain in its environment.

Every new container shell must source the workspace. Use one ROS domain
consistently; these commands use the default domain. Do not run old mappers,
separate mock pose publishers, duplicate bridges or competing robot owners.

### 1.5 Legacy Panda controller and demo connection test

**Physical test: complete section 4 first.** Test one Panda at a time. Stop
new teleoperation launches first. The legacy controller has no Quest deadman,
source watchdog or new CBF. Its callback does not implement the new controller's
controlled Ctrl+C finish: end legacy FCI with the manufacturer stop procedure.
Stopping a demo publisher leaves its last target held.

**C2:** derive legacy gains/IP from the reviewed files created in 4.3, preserving
source configuration:

```bash
python3 - <<'PY'
from pathlib import Path
import yaml
p = Path('/ws/log/commissioning/current')
a = yaml.safe_load((p/'arms_single.yaml').read_text())['arms']['panda_1']
r = yaml.safe_load((p/'runtime_hardware.yaml').read_text())
c = {k: r['controller'][k] for k in (
    'lambda_translation','lambda_rotation','gain_translation','gain_rotation')}
c.update(robot_ip=a['robot_ip'], operator_confirm=True)
(p/'legacy_controller.yaml').write_text(yaml.safe_dump(
    {'panda_cartesian_impedance_controller': {'ros__parameters': c}}))
PY
```

**Operator:** confirm approved starting pose/clearance, observer and stop;
unlock/activate FCI in Desk according to the robot manual. 

**C1:**
```bash
ros2 run panda_bringup cartesian_impedance_controller --ros-args \
  --params-file /ws/log/commissioning/current/legacy_controller.yaml \
  2>&1 | tee /ws/log/commissioning/current/legacy-hold.txt
```

**C2:**
```bash
ros2 topic echo /panda_cartesian_impedance_controller/initial_pose \
  --qos-durability transient_local --once \
  > /ws/log/commissioning/current/legacy-initial-pose.yaml
ros2 topic echo /franka_robot_state_broadcaster/robot_state --once \
  > /ws/log/commissioning/current/legacy-state.yaml
```

**Pass:** actual pose/state arrives, reviewed gains reported without libfranka
errors, arm holds measured start pose without unexpected motion. Ping alone is
not an FCI test. Next test the `panda_demos` publisher with **zero radius**.

**C2:**

```bash
ros2 run panda_demos cartesian_circle_publisher --ros-args \
  -p radius:=0.0 -p approach_seconds:=6.0 -p circle_seconds:=20.0 \
  2>&1 | tee /ws/log/commissioning/current/legacy-demo.txt
```

**C3:**

```bash
ros2 topic echo /panda_cartesian_impedance_controller/target_pose --once \
  > /ws/log/commissioning/current/legacy-zero-radius-target.yaml
ros2 topic hz /panda_cartesian_impedance_controller/target_pose
```

**Pass:** approximately 100 Hz targets equal the initial pose/orientation;
physical arm still holds. Stop the C3 observer and C2 demo with Ctrl+C. Use the
manufacturer stop to end legacy FCI, wait for control to end, then stop any
remaining C1 process. Record hold/stop/recovery results. The legacy executable
can return zero after catching an exception: read `legacy-hold.txt`, not only
its exit code.

Optional motion requires a separately reviewed radius, speed, approach, swept
path and stop plan. **C2:** save the approved parameters (metres/seconds):

```bash
python3 - <<'PY'
import math, yaml
p = {k: float(input(k + ': ')) for k in
     ('radius','approach_seconds','circle_seconds')}
assert all(math.isfinite(v) and v > 0 for v in p.values())
with open('/ws/log/commissioning/current/legacy_demo.yaml', 'w') as f:
    yaml.safe_dump({'panda_circle_trajectory': {'ros__parameters': p}}, f)
PY
```

Restart the legacy owner in **C1**, then **C2**:

```bash
ros2 run panda_demos cartesian_circle_publisher --ros-args \
  --params-file /ws/log/commissioning/current/legacy_demo.yaml
```

**Pass:** measured arm follows the approved small trajectory with reviewed
tracking error, no instability or robot faults. Stop as above; record actual
error and physical behavior. Steady target speed is `2*pi*radius/circle_seconds`;
review the initial approach too. **Do not use the default demo launch for first
motion:** `ros2 launch panda_demos circle_trajectory.launch.py` gives a 0.10 m
circle and exposes no smaller-radius launch argument.

For reference, the existing bringup launch is available in **C1**, only after
reviewing its shipped gains:

```bash
ros2 launch panda_bringup cartesian_impedance_controller.launch.py \
  robot_ip:=REPLACE_WITH_ROBOT_IP operator_confirm:=true use_rviz:=false
```

It loads `src/panda_bringup/config/cartesian_impedance_controller.yaml`:
translation `Lambda=50, Kr=30` (1500 N/m), rotation `Lambda=40, Kr=4.5`
(180 Nm/rad). The executable/new runtime reference `Kr=20/2.5` differs. None
of these values is an automatic commissioning approval.

## 2. Quest USB and headset checks

No robot connection is needed. Stop robot controllers. Enable Quest developer
mode through Meta's device setup, connect a **USB data cable**, wear/unlock the
headset and authorize USB debugging for this PC. Wireless ADB is not the transport.

### 2.1 Build, authorize and install

**H1, host repository root:**

```bash
docker compose -f docker/android-compose.yaml build
docker compose -f docker/android-compose.yaml run --rm android-build gradle assembleDebug
sha256sum android/quest_teleop/app/build/outputs/apk/debug/app-debug.apk \
  > log/commissioning/current/apk.sha256
docker compose -f docker/android-compose.yaml up -d quest-adb-server
docker compose -f docker/android-compose.yaml run --rm quest-adb adb devices -l \
  | tee log/commissioning/current/adb-devices.txt
```

**Pass:** one intended USB headset listed as `device`, not `unauthorized` or
`offline`. Disconnect extra Android devices. For unauthorized, accept the
headset prompt and repeat. For no device, use **H1** `lsusb` and check cable/data
capability/developer mode. Avoid a competing host ADB server; use this compose service.

**H1:**

```bash
bash scripts/quest_adb_reverse.sh 45454
bash scripts/quest_install.sh
docker compose -f docker/android-compose.yaml run --rm quest-adb adb reverse --list \
  | tee log/commissioning/current/adb-reverse.txt
```

**Pass:** app starts and reverse mapping includes TCP 45454. Quest localhost is
forwarded over USB to the PC's bridge. Debug signing persists in the
`android-signing` volume. Reuse the installed APK later unless Android code changes.

For `INSTALL_FAILED_UPDATE_INCOMPATIBLE`, first confirm old app data may be
lost, then **H1** uninstall/reinstall:

```bash
docker compose -f docker/android-compose.yaml run --rm quest-adb \
  adb uninstall org.franka.questteleop
bash scripts/quest_install.sh
```

### 2.2 Bridge and raw controller states

**C1:**

```bash
ros2 launch quest_teleop quest_usb_bridge.launch.py listen_port:=45454 \
  2>&1 | tee /ws/log/commissioning/current/quest-bridge.txt
```

**C2:** save samples and watch right input:

```bash
ros2 topic echo /quest_usb_bridge/right/controller_state --once \
  > /ws/log/commissioning/current/quest-right.yaml
ros2 topic echo /quest_usb_bridge/left/controller_state --once \
  > /ws/log/commissioning/current/quest-left.yaml
ros2 topic echo /quest_usb_bridge/right/controller_state
```

Ctrl+C that echo, then **C2**:

```bash
ros2 topic echo /quest_usb_bridge/left/controller_state
```

**Pass while focused/tracking:** correct hand ID; increasing sequence/source
clock (per-hand sequence gaps are normal); `reference_space: LOCAL`; connection,
focus, pose action, valid/tracked pose, clock sync and source time true. Finite
native twist is small at rest and responds to motion. Clock uncertainty must
meet the downstream 10 ms gate; source freshness must meet 100 ms.

Current ROS input has a `buttons` array: `name`, `pressed`, `active`; right
`a/b`, left `x/y/menu`, empty on disconnect. There should be no opposite-hand
scalar placeholders such as right `button_x_active`. Activity is action
availability, not a duplicate pressed state. If the old schema appears, stop
stale publishers, source this overlay and inspect **C2**:

```bash
ros2 interface show quest_teleop_interfaces/msg/QuestControllerState
ros2 topic info /quest_usb_bridge/right/controller_state --verbose
```

### 2.3 Display, flicker, focus and USB recovery

**Headset actions, C2 watching the matching hand:**

| Check | Action | Successful sign; record in `verification.md` |
| --- | --- | --- |
| Passthrough | Launch app and look around | Real surroundings visible behind stable HUD, no black/flickering composition |
| HUD / aim | Move both controllers | Readable pose/velocity/input diagnostics and aim rays follow the correct hand |
| Recenter | Quest system recenter gesture | Panels remain usable; reference generation changes; later control requires fresh release/press |
| Button flicker | Hold right A/B and left X/Y/Menu separately for several seconds, then release | Correct `pressed` stays true while held and false on release; no spurious toggles; `active` stays available while focused |
| Analog input | Slowly squeeze/release each trigger/grip | Correct hand values move smoothly between approximately 0 and 1 |
| Focus | Open system UI/leave app then return | Focus changes; unfocused input is not treated as control-valid |
| USB | Unplug, wait, replug and restore reverse | Loss produces invalid/disconnected input; fresh samples resume without rebuild/reinstall |

**H1, after replug:**

```bash
docker compose -f docker/android-compose.yaml run --rm quest-adb adb devices -l
bash scripts/quest_adb_reverse.sh 45454
docker compose -f docker/android-compose.yaml run --rm quest-adb \
  adb shell am start -n org.franka.questteleop/android.app.NativeActivity
docker compose -f docker/android-compose.yaml run --rm quest-adb \
  adb logcat -d -s QuestTeleop:V > log/commissioning/current/quest-logcat.txt
```

Review passthrough setup and repeated `xrEndFrame` errors in `quest-logcat.txt`.
An opaque fallback is supported, but does not pass the passthrough test. The
HUD trigger latch is local input state, not proof of robot arming. Record all
observations. Details: [VR_APP_PORT.md](docs/VR_APP_PORT.md).

## 3. Mock acceptance

All commands here use **`backend:=mock`**, opening no FCI. The fixture has fixed
joint/Cartesian state, synthetic inertia and zero coriolis; it is not a physics
simulator or physical clearance/timing certificate.

Stop the C1 bridge-only launch with Ctrl+C; the integrated launch will own the
only bridge. Stop older controllers, Hand workers and `robot_free.launch.py`
mappers. Do not add a separate mock measured-pose publisher.

### 3.1 Mock configuration and startup

**C2:** create a mock-only object configuration:

```bash
python3 - <<'PY'
from pathlib import Path
import yaml
r = yaml.safe_load(Path('/ws/src/teleop_bringup/config/teleoperation_runtime.yaml').read_text())
r['hand'].update(grasp_width_m=0.006, epsilon_inner_m=0.001, epsilon_outer_m=0.002)
Path('/ws/log/commissioning/current/mock_runtime.yaml').write_text(yaml.safe_dump(r))
PY
```

These dimensions are mock examples, not real object settings. **C1:**

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  backend:=mock mode:=single start_bridge:=true \
  runtime_config:=/ws/log/commissioning/current/mock_runtime.yaml \
  recording_output_root:=/ws/log/commissioning/current/mock_bags \
  2>&1 | tee /ws/log/commissioning/current/mock-single-right.txt
```

Controller, mapper, separate Hand worker, left Menu recorder and bridge start.
No automatic home/grasp/recording occurs. With `start_hand:=true`, the Hand
worker automatically calibrates once at startup. **C2:**

```bash
ros2 node list > /ws/log/commissioning/current/mock-nodes.txt
ros2 topic list > /ws/log/commissioning/current/mock-topics.txt
ros2 topic echo /panda_1/controller_status --once \
  > /ws/log/commissioning/current/mock-controller.yaml
ros2 topic echo /panda_1/cbf_status --once \
  > /ws/log/commissioning/current/mock-cbf.yaml
ros2 topic echo /panda_1/teleop_anchor_pose --once \
  > /ws/log/commissioning/current/mock-anchor.yaml
```

**Pass:** `hardware_backend: false`, arm disarmed, measured anchor present,
solver successful, no fallback/fatal fault or growing failure count. Record startup.

### 3.2 Mapping, impedance commands and input faults

**C2:**

```bash
ros2 topic echo /panda_1/cartesian_teleop_command
```

**Headset:** fully release right trigger, squeeze above 0.75 and hold. Move the
right hand along each local axis separately; rotate about each axis; release
below 0.25. Record observed axes/signs and target changes in `verification.md`.
The default identity basis is a fixture, not a surveyed physical mapping.

**Pass:** release disables; first engagement anchors at measured pose with
zero initial feedforward; later motion changes target pose/finite desired twist.
Left trigger does not enable this right-assigned arm. `input_valid` and
`trigger_released` distinguish deliberate release from a fault.

Stop C2's echo. Observe the actual consumer and filter separately:

```bash
# C2
ros2 topic echo /panda_1/controller_status
```

```bash
# C3
ros2 topic echo /panda_1/cbf_status
```

Repeat small hand motions. **Pass:** consumer `motion_enabled` follows valid
engagement, torque vectors stay finite, queue overflow does not increase,
limiting/intervention is reported, solver healthy. The nominal impedance law
is `r = J*dq - Vd + Lambda*e`, `tau = -J^T*Kr*r + coriolis`, then CBF filtering.
Fixed mock state does not follow targets or demonstrate physical impedance feel.

While engaged, unplug USB, lose tracking or leave focus. **Pass:** input loss
or timeout disables motion and removes stale target/feedforward. Recover using
section 2's H1 reverse commands: holding trigger must not resume; release/press
is required. Recenter also requires fresh reengagement. Record each fault/recovery.

### 3.3 Mock Hand calibration and toggle

The launched Hand worker calibrates automatically. The service below is only
for deliberate recalibration after startup or a calibration fault.

Stop C2/C3 echoes. **C3:**

```bash
ros2 topic echo /panda_1/gripper_events
```

**C2:**

```bash
ros2 service call /panda_1/calibrate_gripper \
  quest_teleop_interfaces/srv/CalibrateGripper \
  '{empty_hand_confirm: true, operator_confirm: true}' \
  | tee /ws/log/commissioning/current/mock-hand-calibration.txt
```

**Pass:** request accepted, then completion events show `calibrated: true`,
`busy: false` and finite positive maximum width. Acceptance alone is asynchronous,
not completion. Release grip, squeeze above 0.75, release below 0.25 between
presses. First deliberate press opens, next grasps, next opens. Holding does
not repeat; release alone does not open; busy presses are discarded, not queued.
Check operation counters. Input loss does not stop an already issued operation.
Record outcomes, then stop the echo.

### 3.4 Mock home and cancellation

**C2:**

```bash
ros2 topic echo /panda_1/homing_status
```

**Headset:** trigger released; hold **right A at least one second**, then
release. **Pass:** one request ID increment, `homing` then `succeeded`, control
still disarmed. Short/startup-held gestures do not home. Mock joints already
match home, so completion is quick. Fixed Panda home is:

```text
q_home [rad] = [0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4]
```

If still active, trigger press or input/tracking loss cancels: **pass** is
`stopping` then `cancelled`/measured hold, `stop_failed: false`, no automatic
resume and fresh release/press required. A human may miss the short fixed-home
window; automated cases below cover cancellation. Record what was observed.

### 3.5 Menu recording

Stop C2 echo. **C2:**

```bash
ros2 topic echo /quest/recording_status
```

**Headset:** release left Menu, press once, hold, release, wait over 350 ms,
press again. **Pass:** one episode starts; holding does not repeat; second press
finalizes, status `idle` / `bag finalized`. Menu works in single-right without
left-arm mapping. Valid button/focus/connection/source-clock metadata is required,
but left pose tracking is not. A started bag survives input loss; after reconnect,
a fresh release/press stops it. Normal launch shutdown also finalizes recording.

**C3:** use the exact path reported in recording status:

```bash
ros2 bag info /ws/log/commissioning/current/mock_bags/REPLACE_WITH_EPISODE \
  | tee /ws/log/commissioning/current/mock-bag-info.txt
```

**Pass:** finalized `metadata.yaml`/sqlite3 file, nonzero messages, active-arm
and left Menu input topics, no inactive arm topics. New episodes get unique
paths. Only selected topics published at start are included; absent topics
are omitted for that episode. Record bag paths. Failures appear in status and
adjacent `.recorder.log`.

Selection is configurable through `src/recording/config/recording.yaml` or
launch overrides. Example **C1**, after stopping the previous launch:

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  backend:=mock mode:=single start_bridge:=true \
  recording_topics:='["/panda_1/controller_status","/panda_1/cbf_status","/panda_1/robot_state"]' \
  recording_output_root:=/ws/log/commissioning/current/mock_bags
```

### 3.6 Single-left and dual routing

Ctrl+C C1 and wait for exit before each new launch. Stop observers as needed.
**C1, single-left:**

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  backend:=mock mode:=single single_controller:=left start_bridge:=true \
  runtime_config:=/ws/log/commissioning/current/mock_runtime.yaml
```

Observe `/panda_1` as above. **Pass:** only left trigger/grip controls that arm;
left **X** held one second then released homes it; right A/trigger does not.
Stop it. **C1, dual:**

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  backend:=mock mode:=dual start_bridge:=true \
  arms_config:=/ws/src/quest_teleop/config/arms_two_mock.yaml \
  runtime_config:=/ws/log/commissioning/current/mock_runtime.yaml \
  recording_output_root:=/ws/log/commissioning/current/mock_dual_bags \
  2>&1 | tee /ws/log/commissioning/current/mock-dual.txt
```

```bash
# C2
ros2 topic echo /panda_left/controller_status
```

```bash
# C3
ros2 topic echo /panda_right/controller_status
```

Substitute `homing_status`, `gripper_events`, `cbf_status` or
`cartesian_teleop_command` to observe each behavior. **Pass:** left/right
trigger enables only its arm; X homes left, A homes right independently;
both filter statuses healthy; dual bag contains both active arms.

Both started Hand workers calibrate automatically. For deliberate
recalibration, stop C2 echo, then calibrate both:

```bash
# C2
ros2 service call /panda_left/calibrate_gripper \
  quest_teleop_interfaces/srv/CalibrateGripper \
  '{empty_hand_confirm: true, operator_confirm: true}'
ros2 service call /panda_right/calibrate_gripper \
  quest_teleop_interfaces/srv/CalibrateGripper \
  '{empty_hand_confirm: true, operator_confirm: true}'
```

One process owns two arms and one coupled solver; two independent single-arm
hardware controllers cannot substitute for dual mode. Record routing results.

### 3.7 Automated acceptance and intentional mock CBF failure

After the initial major build, run acceptance once. Stop C1 launches and C2/C3
observers first to avoid topic/port conflicts. **C1:**

```bash
colcon test --packages-select quest_teleop recording safety_filter teleop_controllers teleop_gripper teleop_bringup \
  --event-handlers console_direct+ \
  2>&1 | tee /ws/log/commissioning/current/mock-tests.txt
colcon test-result --all --verbose \
  2>&1 | tee /ws/log/commissioning/current/mock-test-results.txt
```

**Pass:** no failures/errors in these packages. Cases cover clock/order/freshness,
release gating, torque bounds, Hand/home cancellation, invalid/infeasible CBF,
coupled stop/timing, actual launches and bags. Colcon counts wrappers too;
aggregate counts need not equal the handoff's 81 individual cases.

Optional explicit failure check, **C1, mock only**, no other owner:

```bash
ros2 run teleop_controllers cartesian_impedance_controller --ros-args \
  -r __ns:=/cbf_failure_check -p backend:=mock \
  -p cbf.table_height:=2.0 -p cbf.maximum_failures:=3 \
  2>&1 | tee /ws/log/commissioning/current/mock-cbf-failure.txt
```

**Pass:** intentional infeasibility produces fallback/repeated-failure stop
and termination without restart. Never use that table setting on hardware.
Resolve mock failures before advancing. More: [TELEOPERATION_RUNTIME.md](docs/TELEOPERATION_RUNTIME.md).

## 4. Physical preparation and verification

Complete this before any physical launch, including legacy section 1.5.
Commissioning numbers require measured installation data and qualified review;
shipped gains/mock dimensions are not approvals. Start empty-handed with one
Panda. An observer must be able to stop it while the headset wearer cannot
see the terminals. Passing setup does not approve all later motion stages.

### 4.1 Compatibility, network, Desk and external stop

**Operator:** identify FER/Panda, serial, Desk system/robot/gripper server
versions, FCI availability and robot health. Compare the actual protocol with
the pinned LCAS fork. The target legacy system is 4.2.2; an image name does not
prove compatibility. Use vendor [compatibility documentation](https://frankarobotics.github.io/docs/compatibility.html)
and the fork's definitions. **C2:**

```bash
grep -R -n -E 'VERSION|version' src/third_party/libfranka_0_9_2/common/include \
  > /ws/log/commissioning/current/library-protocol.txt
```

Save Desk screenshots/notes and the comparison in `verification.md`. Verify
actual tool/end-effector and payload configuration independently; these runtime
YAML files do not automatically configure the robot's physical load. Do not
upgrade to a current FR3/upstream driver as an unreviewed workaround.

**H1:** identify the wired interface and network plan:

```bash
ip -br link
ip -br address
ip route
```

Use actual lab-assigned addresses, not guessed example IPs. If the interface
needs an address, the following prompts add one without changing other interfaces:

```bash
read -r -p 'Robot Ethernet interface (e.g. enp3s0): ' robot_iface
read -r -p 'Approved PC IPv4/CIDR: ' robot_pc_cidr
sudo ip link set dev "$robot_iface" up
sudo ip address add "$robot_pc_cidr" dev "$robot_iface"
read -r -p 'Panda FCI IPv4 address: ' robot_fci_ip
ip route get "$robot_fci_ip" | tee log/commissioning/current/robot-route.txt
ping -c 20 "$robot_fci_ip" | tee log/commissioning/current/robot-ping.txt
sudo ethtool "$robot_iface" | tee log/commissioning/current/robot-link.txt
```

Skip `ip address add` if the address already exists. This is temporary; make it
persistent through the host network manager and record the configuration.
**Pass:** intended wired interface/source route, link at vendor-required speed,
no ping loss, Desk reachable. Ping does not certify FCI/1 kHz jitter. For dual,
check both distinct IPs/routes/links and save separate left/right outputs. Follow
[Franka's FCI requirements](https://frankarobotics.github.io/docs/) for the installation.

**Operator:** review and test the manufacturer-defined stop/activation procedure
initially at rest. Record stop owner, observed Desk indication and deliberate
recovery steps. System 4.2.0+ requires **Activate FCI** in the Desk sidebar.
Activation/unlocking and fault recovery are manual operator actions, not a
command that disables robot safety. No automatic recovery is prescribed here.

### 4.2 PREEMPT_RT, FIFO, memory locking and timing

**H1:** if an approved RT kernel already exists, skip installation and verify
below. One supported generic Ubuntu PC option is
[Ubuntu Pro realtime](https://ubuntu.com/pro-client/docs/en/docs/howtoguides/enable_realtime_kernel/):

```bash
sudo apt-get update
sudo apt-get install -y ubuntu-pro-client
sudo pro attach
sudo pro enable realtime-kernel
sudo reboot
```

Follow subscription prompts; do not save credentials in evidence. Select the
supported machine variant; the generic command is for a PC, not Raspberry Pi.
Without Pro, follow the vendor [RT kernel guide](https://frankarobotics.github.io/docs/doc/libfranka/docs/real_time_kernel.html)
and deployment kernel policy. Reboot ends running tests/containers: reopen
C1/C2/C3 through 1.3/1.4 without rebuilding.

**H1, booted into RT:**

```bash
cd ~/franka_teleoperation
uname -a | tee log/commissioning/current/rt-kernel.txt
cat /sys/kernel/realtime | tee -a log/commissioning/current/rt-kernel.txt
sudo groupadd -f realtime
sudo usermod -aG realtime "$USER"
sudo tee /etc/security/limits.d/99-franka-realtime.conf >/dev/null <<'EOF'
@realtime soft rtprio 99
@realtime hard rtprio 99
@realtime soft memlock unlimited
@realtime hard memlock unlimited
EOF
```

Log out/in for limits. **Pass:** PREEMPT_RT running, `/sys/kernel/realtime` is
`1`. Compose already grants `SYS_NICE`, `rtprio: 99`, unlimited memlock. Preserve
that Docker configuration; it still depends on the actual host kernel.

**H1, host terminal with controllers stopped:** inspect and set the CPU governor
for realtime control, following [Franka's frequency-scaling guidance](https://frankarobotics.github.io/docs/troubleshooting.html#disabling-cpu-frequency-scaling).
A realtime kernel alone does not prevent CPU frequency changes.

```bash
cat /sys/devices/system/cpu/cpufreq/policy*/scaling_governor \
  | sort -u | tee log/commissioning/current/cpu-governor-before.txt
printf '%s\n' performance | sudo tee \
  /sys/devices/system/cpu/cpufreq/policy*/scaling_governor >/dev/null
cat /sys/devices/system/cpu/cpufreq/policy*/scaling_governor \
  | sort -u | tee log/commissioning/current/cpu-governor-after.txt
```

**Pass:** every policy reports `performance`. This setting may reset at reboot;
recheck before hardware tests. The direct sysfs command avoids Ubuntu's
`cpupower` wrapper requiring tools for the exact realtime kernel version.

**C2, reopened/sourced container shell:**

```bash
uname -a > /ws/log/commissioning/current/container-rt.txt
cat /sys/kernel/realtime >> /ws/log/commissioning/current/container-rt.txt
ulimit -r >> /ws/log/commissioning/current/container-rt.txt
ulimit -l >> /ws/log/commissioning/current/container-rt.txt
chrt -f 99 /bin/true
python3 - <<'PY' | tee /ws/log/commissioning/current/mlock-check.txt
import ctypes, os
libc = ctypes.CDLL(None, use_errno=True)
if libc.mlockall(1 | 2) != 0:
    raise OSError(ctypes.get_errno(), os.strerror(ctypes.get_errno()))
print('mlockall succeeded')
libc.munlockall()
PY
```

**Pass:** same RT kernel, priority 99, memlock unlimited, FIFO command and
memory-lock check succeed. Actual hardware runtime must also start without
scheduling/mlock failures. Do not bypass libfranka's realtime enforcement.

**H1, no robot controller running:** capture a preliminary scheduler check:

```bash
sudo cyclictest --mlockall --smp --priority=90 --interval=1000 --duration=60s \
  | tee log/commissioning/current/cyclictest.txt
```

Record maximum latency, load and CPU placement. This is not a model/solver/FCI
measurement. Hardware gates are **2 ms model age, 1 ms dual callback timestamp
skew, 0.9 ms callback-to-result, 0.8 ms filter time**. Callback-entry timestamps
do not establish synchronized physical acquisition across robot clocks.

Commissioning requires qualified worst-case per-cycle callback/model/solver and
network/jitter measurements under representative load, with traces and method
stored in `current/` and results in `verification.md`. ROS status is downsampled:
its sampled maxima and `ros2 topic hz` cannot certify every 1 kHz cycle.
There is no bundled one-command timing certification tool. Section 5 begins
with a supervised measured-hold trial; **do not advance to movement until
its timing/stop checks and deployment measurements are reviewed**. Timing faults
are failed checks, not reasons to relax deadlines. See [CBF_PANDA.md](docs/CBF_PANDA.md).

### 4.3 Physical survey and deployment files

**Operator:** survey base/world transforms, horizontal table height in that
world, tool frame/offset/payload, Hand/finger/tool/cable extents and clearance
uncertainty/padding. Review the complete home swept path and test motions.
Coarse capsules are not mesh collision certificates; CBF fallback has no
geometric guarantee. No formal singularity barrier is implemented. Start and
remain away from joint limits, singularities and obstacles; do not test by
approaching a barrier. Altered geometry/tilted tables not representable by the
current model require model work and validation before physical use.

**H1:** create site files without changing source defaults. These templates
are deliberately incomplete until reviewed values are entered:

```bash
python3 - <<'PY'
from pathlib import Path
import yaml
p = Path('log/commissioning/current')
a = yaml.safe_load(Path('src/quest_teleop/config/arms.yaml').read_text())
s = a['arms']['panda_1']
s['robot_ip'] = 'REPLACE_WITH_FCI_IP'
for k in ('reference_to_robot_xyzw','translation_scale','rotation_scale','feedforward_scale'):
    s[k] = None
(p/'arms_single.yaml').write_text(yaml.safe_dump(a))
r = yaml.safe_load(Path('src/teleop_bringup/config/teleoperation_runtime.yaml').read_text())
for k in ('lambda_translation','lambda_rotation','gain_translation','gain_rotation',
          'maximum_torque_rate','homing_speed_factor'):
    r['controller'][k] = None
r['controller'].update(tool_offset_m=None, envelope_margin_m=None,
    homing_stiffness=None, homing_damping=None)
for k in ('speed_mps','grasp_force_n','grasp_width_m','epsilon_inner_m','epsilon_outer_m'):
    r['hand'][k] = None
for k in ('table_height','table_minimum_clearance','self_minimum_clearance','inter_arm_minimum_clearance'):
    r['cbf'][k] = None
r['arm_settings'] = {'panda_1': {'world_from_base_xyz_xyzw': None}}
(p/'runtime_hardware.yaml').write_text(yaml.safe_dump(r))
PY
nano log/commissioning/current/arms_single.yaml
nano log/commissioning/current/runtime_hardware.yaml
nano log/commissioning/current/verification.md
```

Replace every `null`/IP placeholder with surveyed/reviewed values, using numeric
YAML types, not quoted numbers. Use decimal floating-point values such as
`50.0`, including transform/gain arrays; `cbf.maximum_failures` remains an
integer. The screen below normalizes runtime numeric values before ROS loads
them. Record reviewer and rationale.

| File / setting | What must be verified |
| --- | --- |
| `arms_single.yaml`: IP, namespace, base frame, controller | Intended Panda; initially right; distinct identities for multiple arms |
| `reference_to_robot_xyzw` | Unit quaternion rotating Quest LOCAL into robot base; each physical axis/sign verified in mock first |
| Translation/rotation/feedforward scales, cutoff | Reviewed first-test mapping and motion bounds |
| Arms YAML `controller_enabled`, `operator_confirm` | **Keep both false**: mapper guards. Real activation uses the launch confirmation |
| `runtime_hardware.yaml`: four impedance gains | Damping `Kr`, static stiffness `Kr*Lambda`, units and physical response reviewed |
| Torque rate, tool offset and envelope margin | Reviewed bounds within Panda limits; envelope is a coarse reach bound, not IK/collision validation |
| Homing speed and seven-element stiffness/damping arrays | Full physical path, cancellation/settling and gains reviewed before A/X |
| Hand settings | Actual object width/tolerances/force/speed; use `-1.0` for all three unknown width/tolerance values to refuse grasp; omit Hand during first arm trial |
| CBF table/self/inter-arm clearances | Measured world/table and conservative physical padding; retain failure handling |
| `arm_settings.panda_1.world_from_base_xyz_xyzw` | `[x,y,z,qx,qy,qz,qw]`, metres and unit quaternion, robot base into world |

A single-arm world may equal its base if that choice is explicitly recorded;
then use identity transform and measure the table in that frame. These YAML
entries do not set Desk/tool/load configuration automatically.

**C2:** screen and normalize configuration without FCI. This is not physical approval:

```bash
python3 - <<'PY' | tee /ws/log/commissioning/current/config-check.txt
from pathlib import Path
import math, yaml
from quest_teleop.configuration import load_arms
p = Path('/ws/log/commissioning/current')
def complete(v):
    if isinstance(v, dict):
        for x in v.values(): complete(x)
    elif isinstance(v, list):
        for x in v: complete(x)
    else:
        assert v is not None and 'REPLACE_' not in str(v), 'Unfilled value'
        if isinstance(v, float): assert math.isfinite(v), 'Nonfinite value'
for name in ('arms_single.yaml','runtime_hardware.yaml'):
    complete(yaml.safe_load((p/name).read_text()))
arms = load_arms(str(p/'arms_single.yaml'))
assert len(arms) == 1 and arms[0].robot_ip
r = yaml.safe_load((p/'runtime_hardware.yaml').read_text())
def floats(v):
    if isinstance(v, list): return [floats(x) for x in v]
    assert isinstance(v, (float,int)) and not isinstance(v, bool) and math.isfinite(v)
    return float(v)
for section in ('controller','hand','cbf'):
    for k, v in r[section].items():
        if section == 'cbf' and k == 'maximum_failures':
            assert isinstance(v, int) and not isinstance(v, bool) and 1 <= v <= 100
        else:
            r[section][k] = floats(v)
c = r['controller']
for k in ('lambda_translation','lambda_rotation','gain_translation','gain_rotation'):
    assert isinstance(c[k], (float,int)) and c[k] > 0
assert 0 < c['maximum_torque_rate'] <= 1000
assert 0 < c['homing_speed_factor'] <= 1
for k in ('homing_stiffness','homing_damping'):
    assert len(c[k]) == 7 and all(x > 0 for x in c[k])
for a in arms:
    t = floats(r['arm_settings'][a.name]['world_from_base_xyz_xyzw'])
    assert len(t) == 7 and abs(sum(x*x for x in t[3:]) - 1) < 1e-6
    r['arm_settings'][a.name]['world_from_base_xyz_xyzw'] = t
(p/'runtime_hardware.yaml').write_text(yaml.safe_dump(r))
print('Basic configuration screen passed; physical review still required.')
PY
```

**Pass:** filled finite values, valid assignments/transforms and documented
physical review/stop plan. Allowed keys are in
`src/teleop_bringup/launch/teleoperation.launch.py`;
unknown settings are rejected.

Validate the commissioned basis/scales using **mock geometry**, not real FCI.
Stop old launches. **C1:**

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  backend:=mock mode:=single start_bridge:=true start_hand:=false \
  arms_config:=/ws/log/commissioning/current/arms_single.yaml \
  runtime_config:=/ws/log/commissioning/current/mock_runtime.yaml
```

**C2:**

```bash
ros2 topic echo /panda_1/cartesian_teleop_command
```

Repeat known-axis movements from 3.2. **Pass:** directions, handedness, units,
anchoring and measured target deltas match the reviewed basis/scales. Record
measurements, then stop this mock before FCI. Complete section 1.5's legacy
connection check before new physical teleoperation.

## 5. First real FCI teleoperation test

Advance only through stages whose prerequisites passed. Hand, home and dual
motion need their own acceptance. Stop legacy owners/demos and all mocks.
One owner per physical arm is mandatory; do not launch separate homing/bringup
controllers alongside this integrated runtime.

### 5.0 Hardware recording test with the accepted teleoperation baseline

Stop previous launches with Ctrl+C and wait for their processes to exit. In a
new container terminal, source only `/ws/install/setup.bash`. A mock run does
not change the robot configuration, but leftover mappers or mock measured-pose
publishers can interfere with later hardware operation on the same topics.
Before launching, `ros2 node list` should contain no `/panda_1/pose_mapper`, arm
owner, `/quest_usb_bridge`, or previous recorder from a stopped teleop launch.
Other application nodes may remain.

**Operator:** activate FCI in Desk. **Headset:** run the existing connected
Quest app and release the right index trigger. **C1:**

```bash
cd /ws
source install/setup.bash
mkdir -p log/commissioning/current
set -o pipefail
ros2 launch teleop_bringup teleoperation.launch.py \
  backend:=hardware operator_confirm:=true mode:=single single_controller:=right \
  start_controller:=true start_mapper:=true start_bridge:=true \
  start_hand:=false homing_enabled:=false cbf_enabled:=false \
  mock_measured_pose:=false recording_enabled:=true recording_mode:=menu \
  arms_config:=/ws/log/commissioning/current/arms_single.yaml \
  runtime_config:=/ws/log/commissioning/current/runtime_hardware.yaml \
  recording_output_root:=/ws/log/commissioning/current/hardware_bags \
  2>&1 | tee /ws/log/commissioning/current/hardware-recording.txt
```

This uses the accepted CBF-disabled hardware baseline. Input/model/actuator
guards remain enforced. Homing and gripper motion are omitted from this recording
test. The owner must report `hardware backend`, and the bridge must report
`Quest TCP client connected`.

**C2**, a second terminal in the same container:

```bash
source /ws/install/setup.bash
ros2 topic info /panda_1/cartesian_teleop_command --verbose
ros2 topic info /panda_1/teleop_anchor_pose --verbose
ros2 topic echo /quest/recording_status --qos-durability transient_local
```

Expect one mapper publisher on the command topic and one arm-owner publisher
on the measured-pose topic. Duplicate publishers indicate a leftover launch;
stop it before testing. On Quest, release left Menu then press it once. Status
must report `recording`, an output path and selected topics. Release the right
trigger fully, then hold it and make the reviewed small motion. Verify the
physical arm follows, then release the trigger. After about ten seconds,
release/press left Menu again. Wait for `idle` with `bag finalized`.

Stop the status echo with Ctrl+C and inspect the latest finalized episode:

```bash
BAG_DIR=$(python3 - <<'PY'
from pathlib import Path
root = Path('/ws/log/commissioning/current/hardware_bags')
files = list(root.glob('*/metadata.yaml'))
if not files:
    raise SystemExit('No finalized hardware bag found')
print(max(files, key=lambda p: p.stat().st_mtime).parent)
PY
)
ros2 bag info "$BAG_DIR" \
  | tee /ws/log/commissioning/current/hardware-bag-info.txt
```

**Pass:** duration approximately matches the episode; raw right input, mapped
commands, target/measured pose and robot state have nonzero message counts.
Compare target and measured pose during the physical motion. Only published
topics are selected at episode start; Hand topics may be absent. Repeat Menu
start/stop and verify a different output path with the first episode preserved.
For shutdown finalization, start a further episode, then Ctrl+C C1 and wait for
exit; its bag must contain `metadata.yaml` and be readable with `ros2 bag info`.
If following fails, capture `bash scripts/teleop_diagnostics.sh /panda_1 30`
in C2 while repeating trigger release/hold, rather than restarting more owners.

### 5.1 Single-arm measured hold, timing and stopping

**Operator:** verify load/tool/start pose, cleared area, observer/stop and Desk
FCI activation. **Headset:** triggers/grips released; do not press A/X.
**C1:**

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  backend:=hardware operator_confirm:=true mode:=single single_controller:=right \
  start_bridge:=true start_hand:=false \
  arms_config:=/ws/log/commissioning/current/arms_single.yaml \
  runtime_config:=/ws/log/commissioning/current/runtime_hardware.yaml \
  recording_output_root:=/ws/log/commissioning/current/hardware_bags \
  2>&1 | tee /ws/log/commissioning/current/hardware-single.txt
```

Confirmation opens real torque control. Measured-pose hold acts even while
Quest motion is disarmed; Hand is omitted at this stage.

**C2:**

```bash
ros2 topic echo /panda_1/controller_status --once \
  > /ws/log/commissioning/current/hardware-controller.yaml
ros2 topic echo /panda_1/cbf_status --once \
  > /ws/log/commissioning/current/hardware-cbf.yaml
ros2 topic echo /panda_1/robot_state --once \
  > /ws/log/commissioning/current/hardware-state.yaml
ros2 topic echo /panda_1/teleop_anchor_pose --once \
  > /ws/log/commissioning/current/hardware-anchor.yaml
ros2 topic echo /panda_1/controller_status
```

**C3:**

```bash
ros2 topic echo /panda_1/cbf_status \
  | tee /ws/log/commissioning/current/hardware-cbf-observation.yaml
```

**Pass:** `hardware_backend: true`, released-trigger `motion_enabled: false`,
physical pose holds without unexpected movement, finite state/torques, no
fallback/fatal fault/failure growth/overflow. Padded predicted clearances are
positive and agree with the physical survey. Observe filter/solver time, source
timestamps/skew and stop reason; complete the qualified timing review in 4.2.
An inactive inter-arm clearance sentinel is not a measured separation.

Verify normal shutdown with Ctrl+C **C1** during supervised hold. **Pass:**
filtered hold/finish completes, launch exits without restart/unexpected movement,
active bag finalizes. Test manufacturer stop/deliberate recovery at the approved
hold stage, recording Desk/process response. Trigger release/software hold/Ctrl+C
is not an emergency stop. For unexpected behavior use the manufacturer stop.
Write results in `verification.md` before allowing movement.

### 5.2 Small single-arm motion and loss/recovery

For an explicit diagnostic comparison without CBF constraints, add
`cbf_enabled:=false` to the integrated launch command. The default is `true`.
This disables joint position/velocity/acceleration and table/self/inter-arm/
obstacle CBF constraints and the iterative QP. Trigger release/hold, tracking,
freshness, workspace envelope, torque magnitude/slew limits, model validity,
realtime deadlines and Franka's built-in protections remain in place. The
controller logs the disabled mode and `/panda_1/cbf_status` reports
`solver_status: disabled_actuator_bounds_only`. This is a diagnostic comparison
with reduced software protection: use the already reviewed small single-arm
test, Hand disabled, and operator stop arrangement. No homing or dual-arm
collision acceptance can be inferred from this comparison. Remove the argument
or set `cbf_enabled:=true` to restore CBF on the next launch.

If the owner runs but the arm does not follow, collect the full input chain in
another **sourced ROS container terminal** while the launch remains running:

```bash
bash /ws/scripts/teleop_diagnostics.sh /panda_1 15
```

During those 15 seconds, use the right index-trigger release/hold gesture and
the already reviewed small displacement. The script only subscribes; it sends
no robot commands. It saves right input, mapped command, measured pose,
controller status and CBF status together under the printed commissioning
directory. Inspect `input_valid`, `motion_enabled`, controller `reason`,
`fallback_active`, `solver_status` and target/measured position changes. Empty
files indicate missing samples in that terminal's ROS environment, not evidence
that the arm is following. Preserve stderr files and the environment manifest.

Restart the same C1 command after deliberate recovery/operator recheck.
**C2:**

```bash
ros2 topic echo /panda_1/cartesian_teleop_command
```

Keep C3 watching CBF (or controller status as needed). **Headset:** release
left Menu then press to record; note the output path. Release right trigger,
press/hold with the hand still. **Pass:** no anchor jump; engage target equals
measured pose and only the selected consumer enables.

Move slowly through the **reviewed small displacement** on one axis, release,
repeat each remaining axis. Only after translations pass, test reviewed small
rotations separately. **Pass:** real measured pose follows the correct
sign/scale within reviewed error, stable response and no unexpected CBF/robot/
torque-limit fault. Target movement alone does not pass physical tracking.
Record actual bounds/errors and bag path.

Release trigger during approved motion. **Pass:** disarmed measured-pose damping
hold, no pursuit of stale target. Near stationary, separately test the approved
USB/focus/tracking loss conditions. **Pass:** controlled disarm, no recovery
while trigger remains held. H1 reapplies reverse after replug; fresh release/
press permits reengagement. Do not intentionally drive into collision, joint
limits or singularities, or inject stale models on hardware.

Stop Menu recording with fresh release/press. Stop C2 echo, then **C2**:

```bash
ros2 bag info /ws/log/commissioning/current/hardware_bags/REPLACE_WITH_EPISODE \
  | tee /ws/log/commissioning/current/hardware-bag-info.txt
```

Finalize/record evidence, Ctrl+C C1 and wait for exit before changing settings.
Any failed timing, clearance, tracking or stop check blocks the next stage;
investigate rather than lowering safety thresholds.

### 5.3 Physical Franka Hand

After reviewing actual object settings, pinch clearance and calibration stroke,
restart the **5.1 C1 command** with `start_hand:=true` instead of false. Worker
startup now calibrates automatically: keep the fingers empty and clear and
the arm trigger released. Wait for `calibrated: true` and `busy: false` on
`/panda_1/gripper_events` before testing grip toggles. Use the manual procedure
below only when deliberate recalibration is needed.

Stop C3's earlier observer with Ctrl+C, then **C3:**

```bash
ros2 topic echo /panda_1/gripper_events \
  | tee /ws/log/commissioning/current/hardware-hand-events.yaml
```

**Operator:** Hand empty and full stroke clear. **C2**, this moves real fingers:

```bash
ros2 service call /panda_1/calibrate_gripper \
  quest_teleop_interfaces/srv/CalibrateGripper \
  '{empty_hand_confirm: true, operator_confirm: true}' \
  | tee /ws/log/commissioning/current/hardware-hand-calibration.txt
```

**Pass:** actual completion, calibrated/nonbusy, no fault, maximum width agrees
with installed Hand. First deliberate grip press opens. Release/press tests a
reviewed grasp with the actual object; release/press opens again. Confirm width,
force/tolerances, no repeats while held and no uncommanded release. USB loss
does not interrupt an already issued operation. Communication failure requires
deliberate recalibration/reconnect; no automatic retry or force increase.
Record outcomes before allowing normal grasp operation.

### 5.4 Physical home and cancellation

Home is the fixed joints in 3.4, not a taught/current pose. Review its full
swept path from the actual start, tool/fingers/cables, stiffness/damping, speed
and stop trajectory first. No separate FCI homing process is needed.

Stop any previous C2 echo. **C2:**

```bash
ros2 topic echo /panda_1/homing_status \
  | tee /ws/log/commissioning/current/hardware-home-events.yaml
```

Stop C3's Hand echo and resume CBF observation there:

```bash
# C3
ros2 topic echo /panda_1/cbf_status
```

**Headset:** trigger released; hold A at least one second,
release. **Pass:** one request, smooth approved motion, `succeeded` after
settling, no stop failure/robot/CBF/timing fault or physical clearance violation.
An arm that looks near home may still be in `homing`; pressing its trigger then
cancels the request. Wait for `succeeded`, then release the trigger again and
press/hold it to resume teleoperation. The owner remains connected. Cancellation
decelerates and captures measured joint hold once velocity is low; it does not
require reaching the unfinished home goal before returning to Cartesian hold.
From an approved trajectory portion, test cancellation by trigger press or
approved input loss: `stopping` then `cancelled`/measured hold, fresh release/
press required, no automatic home restart. A stop-failed/fatal path ends FCI
and requires investigation. Record maximum joint error/velocity and physical
stop distance/path in `verification.md`.

### 5.5 Two real arms

Pass relevant single-arm checks on **each** Panda first. Single mode masks the
other arm, so keep the parked arm outside the tested swept area during those
checks. Mock two-metre spacing does not validate physical inter-arm geometry.

**H1:** create separate dual files from the reviewed single configuration:

```bash
python3 - <<'PY'
from pathlib import Path
from copy import deepcopy
import yaml
p = Path('log/commissioning/current')
one = yaml.safe_load((p/'arms_single.yaml').read_text())['arms']['panda_1']
arms = {}
for side in ('left','right'):
    name = 'panda_' + side
    a = deepcopy(one)
    a.update(robot_ip='REPLACE_WITH_' + side.upper() + '_IP', controller=side,
        namespace='/' + name, base_frame=name + '/panda_link0', reference_to_robot_xyzw=None)
    arms[name] = a
(p/'arms_dual.yaml').write_text(yaml.safe_dump({'arms': arms}))
r = yaml.safe_load((p/'runtime_hardware.yaml').read_text())
r['arm_settings'] = {name: {'world_from_base_xyz_xyzw': None} for name in arms}
(p/'runtime_dual_hardware.yaml').write_text(yaml.safe_dump(r))
PY
nano log/commissioning/current/arms_dual.yaml
nano log/commissioning/current/runtime_dual_hardware.yaml
nano log/commissioning/current/verification.md
```

Fill distinct IPs, each verified controller basis/scales, and **both measured
base transforms in one shared world**. Hardware dual explicitly requires these
transforms. Review common table/inter-arm padding and both home/test paths.
Per-arm `controller`/`hand` overrides may be added under each `arm_settings`
name. Shared gains/object values require review, not copying by assumption.

**C2:** check assignments/transforms without FCI:

```bash
python3 - <<'PY' | tee /ws/log/commissioning/current/dual-config-check.txt
from pathlib import Path
import yaml, math
from quest_teleop.configuration import load_arms, select_arms
p = Path('/ws/log/commissioning/current')
arms = select_arms(load_arms(str(p/'arms_dual.yaml')), 'dual', 'right')
assert len({a.robot_ip for a in arms}) == 2
r = yaml.safe_load((p/'runtime_dual_hardware.yaml').read_text())
for a in arms:
    assert a.robot_ip and 'REPLACE_' not in a.robot_ip
    t = r['arm_settings'][a.name]['world_from_base_xyz_xyzw']
    assert isinstance(t, list) and len(t) == 7 and all(math.isfinite(x) for x in t)
    assert abs(sum(x*x for x in t[3:]) - 1) < 1e-6
    r['arm_settings'][a.name]['world_from_base_xyz_xyzw'] = [float(x) for x in t]
    print(a.name, a.robot_ip, a.namespace, a.parameters['controller_id'], t)
(p/'runtime_dual_hardware.yaml').write_text(yaml.safe_dump(r))
PY
```

For individual acceptance using the dual files, **C1** (one launch at a time):

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  backend:=hardware operator_confirm:=true mode:=single single_controller:=left \
  start_bridge:=true start_hand:=false \
  arms_config:=/ws/log/commissioning/current/arms_dual.yaml \
  runtime_config:=/ws/log/commissioning/current/runtime_dual_hardware.yaml
```

Then stop it and repeat with `single_controller:=right`. Topics/services now
use `/panda_left` or `/panda_right`; repeat 5.1/5.2's observations. This also
requires the individually reviewed gains/tool/load for the selected robot.

After single passes, shared geometry/timing review, both FCI activations and
both triggers released, **C1** starts **one dual owner**:

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  backend:=hardware operator_confirm:=true mode:=dual \
  start_bridge:=true start_hand:=false \
  arms_config:=/ws/log/commissioning/current/arms_dual.yaml \
  runtime_config:=/ws/log/commissioning/current/runtime_dual_hardware.yaml \
  recording_output_root:=/ws/log/commissioning/current/hardware_dual_bags \
  2>&1 | tee /ws/log/commissioning/current/hardware-dual.txt
```

```bash
# C2
ros2 topic echo /panda_left/cbf_status --once \
  > /ws/log/commissioning/current/hardware-dual-left-cbf.yaml
ros2 topic echo /panda_left/controller_status
```

```bash
# C3
ros2 topic echo /panda_right/cbf_status \
  | tee /ws/log/commissioning/current/hardware-dual-right-cbf.yaml
```

**Hold pass:** both measured poses stable/disarmed; coupled solver healthy with
both surveyed arm models; positive padded inter-arm clearance consistent
with survey; age/skew/filter/result gates met, no fallback/fatal fault/overflow.
`inter_arm_constraints` may be zero when separation exceeds the 0.15 m
activation distance above the padded boundary. That is expected at wide
separation; do not move closer merely to force a nonzero count. Inter-arm
activation/torque behavior is covered by the controlled geometry software tests.
Repeat qualified timing measurements with **both** arms; single-arm timing is
insufficient. A shared latched stop must end the owners without reconnect.

Record with Menu. Engage only left and perform its approved small motions,
release; repeat only right with left parked. **Pass:** correct independent
assignment, parked arm holds and remains in collision constraints. Only after
both passes test reviewed simultaneous movements with wide separation. Do not
validate collision avoidance by moving arms into each other.

Review home paths separately: X requests left, A right; begin with one home at
a time. Enable Hand only after each physical Hand acceptance. Stop C2 echo for
services; calibration commands are those in 3.6 and now **move real fingers**.
Record independent Hand/home and shared timing/clearance/normal paired shutdown/
manufacturer stop results. Finalize Menu recording, then **C2**:

```bash
ros2 bag info /ws/log/commissioning/current/hardware_dual_bags/REPLACE_WITH_EPISODE \
  | tee /ws/log/commissioning/current/hardware-dual-bag-info.txt
```

Ctrl+C C1 and wait for both owners to finish. A failed shared stop/deadline
check blocks dual acceptance. Update `verification.md` with each arm's outcome.

## Evidence, diagnostics and optional plots

Stop observers and active launches normally; wait for owner exit and bag
finalization. **H1:** finish and archive the record/configs/logs/bags/traces:

```bash
nano log/commissioning/current/verification.md
cp src/teleop_bringup/config/teleoperation_runtime.yaml \
  log/commissioning/current/shipped-runtime-reference.yaml
tar -czf "log/commissioning/commissioning-$(date +%Y%m%d-%H%M%S).tar.gz" \
  -C log/commissioning current
```

Record separate outcomes for arm motion, Hand, home and dual. An implementation
pass with an incomplete physical record means hardware acceptance is unresolved.

| Symptom | Check / next action |
| --- | --- |
| Missing package | Correct revision/source, build log, source `/ws/install/setup.bash` in this shell |
| No Quest input | USB authorization/data cable, `adb devices -l`, reverse mapping, one bridge/app focus |
| Old button schema | Stop stale publishers; current overlay, `ros2 interface show` / `topic info --verbose` |
| No reengagement | Valid tracking/clock plus release below 0.25 before press above 0.75 |
| Home ignored | Correct X/left or A/right, fresh tracking, trigger released, one-second hold then release, not busy |
| Hand accepted but not calibrated | Actual `gripper_events` completion/fault; service response is asynchronous |
| Grasp refused | Calibration/object width/tolerances; deliberately calibrate after reviewing settings |
| FCI failure | Desk health/activation, correct IP/protocol, wired route, exclusive owner, actual libfranka error |
| FIFO/mlock failure | Host RT kernel and effective container limits/privileges; do not bypass enforcement |
| CBF fallback/fatal/deadline stop | Preserve evidence, operator stop if needed; investigate geometry/feasibility/timing before deliberate restart |
| Missing/empty bag | Recording status, selected published topics, exact output path, `.recorder.log`, normal finalization |

Optional X11 visualization: **H1**, before opening the ROS container:

```bash
xhost +SI:localuser:root
```

For legacy RViz, use the reviewed existing bringup launch with `use_rviz:=true`;
it still opens real FCI. For PlotJuggler, **C3**, sourced workspace:

```bash
cd /ws
./scripts/plotjuggler_tracking.sh
```

Subscribe to legacy `/franka_robot_state_broadcaster/robot_state` or new
`/panda_1/robot_state` (dual `/panda_left/robot_state`, `/panda_right/robot_state`).
Compare actual `o_t_ee[12..14]` and desired `o_t_ee_d[12..14]` (column-major SE(3)
positions); compare orientation separately. Save layout at
`/ws/config/plotjuggler_tracking.xml`. GUI/ROS plots do not measure the whole
realtime path. After use, **H1**:

```bash
xhost -SI:localuser:root
```

Further details: [runtime](docs/TELEOPERATION_RUNTIME.md),
[CBF](docs/CBF_PANDA.md), [mapping](docs/ROBOT_FREE_MAPPING.md),
[USB protocol](docs/quest-usb-protocol.md), [development](docs/DEVELOPMENT.md),
[VR port](docs/VR_APP_PORT.md).
