# Franka Panda + Quest 3 teleoperation

Run the Quest 3 USB app and a single or dual **legacy Panda (FER)** through ROS 2
Humble. Use Ubuntu 22.04 with native Docker Engine/Compose v2; real FCI control
requires a PREEMPT_RT host, compatible robot firmware and a wired robot network.
Use your installation's configured robot IPs, mapping, gains and geometry.

New to the repository? The [documentation guide](docs/README.md) links the
architecture, operation, monitoring and development instructions. The current
pipeline holds measured joint position after normal trigger release and uses
35% default torque/rate caps even with CBF disabled.

| Terminal | Where commands run |
| --- | --- |
| **Host 1** | PC host: clone and open the ROS container |
| **CONTAINER 1** | That container, `/ws`: build and run teleoperation |
| **Host 2** | PC host, `~/franka_teleoperation`: edit configs and start Quest |
| **CONTAINER 2** | Optional second container shell: standalone commands / launch help |

## 1. Open and build the Docker workspace

**Host 1 — first setup:**

```bash
cd ~
git clone https://github.com/jiachunl0727/franka_teleoperation.git
cd ~/franka_teleoperation
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml run --rm --name franka-teleop panda-ros2
```

**CONTAINER 1 — first build:**

```bash
cd /ws
source /opt/ros/humble/setup.bash
bash scripts/fetch_teleop_dependencies.sh
apt-get update
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y \
  --skip-keys ament_cmake_clang_format
colcon build --base-paths src --symlink-install --packages-up-to teleop_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

Colcon builds libfranka before its dependent controllers automatically. This
command builds the teleoperation pipeline, excluding the legacy Panda demos.
Libfranka's own tests/examples default to off; an unused `BUILD_TESTING` warning
from libfranka is harmless because that option controls the ROS packages.

For subsequent sessions, **Host 1** opens the existing image:

```bash
cd ~/franka_teleoperation
docker compose -f docker/compose.yaml run --rm --name franka-teleop panda-ros2
```

Then **CONTAINER 1** sources the existing workspace; no rebuild is needed unless code changes:

```bash
cd /ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 2. Set the unified configuration

**Host 2:** edit the single-arm template directly (use `dual_panda.yaml` for two arms):

```bash
cd ~/franka_teleoperation
nano src/teleop_bringup/config/single_panda.yaml
```

Select a file explicitly: `single_panda.yaml` for one arm or `dual_panda.yaml`
for two arms, both under `src/teleop_bringup/config/`. Package-local configs remain the defaults
for standalone launches; no config copies under workspace `config/` are needed.
The symlink-install build makes source config edits available without rebuilding;
restart the launch after changing settings.

| YAML section | Configure here |
| --- | --- |
| `pipeline` | Backend, mode and feature switches; `single_controller` selects the hand only in the single-arm template |
| `quest.bridge` | USB receiver port, bind address, namespace and clock/freshness limits; retain port `45454` with the current Quest app |
| `arms.<arm_name>` | Actual `robot_ip`, controller assignment, namespace/base frame, input/output topics, calibrated basis, scales, trigger thresholds and mapper timing |
| `controller` | Impedance gains, actuator limits/rate, tool envelope and homing speed/gains/stop settings |
| `hand` | Finger speed/force, object grasp width/tolerances, open width and input debounce; `-1.0` object values refuse grasp |
| `cbf` | Torque/rate scales (always enforced), surveyed table height, clearances, obstacle spheres and solver failure limit |
| `arm_settings.<arm_name>` | Per-arm controller/Hand overrides and `world_from_base_xyz_xyzw` transforms; names must match `arms` |
| `recording` | `mode: menu/continuous`, namespace, Menu input/debounce, selected topics, output folder and storage plugin |

The single-arm working configuration currently selects hardware with
`pipeline.cbf_enabled: false`; the dual template selects mock with CBF enabled.
Inspect the selected file and set `pipeline.backend: mock` for robot-free work.
Enable CBF on hardware only with commissioned geometry and realtime timing.
Set `pipeline.start_hand: false` to omit fingers,
or `pipeline.homing_enabled: false` to omit home gestures.

Hardware authorization is the explicit `operator_confirm:=true` launch argument;
it cannot be saved in YAML. Internal mapper activation guards retain their fixed
false defaults and are omitted from the templates. Use calibrated IPs, gains and mapping rather than
assuming the supplied software reference values fit your installation.

Use floating-point YAML values and arrays for gains/transforms. The mapping basis
is `[qx,qy,qz,qw]`; the base-to-world transform is `[x,y,z,qx,qy,qz,qw]` in metres.
Both surveyed transforms are required for hardware dual operation. The single
template uses its base frame as world by default and omits inter-arm settings;
the dual template includes left/right assignments and inter-arm clearance.

## 3. Start the Quest app over USB

Enable Quest developer mode, connect a USB **data** cable and accept the USB
debugging prompt inside the headset. Use one connected Android device.

**Host 2 — first Android build/install:**

```bash
cd ~/franka_teleoperation
docker compose -f docker/android-compose.yaml build
docker compose -f docker/android-compose.yaml run --rm android-build gradle assembleDebug
docker compose -f docker/android-compose.yaml up -d quest-adb-server
docker compose -f docker/android-compose.yaml run --rm quest-adb adb devices -l
bash scripts/quest_adb_reverse.sh 45454
bash scripts/quest_install.sh
```

`adb devices` must list the authorized headset as `device`. The install helper
also launches the app. For subsequent sessions, reuse the APK:

```bash
docker compose -f docker/android-compose.yaml up -d quest-adb-server
bash scripts/quest_adb_reverse.sh 45454
docker compose -f docker/android-compose.yaml run --rm quest-adb \
  adb shell am start -n org.franka.questteleop/android.app.NativeActivity
```

After USB replug, rerun `bash scripts/quest_adb_reverse.sh 45454`. If an APK
update reports a signing mismatch, the install helper prints the uninstall
command; uninstalling erases that app's data.

## 4. Start teleoperation

Activate **FCI in Desk**, release the Quest triggers and keep the Hand empty
with its calibration stroke clear. Enabling Hand starts one calibration at launch.

**CONTAINER 1**, after setting `pipeline.backend: hardware` in the YAML:

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  runtime_config:=/ws/src/teleop_bringup/config/single_panda.yaml operator_confirm:=true
```

The file selects single/dual mode and every feature. Only one arm owner per
robot and one USB receiver per port may run. Do not start overlapping standalone
launches. The existing package name is `teleop_bringup`.

| Quest action | Operation |
| --- | --- |
| Release trigger, then hold above its engage threshold (default 0.75) | Engage assigned arm at measured pose; controller motion changes its target |
| Release below its release threshold (default 0.25) | Capture measured joints once and hold the whole arm configuration |
| Fresh grip presses, releasing between presses | First post-calibration press opens; later presses alternate grasp/open |
| Right A / left X held at least one second, then released | Home assigned arm with trigger released and homing enabled |
| Fresh left Menu presses | Start/stop an episode with `recording.mode: menu` |

Ctrl+C stops bringup. Input loss/recenter requires trigger release/press before
resuming. The manufacturer stop remains the physical emergency-stop mechanism.

Normal trigger release selects joint position holding (`reason: position_hold`,
`motion_enabled: false`). Hold stiffness ramps up over 0.25 s; joint damping
acts immediately, and all output still passes through the shared torque limits.
Tune `controller.hold_stiffness` [Nm/rad], `hold_damping` [Nm s/rad] and
`hold_stiffness_ramp_seconds` in the selected runtime YAML. These are finite
impedance gains, not a mechanical lock; physical response needs commissioning.
Invalid/stale input, safety fallback and shutdown retain damping rather than
continuing an old hold target. Startup captures the initial configuration, and
normal homing completion returns to measured hold with fresh input.

Arm torque/rate caps default to **35% of Panda FCI ceilings**, with CBF enabled
or disabled. Edit `cbf.torque_limit_scale` and `cbf.torque_rate_limit_scale`
in the selected runtime YAML and restart: defaults give `30.45 Nm` for joints
1--4, `4.2 Nm` for joints 5--7 and `350 Nm/s`. Existing controller absolute caps
can tighten these further. Standalone safety settings live in
`src/safety_filter/config/safety.yaml`; combined bringup uses its selected YAML.
See [actuator limit settings](docs/CBF_PANDA.md#editable-actuator-limits-including-cbf-disabled-operation).

For dual, edit `src/teleop_bringup/config/dual_panda.yaml`: set actual distinct
IPs, calibrate each controller mapping, supply both surveyed base transforms
under `arm_settings`, and set `pipeline.backend: hardware`. Stop the single
launch, activate FCI for both arms, then **CONTAINER 1**:

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  runtime_config:=/ws/src/teleop_bringup/config/dual_panda.yaml operator_confirm:=true
```

Both enabled Hands calibrate at startup. Dual geometry/timing requires separate
commissioning. Use one dual launch rather than two single-arm owners.

## 5. Overall launch arguments

The overall launch exposes exactly two arguments. **`runtime_config` is required
and has no default path.** Mode, backend, features, port and recording settings
come exclusively from the selected YAML.

| Argument | Required / default | Purpose |
| --- | --- | --- |
| `runtime_config` | Required; no default | Path to the complete `single_panda.yaml`, `dual_panda.yaml` or another file with the same schema |
| `operator_confirm` | Defaults to `false`; `true` required with hardware backend | Explicit authorization to open real FCI connections and perform enabled Hand startup calibration |

`operator_confirm` is unnecessary for mock operation. It does not arm the
trigger, validate installation settings or replace an emergency stop. It prevents
loading a hardware configuration from automatically authorizing physical operation.
For mock operation, after keeping `pipeline.backend: mock`, **CONTAINER 1**:

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  runtime_config:=/ws/src/teleop_bringup/config/single_panda.yaml
```

All other configuration and feature switches must be edited in YAML; there are
no overall command-line overrides or per-package config-file arguments.

## 6. Standalone packages

Standalone launches keep their own package defaults and configuration arguments.
Open **CONTAINER 2** from another host terminal when needed:

```bash
docker exec -it franka-teleop bash
cd /ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Command form: `ros2 launch <package> <launch_file> argument:=value`.

| Package / launch | Standalone configuration / important arguments |
| --- | --- |
| `quest_teleop quest_teleop.launch.py` | Package `config/arms.yaml`; `arms_config`, `mode`, `single_controller`, bridge/mapper switches and bridge parameters |
| `quest_teleop quest_usb_bridge.launch.py` | Bridge launch defaults; port, namespace, bind address and freshness parameters |
| `quest_teleop pose_mapper.launch.py` | Mapper launch defaults; input/output topics, frame, hand, basis/scales and timing |
| `teleop_controllers cartesian_impedance_controller.launch.py` | Package `controllers.yaml`, safety package `safety.yaml`; `runtime_config`, `safety_config`, `arms_config`, backend/confirmation/mode and home/CBF switches |
| `teleop_controllers homing.launch.py` | Same owner/configuration with homing enabled; no automatic home |
| `teleop_gripper gripper.launch.py` | Package `gripper.yaml`; `runtime_config`, `arms_config`, backend/confirmation/mode; one startup calibration |
| `recording recording.launch.py` | Package `recording.yaml`; `recording_config`, `arms_config`, recording mode/topics/output/namespace |
| `safety_filter safety_filter.launch.py` | Package `safety.yaml`; `safety_config`, `arms_config`, `cbf_enabled`; parameter include for an owner, not a separate robot process |

Show complete arguments for any launch in **CONTAINER 2**:

```bash
ros2 launch teleop_bringup teleoperation.launch.py \
  runtime_config:=/ws/src/teleop_bringup/config/single_panda.yaml --show-args
```

The arm controller separates `controller_runtime.cpp` (ROS, mock/hardware and
FCI ownership), `controller_manager.cpp` (arming and nominal-source selection),
and the Cartesian impedance, position hold, fault damping and homing implementations.
All laws share a joint-torque interface and final safety path;
`joint_motion_generator.cpp` remains the homing reference generator. Existing
launch commands and configuration keys are unchanged.

For configuration and controller extension details, see
[docs/PACKAGE_STRUCTURE.md](docs/PACKAGE_STRUCTURE.md#arm-runtime-and-controller-structure).

## Trajectory monitoring and experiment reports

Each arm can set `pose_filter` and `velocity_filter` independently to `none`,
`low_pass`, or `four_pole`. The four-pole filter has a C3 continuous step
response. Each arm can also set `pose_interpolation` and
`velocity_interpolation` to `none`, `ptp_polynomial`, or `spline`.
The septic point-to-point curve is C3 at completed knots; the quintic spline
is C2 there. Separate cutoff and interpolation-duration settings are available
under each arm in the package and bringup YAML files. Interpolation coalesces
new waypoints while a segment runs, so it adds trajectory delay and may skip
intermediate waypoints when input arrives faster than segments complete.

While holding the trigger, hold Y on the left Quest controller or B on the
right to follow orientation while holding the measured end-effector position.
Release Y or B to return to Cartesian impedance tracking without releasing
the trigger. The latest live tracking view is `/<arm>/tracking_status`.
The default bag profile records controller input, mapped command, measured
pose, and `/<arm>/tracking_samples`; its CSV exporter selects only trajectory
fields and timing data for the MATLAB report.

In a second ROS container terminal, run
`bash scripts/plotjuggler_tracking.sh /panda_1` and subscribe to
`/panda_1/tracking_status`. For recorded reports, export bags to CSV with
`ros2 run recording export_tracking BAG OUTPUT_DIRECTORY`.

MATLAB analysis runs in the separate optional `docker/matlab-compose.yaml`
container and exports vector PDFs, high-resolution PNGs, metrics and estimated
acceleration/jerk. Torque and diagnostic report pages are omitted. It requires base MATLAB with an eligible license; no host
installation, ROS Toolbox or Signal Processing Toolbox is required. First-time
account/network activation, batch/browser commands and license handover are
documented in [docs/TRAJECTORY_MONITORING.md](docs/TRAJECTORY_MONITORING.md).
Every plot panel includes an explicit legend identifying its signals. Regenerate
reports to update older figures.

Generated recordings, exports, reports and runtime logs live under ignored
`log/`. The existing experiment data was removed during the requested cleanup;
new recordings create their directories automatically. The reusable analysis
code remains in `analysis/matlab/`. For formatting and robot-free validation,
see [DEVELOPMENT.md](docs/DEVELOPMENT.md).
