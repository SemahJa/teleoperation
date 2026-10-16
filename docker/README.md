# Container workflow

The image preinstalls ROS 2 Humble desktop tools, RViz, PlotJuggler, and the legacy Panda build
dependencies. Its ROS base image is pinned by digest. The only rosdep key intentionally skipped below is the legacy repository's
unavailable lint-only `ament_cmake_clang_format` key.

From the workspace root, build and enter the development container:

    docker compose -f docker/compose.yaml build
    docker compose -f docker/compose.yaml run --rm --name franka-teleop panda-ros2

Inside the container at `/ws`, import pinned dependencies and build the active
teleoperation pipeline:

    cd /ws
    source /opt/ros/humble/setup.bash
    bash scripts/fetch_teleop_dependencies.sh
    apt-get update
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys ament_cmake_clang_format
    colcon build --base-paths src --symlink-install --packages-up-to teleop_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
    source install/setup.bash

Run `apt-get update` in each fresh container before installing dependencies.
The image removes APT package indexes to reduce its size; `rosdep update`
refreshes ROS dependency mappings and does not restore those indexes. An
`Unable to locate package ros-humble-warehouse-ros-sqlite` error during rosdep
installation should first be resolved by successfully running `apt-get update`
and retrying the same command. Keep this MoveIt runtime dependency enabled.

ROS builds use `--base-paths src` to restrict discovery to ROS sources.
`android/COLCON_IGNORE` also excludes the Android project from default colcon
discovery; build the APK with the separate Gradle/NDK container below.

Colcon orders the active packages' libfranka dependency automatically. If you
also need the entire legacy workspace, build and source libfranka first; the
legacy `panda_bringup` manifest omits that dependency. Legacy demos are not
needed for teleoperation. The standard overlay is `/ws/install/setup.bash`.

There is no workspace patch to apply. Use this container for development and fake hardware. For first physical-robot
bring-up, boot the host into its PREEMPT_RT kernel. Docker shares the host kernel and
does not create realtime capability. This compose configuration grants `SYS_NICE`,
realtime priority, and memory locking, but they are only effective after the host RT
kernel and user limits are configured. Do not use Docker Desktop for FCI.

## Quest Android build and ADB

The native C++ OpenXR toolchain and USB ADB service are defined separately. Debug signing credentials persist in the `android-signing` Docker volume so APK updates keep the same signature:

    docker compose -f docker/android-compose.yaml build
    docker compose -f docker/android-compose.yaml run --rm android-build gradle assembleDebug

For USB deployment, authorize the Quest over a physical cable, then:

    docker compose -f docker/android-compose.yaml up -d quest-adb-server
    docker compose -f docker/android-compose.yaml run --rm quest-adb adb devices

The ADB server container has direct `/dev/bus/usb` access and is not exposed on a host port. Never use wireless ADB. USB permissions and device hotplug are host-dependent. See `docs/DEVELOPMENT.md` for details.

## Optional MATLAB reports

`matlab-compose.yaml` is separate from the ROS and Android environments. It uses
the official R2024b image, mounts `analysis/matlab` read-only and `log` read/write,
and binds its browser port to localhost. An eligible MATLAB license is required;
Docker does not include or transfer a license. See
[first-time setup, batch licensing and handover](../docs/TRAJECTORY_MONITORING.md#first-time-matlab-docker-setup-and-licensing).
Teleoperation and CSV export work without starting this container.
