# Container workflow

The image intentionally uses the complete `ros:humble-desktop-jammy` base and also
preinstalls the ROS 2 control-controller set, RViz, MoveIt, robot-state publishers,
and the legacy Panda build dependencies. This avoids discovering missing runtime
packages during hardware bring-up. The only rosdep key intentionally skipped below
is the legacy repository's unavailable lint-only `ament_cmake_clang_format` key.

From the workspace root, build and enter the development container:

    docker compose -f docker/compose.yaml build
    docker compose -f docker/compose.yaml run --rm panda-ros2

Inside the container, import the pinned legacy Panda driver and build:

    mkdir -p src/third_party
    vcs import src/third_party < third_party.repos
    git -C src/third_party/libfranka_0_9_2 submodule update --init --recursive
    git -C src/third_party/franka_arm_ros2 apply ../../../patches/lcas-franka-arm-ros2-cartesian-handover.patch
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys ament_cmake_clang_format
    colcon build --symlink-install --packages-select libfranka --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
    source install/setup.bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
    source install/setup.bash

Use this container for development and fake hardware. For first physical-robot
bring-up, boot the host into its PREEMPT_RT kernel. Docker shares the host kernel and
does not create realtime capability. This compose configuration grants `SYS_NICE`,
realtime priority, and memory locking, but they are only effective after the host RT
kernel and user limits are configured. Do not use Docker Desktop for FCI.