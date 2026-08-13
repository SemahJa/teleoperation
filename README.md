# Panda Cartesian impedance — ready to go

This repository contains a direct `libfranka` Cartesian impedance controller for a legacy Panda
on FCI 4.2.2, a separate pose-trajectory publisher, RViz visualization, and PlotJuggler support.

The repository deliberately does **not** contain the two large third-party Git checkouts. They are
listed and pinned in `third_party.repos`; one command fetches both. They are not Git submodules.

## First-time setup

On a Linux host with Docker installed:

```bash
git clone https://github.com/dp-tumLi/panda_ros2_to_go.git
cd panda_ros2_to_go
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml run --rm panda-ros2
```

You are now inside the container at `/ws`. Run these commands once to fetch the exact legacy
dependencies and build the workspace:

```bash
mkdir -p src/third_party
vcs import src/third_party < third_party.repos
git -C src/third_party/libfranka_0_9_2 submodule update --init --recursive

rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y \
  --skip-keys ament_cmake_clang_format

colcon build --symlink-install --packages-select libfranka \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash

colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

There is no patch to apply and no `git submodule update` for this repository. The only submodule
command above belongs to the pinned LCAS `libfranka` checkout.

For RViz or PlotJuggler X11 windows, run this once on the **host** before starting the container:

```bash
xhost +local:root
```

If this repository is opened in VS Code Dev Containers, use **Dev Containers: Rebuild Container**
after Dockerfile changes, then run the same dependency-import and build commands in the VS Code
terminal.

## Run the controller

Open a container shell and source the built workspace:

```bash
cd /ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Start the controller. It is the only process that connects to FCI; it holds the initial end-effector
pose until a target is published:

```bash
ros2 launch panda_bringup cartesian_impedance_controller.launch.py \
  robot_ip:=192.168.3.107 operator_confirm:=true use_rviz:=true
```

In a second container terminal, source the workspace and launch the example trajectory. It never
connects to the robot; it waits for the controller initial pose, then publishes a 10 cm XY circle
at the initial height and orientation:

```bash
cd /ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch panda_demos circle_trajectory.launch.py
```

Your future policy/executor publishes only this standard input:

```text
/panda_cartesian_impedance_controller/target_pose
geometry_msgs/msg/PoseStamped
```

The controller publishes the Franka-compatible state topic:

```text
/franka_robot_state_broadcaster/robot_state
franka_msgs/msg/FrankaState
```

It contains actual `o_t_ee` and desired `o_t_ee_d`, both column-major SE(3) matrices.

## Controller parameters

Edit [cartesian_impedance_controller.yaml](src/panda_bringup/config/cartesian_impedance_controller.yaml),
rebuild `panda_bringup`, and restart the controller. The controller uses Spong filtered-error
feedback:

```text
r = V - Vd + Lambda e
tau = -Jᵀ Kr r + coriolis
```

Defaults are `Lambda=50`, `Kr=20` for translation (equivalent static gain 1000 N/m), and
`Lambda=40`, `Kr=2.5` for rotation (equivalent static gain 100 Nm/rad).

## RViz and PlotJuggler

RViz starts with the controller when `use_rviz:=true`. It uses the Franka Panda mesh and publishes
the target path marker at `/panda_circle_trajectory/target_path`.

Start PlotJuggler in another sourced container terminal:

```bash
cd /ws
./scripts/plotjuggler_tracking.sh
```

Subscribe to `/franka_robot_state_broadcaster/robot_state`. Compare `o_t_ee[12..14]` with
`o_t_ee_d[12..14]` for position. Save your two-panel layout as
`/ws/config/plotjuggler_tracking.xml`; later script invocations load it automatically.
