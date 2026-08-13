# Panda Cartesian impedance

Two independent ROS 2 nodes: a direct libfranka Cartesian-impedance controller and a pose trajectory publisher.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select panda_bringup panda_demos
source install/setup.bash
```

Start the controller first. It connects to the robot and holds its initial end-effector pose:

```bash
ros2 launch panda_bringup cartesian_impedance_controller.launch.py robot_ip:=192.168.3.107 operator_confirm:=true
```

In a second terminal, start the example trajectory. It does not connect to the robot; it waits for the controller initial pose, then publishes a 10 cm XY circle while preserving initial Z and orientation:

```bash
ros2 launch panda_demos circle_trajectory.launch.py
```

For RViz, add `use_rviz:=true` to the controller launch. In RViz set Fixed Frame to `panda_link0` and add a Marker display for `/panda_circle_trajectory/target_path`.

On the Linux host, before starting the container, allow the local container user to use X11:

```bash
xhost +local:root
```

Future trajectory generators publish `geometry_msgs/PoseStamped` to:

```text
/panda_cartesian_impedance_controller/target_pose
```

The controller uses Spong-style filtered-error feedback, `r = V - Vd + Lambda e`,
with `tau = -J^T Kr r + coriolis`. Its fixed controller configuration is
`src/panda_bringup/config/cartesian_impedance_controller.yaml`; edit that file and restart
the controller to tune it. Defaults are `Lambda = 50 s^-1`, `Kr = 20 Ns/m` for translation
(equivalent static stiffness 1000 N/m), and `Lambda = 40 s^-1`, `Kr = 2.5 Nms/rad` for
orientation (equivalent static stiffness 100 Nm/rad).

## Tracking plots

PlotJuggler is installed in the Docker image. Run `scripts/plotjuggler_tracking.sh` in the
container. It compares `o_t_ee` and `o_t_ee_d` from the standard Franka state topic
`/franka_robot_state_broadcaster/robot_state`; save the two-panel layout as
`config/plotjuggler_tracking.xml`.
