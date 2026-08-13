# PlotJuggler tracking layout

Run `scripts/plotjuggler_tracking.sh` inside the container. Connect the ROS Topic Subscriber to
`/franka_robot_state_broadcaster/robot_state`. It is the standard `franka_msgs/FrankaState`
published by the direct controller. Current and desired are both column-major SE(3) matrices:
`o_t_ee` and `o_t_ee_d`. Create the first plot by comparing entries 12, 13, 14 of each matrix.
For the second plot convert the 3x3 rotation portions (entries 0..10) of both matrices to the
same SO(3) rotation-vector convention in PlotJuggler. Save the layout here as
`plotjuggler_tracking.xml`; later runs reload it.
