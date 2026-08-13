#!/usr/bin/env bash
set -euo pipefail

# Run this inside the Panda ROS Docker container after sourcing the workspace.
# On first use: Streaming → ROS Topic Subscriber, then select
# /franka_robot_state_broadcaster/robot_state. It contains both current o_t_ee
# and desired o_t_ee_d as column-major SE(3) matrices. Compare entries 12/13/14
# in the first plot. Use the same matrix-to-SO(3) conversion for entries 0..10
# of both matrices in the second plot.
# Save the layout
# as config/plotjuggler_tracking.xml. Future runs load it automatically.

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
layout="${workspace_root}/config/plotjuggler_tracking.xml"

source /opt/ros/humble/setup.bash
if [[ -f "${workspace_root}/install/setup.bash" ]]; then
  source "${workspace_root}/install/setup.bash"
fi

if [[ -f "${layout}" ]]; then
  exec ros2 run plotjuggler plotjuggler -l "${layout}"
fi
exec ros2 run plotjuggler plotjuggler
