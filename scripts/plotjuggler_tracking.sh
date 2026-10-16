#!/usr/bin/env bash
set -euo pipefail

# Inside the ROS container: bash scripts/plotjuggler_tracking.sh /panda_1
# Multiple arms: pass both namespaces. Subscribe to each tracking_status topic
# using Streaming -> ROS2 Topic Subscriber. See docs/TRAJECTORY_MONITORING.md.

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
layout="$(mktemp /tmp/teleop-tracking-XXXXXX.xml)"
trap 'rm -f "$layout"' EXIT
layout_args=()
for arm_namespace in "${@:-/panda_1}"; do
  layout_args+=(--namespace "$arm_namespace")
done
python3 "${workspace_root}/scripts/create_tracking_layout.py" "${layout_args[@]}" --output "$layout"

source /opt/ros/humble/setup.bash
if [[ -f "${workspace_root}/install/setup.bash" ]]; then
  source "${workspace_root}/install/setup.bash"
fi

ros2 run plotjuggler plotjuggler -l "$layout"
