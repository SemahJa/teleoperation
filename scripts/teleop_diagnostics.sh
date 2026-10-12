#!/usr/bin/env bash
# Read-only ROS observations; never connects to FCI or publishes commands.
set -euo pipefail

namespace="${1:-/panda_1}"
duration="${2:-15}"
if [[ ! "$namespace" =~ ^/[a-zA-Z0-9_/]+$ ]] || [[ ! "$duration" =~ ^[0-9]+$ ]] || (( duration < 1 || duration > 60 )); then
  echo "Usage: bash $0 [/arm_namespace] [duration_seconds 1..60] [output_root]" >&2
  exit 2
fi
command -v ros2 >/dev/null || { echo 'Source your ROS workspace before running this script.' >&2; exit 2; }
command -v timeout >/dev/null || { echo 'The timeout command is required.' >&2; exit 2; }
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
output_root="${3:-$repo_root/log/commissioning/current}"
output="$output_root/teleop-diagnostics-$(date -u +%Y%m%dT%H%M%SZ)-$$"
mkdir -p -- "$output"
namespace="${namespace%/}"
topics=(
  /quest_usb_bridge/right/controller_state
  "$namespace/cartesian_teleop_command"
  "$namespace/teleop_anchor_pose"
  "$namespace/controller_status"
  "$namespace/cbf_status"
  "$namespace/homing_status"
  "$namespace/gripper_events"
)
types=(QuestControllerState CartesianTeleopCommand PoseStamped ControllerStatus CbfStatus HomingStatus GripperStatus)
names=(right-input mapped-command measured-pose controller-status cbf-status homing-status gripper-events)
pids=()
{
  printf 'namespace=%s\nduration_seconds=%s\n' "$namespace" "$duration"
  printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-0 (default)}"
  printf 'ROS_LOCALHOST_ONLY=%s\n' "${ROS_LOCALHOST_ONLY:-0 (default)}"
  printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-default}"
} > "$output/environment.txt"
echo "Recording read-only observations for $duration seconds: $output"
echo 'Use your normal right-trigger release/hold gesture during the capture.'
echo 'For homing, keep the trigger released and hold A for at least one second, then release A.'
trap 'for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done' INT TERM EXIT
for index in "${!topics[@]}"; do
  if [[ "${types[index]}" == PoseStamped ]]; then
    message_type=geometry_msgs/msg/PoseStamped
  else
    message_type="quest_teleop_interfaces/msg/${types[index]}"
  fi
  durability=volatile
  if [[ "${types[index]}" == GripperStatus ]]; then durability=transient_local; fi
  timeout --signal=INT --kill-after=2s "${duration}s" \
    ros2 topic echo "${topics[index]}" "$message_type" --qos-reliability best_effort --qos-durability "$durability" \
    > "$output/${names[index]}.yaml" 2> "$output/${names[index]}.stderr.txt" &
  pids+=("$!")
done
result=0
for index in "${!pids[@]}"; do
  status=0
  wait "${pids[index]}" || status=$?
  if (( status != 0 && status != 124 && status != 137 )); then
    printf '%s: echo exited %s; inspect %s\n' "${names[index]}" "$status" "$output/${names[index]}.stderr.txt" >&2
    result=1
  elif [[ ! -s "$output/${names[index]}.yaml" ]]; then
    if [[ "${types[index]}" == GripperStatus ]]; then
      echo 'gripper-events: no samples; expected when start_hand:=false' >&2
      continue
    fi
    printf '%s: no samples received; inspect ROS discovery and publisher\n' "${names[index]}" >&2
    result=1
  fi
done
trap - INT TERM EXIT
echo "Capture saved: $output"
exit "$result"
