#!/usr/bin/env bash
# Native camera + USB Quest + mock pose + existing Menu recorder; no robot owners.
set -e
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-137}
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=${ROS_LOG_DIR:-$root/log/demonstration_launch}
mkdir -p "$ROS_LOG_DIR"
# Build exactly these packages from this worktree; never use another checkout's overlay.
colcon --log-base "$root/log/demonstration_build" build \
 --base-paths src/quest_teleop_interfaces src/quest_teleop src/camera_bringup \
 src/demonstration_timing src/recording src/teleop_bringup \
 --build-base "$root/build/demonstration" --install-base "$root/install/demonstration" \
 --cmake-args -DBUILD_TESTING=ON
source "$root/install/demonstration/local_setup.bash"
# Explicit, process-local camera transport option; no sysctl/global ROS changes.
case "${CAMERA_TRANSPORT:-default}" in
 default) ;;
 large_shm)
  if [[ ${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp} != rmw_fastrtps_cpp ]]; then
   echo 'CAMERA_TRANSPORT=large_shm requires rmw_fastrtps_cpp.' >&2; exit 1
  fi
  export FASTRTPS_DEFAULT_PROFILES_FILE="$root/src/camera_bringup/config/fastdds_local_large_frames.xml"
  # XML explicitly restricts UDP to127.0.0.1 and supplies larger SHM transport.
  # Humble appends default transports when ROS_LOCALHOST_ONLY=1; avoid that here.
  export ROS_LOCALHOST_ONLY=0
  ;;
 *) echo 'CAMERA_TRANSPORT must be default or large_shm.' >&2; exit 1 ;;
esac
python3 - <<'PY'
import rosbag2_py
assert 'mcap' in rosbag2_py.get_registered_writers(), 'Install ros-humble-rosbag2-storage-mcap'
PY
port=${QUEST_PORT:-45454}
if ss -Hltn "sport = :$port" | read -r _; then
 echo "Port $port already in use. Stop the previous pipeline first." >&2; exit 1
fi
if command -v adb >/dev/null; then adb_cmd=(adb)
else adb_cmd=(docker exec "${QUEST_ADB_CONTAINER:-docker-quest-adb-server-1}" adb); fi
if [[ -n ${QUEST_SERIAL:-} ]]; then serial=$QUEST_SERIAL
else
 mapfile -t devices < <("${adb_cmd[@]}" devices | awk '$2=="device" {print $1}')
 [[ ${#devices[@]} == 1 ]] || { echo 'Set QUEST_SERIAL to one authorized ADB device.' >&2; exit 1; }
 serial=${devices[0]}
fi
adb_cmd+=(-s "$serial")
forward_added=false
if ! "${adb_cmd[@]}" reverse --list | awk -v p="tcp:$port" '$2==p && $3==p {found=1} END {exit !found}'; then
 if "${adb_cmd[@]}" reverse --list | awk -v p="tcp:$port" '$2==p {found=1} END {exit !found}'; then
   echo 'Conflicting ADB reverse exists; preserve it and choose another QUEST_PORT.' >&2; exit 1
 fi
 "${adb_cmd[@]}" reverse "tcp:$port" "tcp:$port"
 forward_added=true
fi
cleanup() { if $forward_added; then "${adb_cmd[@]}" reverse --remove "tcp:$port" || true; fi; }
trap cleanup EXIT
"${adb_cmd[@]}" shell am start -n org.franka.questteleop/android.app.NativeActivity
printf '%s\n' 'Wear/focus the Quest app and wake both controllers.' \
 'Release left Menu, press once to start; press again to stop. Wait for validation before restarting.' \
 'Stop this launch with Ctrl-C after recording finishes.'
ros2 launch recording demonstration.launch.py listen_port:="$port" \
 output_root:="${DEMONSTRATION_OUTPUT_ROOT:-$root/log/demonstrations}" "$@"
