#!/usr/bin/env bash
# Starts REAL torque control and an empty-hand calibration. Run only in a cleared lab session.
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
original=$(dirname -- "$root")/franka_teleoperation
adb_container=${QUEST_ADB_CONTAINER:-docker-quest-adb-server-1}
serial=${QUEST_SERIAL:-2G0YC5ZH1L00T2}
if ss -Hltn 'sport = :45454' | read -r _; then echo 'Port45454 is occupied; stop the previous pipeline first.' >&2; exit 1; fi
[[ -f "$root/install/hardware_demo/local_setup.bash" ]] || { echo 'Build the isolated hardware_demo prefix first.' >&2; exit 1; }
adb_cmd=(docker exec "$adb_container" adb -s "$serial")
[[ $("${adb_cmd[@]}" get-state) == device ]]
added=false
if ! "${adb_cmd[@]}" reverse --list | awk '$2=="tcp:45454" && $3=="tcp:45454" {found=1} END {exit !found}'; then
 if "${adb_cmd[@]}" reverse --list | awk '$2=="tcp:45454" {found=1} END {exit !found}'; then echo 'Conflicting ADB reverse; preserve it and inspect.' >&2; exit 1; fi
 "${adb_cmd[@]}" reverse tcp:45454 tcp:45454
 added=true
fi
cleanup() { if $added; then "${adb_cmd[@]}" reverse --remove tcp:45454 || true; fi; }
trap cleanup EXIT
"${adb_cmd[@]}" shell am start -n org.franka.questteleop/android.app.NativeActivity
echo 'REAL arm + Hand startup: keep trigger/grip released. Empty gripper calibration moves fingers.'
echo 'Wait for Arm control ready, Hand calibration ready, both camera streams, and valid Quest input.'
echo 'Menu starts/stops recording. Confirm /quest/recording_status before moving. Right A requests homing.'
docker run --rm --init --name real-demonstration --network host --ipc host  --user "$(id -u):$(id -g)" --group-add 46 --cap-add SYS_NICE  --ulimit rtprio=99 --ulimit memlock=-1 --device-cgroup-rule 'c 189:* rmw'  -v /dev/bus/usb:/dev/bus/usb -v "$original:/ws:ro" -v "$root:/camera_ws:rw"  -w /camera_ws --entrypoint /bin/bash franka-demonstration-recording:phase5 -lc  'set -e; source /camera_ws/scripts/data_collection/environment.sh; exec ros2 launch /camera_ws/scripts/data_collection/pipeline.launch.py runtime_config:=/camera_ws/scripts/data_collection/hardware.yaml operator_confirm:=true'
