#!/usr/bin/env bash
# Existing export environment; hardware collection must be stopped.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ $(docker inspect -f '{{.State.Running}}' real-demonstration 2>/dev/null || true) == true ]]; then
  echo 'Hardware collection is running; offline reprocessing deferred.' >&2; exit 1
fi
source /opt/ros/humble/setup.bash
export PYTHONPATH="$root/src/recording:$root/src/demonstration_timing:$root/install/upstream_integration_20261028/quest_teleop_interfaces/local/lib/python3.10/dist-packages:$root/install/upstream_integration_20261028/franka_msgs/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$root/install/upstream_integration_20261028/quest_teleop_interfaces/lib:$root/install/upstream_integration_20261028/franka_msgs/lib:${LD_LIBRARY_PATH:-}"
export HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
export HF_HOME="$(dirname -- "$root")/data/.cache/huggingface"
exec "$(dirname -- "$root")/data/.venvs/lerobot-0.4.4/bin/python" "$root/scripts/data_collection/reprocess_all_demonstrations.py" "$@"
