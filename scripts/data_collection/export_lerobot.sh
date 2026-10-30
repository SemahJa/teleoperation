#!/usr/bin/env bash
# Offline read-only raw-bag export. No hardware launch, deletion, or Hub upload.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
data_root=$(dirname -- "$root")/data
python=${LEROBOT_EXPORT_PYTHON:-$data_root/.venvs/lerobot-0.4.4/bin/python}
output=${LEROBOT_OUTPUT_ROOT:-$data_root/panda_cable_clutch_trimmed_lerobot_v3_20261028}
if [[ $(docker inspect -f '{{.State.Running}}' real-demonstration 2>/dev/null || true) == true ]]; then
  echo 'Stop collection before offline dataset export.' >&2; exit 1
fi
[[ -x "$python" ]] || { echo 'Prepare the isolated export environment; see src/recording/LEROBOT_EXPORT.md.' >&2; exit 1; }
source /opt/ros/humble/setup.bash
export PYTHONPATH="$root/src/recording:$root/src/demonstration_timing:$root/install/upstream_integration_20261028/quest_teleop_interfaces/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$root/install/upstream_integration_20261028/quest_teleop_interfaces/lib:${LD_LIBRARY_PATH:-}"
export HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 PYTHONDONTWRITEBYTECODE=1
export HF_HOME="$data_root/.cache/huggingface" OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
exec "$python" -m recording.export_lerobot --input-root "$root/log/real_demonstrations" --output-root "$output" "$@"
