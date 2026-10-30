# Cable LeRobot export — 2026-10-28

Completed offline in the camera worktree; no robot commands, commits or uploads.

| Item | Result |
| --- | --- |
| Source recordings | 23 original MCAP episodes, preserved |
| Contributing sources | 21; source episodes 6 and 7 had no eligible activity |
| Derived LeRobot episodes | 192 continuous segments |
| Frames | 21,781 at 30 Hz; 12 min 6 s |
| Removed grid positions | 7,598 / 29,379 (25.86%); idle or unusable intervals |
| Dataset size | Approximately 1.75 GB, including lossless depth sidecars |
| RGB views | Physical wrist D435i 238222073675; physical scene D435 934222072657 |
| Video frames | Exactly 21,781 per view; H.264 CRF 18 |
| Depth | uint16 NPZ sidecars; 19 wrist / 32 scene missing matches, explicitly masked |
| Recent gripper readbacks | 126 frames; other widths masked unknown |
| Automated package tests | 74 passed, including 10 new export/selection tests |
| Official dataset API | LeRobot 0.4.4, format v3.0; full export passed loader checks |
| Original source content | All inventoried MCAP/metadata/manifest SHA-256 values unchanged |

Output: `/home/mirmi/aljabnouni/data/panda_cable_clutch_trimmed_lerobot_v3_20261028`.

The exporter retains gripper-only activity, plus 100 ms before and 200 ms after
activity. It splits at removed intervals and invalid segments. Recording can
start during an already-held trigger: the recorded controller gate is reused.
Targets are recorded Cartesian xyz/quaternion commands and thumbstick intent;
they are not joint commands or measured grasp force. Use `action_valid` for each
future target and the loader's episode-end padding mask. No state interpolation
or continuous gripper measurement is invented.

RGB/depth are provisionally matched by mapped headers within 50 ms. Maximum
absolute RGB match offsets are 22.28 ms (wrist) and 17.37 ms (scene); maximum
inter-camera header difference is 26.51 ms. These values do **not** establish
exposure synchronization. Native timestamps, bounded host clock models and
uncertainty flags remain available. Extrinsics and metric depth scale remain
unverified. Success labels are unknown: review demonstrations before training.

Reproduce from the camera worktree after stopping collection:

```bash
# Inspect selection without writing another dataset.
LEROBOT_OUTPUT_ROOT=~/aljabnouni/data/cable_export_v2 \
  bash scripts/data_collection/export_lerobot.sh --prepare-only

# Export to a new directory; existing datasets are never overwritten.
LEROBOT_OUTPUT_ROOT=~/aljabnouni/data/cable_export_v2 \
  bash scripts/data_collection/export_lerobot.sh

# Recheck the completed default dataset, including source hashes.
bash scripts/data_collection/export_lerobot.sh --validate-only

# Full recorder + timing regression suite; launches no hardware.
source /opt/ros/humble/setup.bash
source install/upstream_integration_20261028/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="$PWD/src/recording:$PWD/src/demonstration_timing:$PWD/src/quest_teleop:${PYTHONPATH:-}" \
  python3 -m pytest -q -p no:cacheprovider src/recording/test src/demonstration_timing/test
```

See `src/recording/LEROBOT_EXPORT.md` for dependencies, loader examples and masks.
Dataset provenance includes source plans/manifests, segment-to-source mapping,
selection statistics, package versions and `export_validation.json`.

Files added for this export: `scripts/data_collection/export_lerobot.sh`,
`src/recording/recording/export_lerobot.py`, `src/recording/recording/clutch_filter.py`,
`src/recording/test/test_export_lerobot.py`, `src/recording/test/test_clutch_filter.py`,
`src/recording/LEROBOT_EXPORT.md`, and this report.
Updated `DATA_COLLECTION.md` and recording's `CMakeLists.txt`. Other existing
worktree changes and the original checkout are preserved.
