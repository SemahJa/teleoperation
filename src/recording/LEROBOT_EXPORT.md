# Export cable demonstrations to LeRobot

This is an **offline, read-only export**. Original MCAP bags, manifests and timing
files remain in `log/real_demonstrations`. Nothing is uploaded or deleted.

For all-recording, outcome-aware exporter revision 3, use [REPROCESSING.md](REPROCESSING.md).
The existing v3.0 file format and 14D/8D policy features are retained.

## Run

Stop collection first, then:

```bash
cd ~/aljabnouni/franka_teleoperation_cameras
bash scripts/data_collection/export_lerobot.sh --prepare-only
bash scripts/data_collection/export_lerobot.sh
```

Default output: `~/aljabnouni/data/panda_cable_clutch_trimmed_lerobot_v3_20261028`.
Existing outputs are refused. For another export:

```bash
LEROBOT_OUTPUT_ROOT=~/aljabnouni/data/cable_export_v2 \
  bash scripts/data_collection/export_lerobot.sh
```

Plans are cached under `data/.export_work/`; reuse requires identical parameters
and source hashes. `--max-source-episodes 1` limits a smoke export. Parameters:
`--fps 30 --tolerance-ms 50 --pre-ms 100 --post-ms 200 --min-frames 2`.
Do not change or append to the source recordings while exporting.

To export an exact subset without including older or reserved recordings, repeat
`--episode-id <exact-directory-ID>`. Explicit selections are sorted by the MCAP
start timestamps in `metadata.yaml`, rather than filesystem modification times.
Unknown or duplicate IDs, and mixing IDs with `--max-source-episodes`, are refused.
`source.episode_number` is numbered locally from zero in the selected chronological
order; it must not be confused with labels in an older dataset. For example:

```bash
LEROBOT_OUTPUT_ROOT=~/aljabnouni/data/panda_cable_20261030_first4_clutch_trimmed_lerobot_v3 \
  bash scripts/data_collection/export_lerobot.sh \
  --episode-id quest_20261030T101332Z_f24ad625 \
  --episode-id quest_20261030T101434Z_b202e655 \
  --episode-id quest_20261030T101528Z_8d982cac \
  --episode-id quest_20261030T101629Z_4603230a
```

## Format and meaning

The official **LeRobot 0.4.4** writer creates the v3 dataset: metadata and data
Parquet, indexed MP4 video, episode statistics, tasks and episode boundaries.
The official `LeRobotDataset` loader verifies both images and padded action
windows at every segment boundary. This is a local dataset; its repository ID
is only a loader identifier, not an upload.

- `observation.images.wrist`, `.scene`: RGB, 640x480, 30 Hz; H.264 CRF 18,
  lossy derived copies. Raw RGB is retained in MCAP.
- `observation.state`: seven measured joint angles (rad), measured end-effector
  xyz (m), quaternion xyzw, in the original recorded robot frame.
- `observation.joint_velocity`: seven measured rad/s values.
- `action`: seven recorded Cartesian target xyz/xyzw values, then raw right
  thumbstick x **intent**. This is neither a joint action nor measured force.
- `action_valid`: per-component mask. Invalid arm actions contain xyz zero and
  identity quaternion; consumers MUST honor the mask. A standard policy trainer
  does not automatically apply custom masks; configure the loss/filter explicitly.
- `observation.trigger`, `.arm_motion_enabled`: recorded trigger and active arm gate.
- `observation.gripper_width`: sparse readbacks after observed completed operations,
  trusted for at most 100 ms after the status event. Width zero with validity zero
  means unknown. The status timestamp is not an encoder acquisition timestamp.
  Grasp status is meaningful only with the width-valid mask. No force is measured.
- `observation.depth_valid`: wrist/scene depth availability. Lossless uint16 depth
  is in `auxiliary/aligned_depth/episode_NNNNNN/block_NNNNNN.npz`, addressed by
  `source.depth_block` and `source.depth_offset`. This optional sidecar requires
  custom access; it is not a built-in LeRobot RGB video feature. Missing depth is
  zero-filled with a false mask. **Metric depth scale is unverified**; preserve
  integers and obtain the sensor scale before using them in 3D training.

## Pause selection and alignment

Exporter revision 3 uses the recorded controller motion/output gate as arm activity authority;
a missing raw observer sample does not erase a valid issued command.
Retain gripper joystick activity and in-progress open/close operations even when
arm control is clutched. Keep 100 ms before and 200 ms after activity. A fresh
neutral is required after unavailable input, session changes or input gaps.
Recording may begin with an already engaged trigger: the recorded motion gate
preserves that controller state; a bag boundary does not create a new release gate.
Remove other idle intervals. Split every retained continuous interval into a new
LeRobot episode. Do not join them again for action horizons; the loader pads at
boundaries. Short validity gaps can also split episodes, and two-frame segments
are allowed by default.

Sample at 30 Hz. Joint states and applicable commands use causal, past-only
matching, at most 50 ms old. Command source <= receipt <= issue <= anchor time;
these are recorded host control times, not actuator acknowledgments. Quest input
is at most 100 ms old. RGB/depth use nearest headers within 50 ms, allowing repeats
or future headers relative to the anchor. Wrist RGB is required; missing scene RGB and aligned depth are explicitly masked. No robot-state interpolation is performed. Derived state/action values use float32;
source timestamp fields remain integer nanoseconds, and full-precision original
ROS messages stay in the source bags.

Camera headers are mapped through bounded Phase 3 ROS-to-host-monotonic clock
models. Ambiguous/reset clocks and disconnected input are rejected. Never infer
capture accuracy from host clock-fit error. The 50 ms matching tolerance is
**provisional alignment, not synchronized exposures**. Native timestamps, deltas,
clock models, camera calibration messages, source episode IDs and hashes remain
in `source.*` fields and `provenance/`. Original timing is not overwritten by the
compressed LeRobot timeline. Extrinsics are uncalibrated.

Historical camera role names are corrected by recorded serial number:
physical wrist D435i `238222073675`, physical scene D435 `934222072657`.
Unknown/ambiguous serials cause an error rather than swapping silently.

The standalone exporter leaves outcomes unknown. The all-recording workflow preserves
operator-provided outcome labels; failures/recoveries/partial successes remain present.
No subset is automatically selected for training.

## Load locally

Use the private Python environment (PyAV backend avoids TorchCodec dependencies):

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ds = LeRobotDataset(
    "SemahJa/panda_cable_clutch_trimmed",
    root="/home/mirmi/aljabnouni/data/panda_cable_clutch_trimmed_lerobot_v3_20261028",
    video_backend="pyav", download_videos=False,
    delta_timestamps={
        "action": [0., 1/30, 2/30],
        "action_valid": [0., 1/30, 2/30],
    },
)
item = ds[0]
print(item["observation.images.scene"].shape, item["action"], item["action_valid"])
# Honor action_is_pad at episode ends and action_valid for every future target.
```

## Isolated dependencies

Python 3.10 environment: `~/aljabnouni/data/.venvs/lerobot-0.4.4`.
It includes system site packages for ROS Humble message bindings, while private
NumPy 1.26.4 avoids the NumPy 2 ABI conflict. No native ROS installation is replaced.
Only the dataset portion of LeRobot is installed; training/robot-control extras
are intentionally omitted. Core dependency versions used for each export are in
`provenance/environment.txt`; private-environment package pins are in
`provenance/requirements-local.txt`. ROS system packages remain supplied by Ubuntu.

To recreate on the same ROS installation (use a fresh environment path):

```bash
python3 -m pip install --target /tmp/lerobot-venv-bootstrap 'virtualenv<21'
PYTHONPATH=/tmp/lerobot-venv-bootstrap python3 -m virtualenv --system-site-packages \
  ~/aljabnouni/data/.venvs/lerobot-0.4.4
p=~/aljabnouni/data/.venvs/lerobot-0.4.4/bin/python
"$p" -m pip install torch==2.7.1+cpu torchvision==0.22.1+cpu \
  --index-url https://download.pytorch.org/whl/cpu
"$p" -m pip install --no-deps lerobot==0.4.4
"$p" -m pip install numpy==1.26.4 pandas==2.2.3 pyarrow==21.0.0 datasets==4.1.1 \
  huggingface-hub==0.35.3 accelerate==1.10.1 av==15.1.0 Pillow==10.4.0 \
  opencv-python-headless==4.11.0.86 draccus==0.10.0 jsonlines==4.0.0 \
  packaging==25.0 scipy==1.15.3 einops==0.8.1 termcolor==3.1.0 setuptools==80.9.0
```

The wrapper supplies the existing ROS interface overlay. Rebuild that overlay
first if it is absent. This exporter does not start cameras, Quest or the Panda.

## Verification

`export_validation.json` records official loader checks and raw hash verification.
Rerun those checks with `bash scripts/data_collection/export_lerobot.sh --validate-only`.
`provenance/segments.json` maps every derived episode back to source times and
source episode number. `provenance/export_parameters.json` records retention and
rejection counts. Selection tests:

```bash
PYTHONPATH=src/recording:src/demonstration_timing python3 -m pytest -q src/recording/test/test_clutch_filter.py src/recording/test/test_export_lerobot.py
```

## Concatenated review videos

Create one full scene video and one full wrist video, with source episode/segment
labels and chapter markers. Both follow the same trimmed frame timeline:

```bash
python3 scripts/data_collection/export_lerobot_videos.py \
  ~/aljabnouni/data/panda_cable_clutch_trimmed_lerobot_v3_20261028
```

Outputs go to the sibling `panda_cable_clutch_trimmed_lerobot_v3_20261028_review/`
directory: `scene_all_episodes.mp4`, `wrist_all_episodes.mp4`,
`episode_index.csv` and `review_manifest.json`. Use `--output <new-directory>`
for a different destination. Existing directories are refused. This requires
FFmpeg/FFprobe with the subtitles filter; ROS and LeRobot Python dependencies
are unnecessary. Videos are re-encoded H.264 CRF 18 review copies, and every
output frame is decoded during verification. The dataset remains unchanged.
Matching playback timelines do not establish camera exposure synchronization.
