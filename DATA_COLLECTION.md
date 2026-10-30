# Collect one demonstration

For the prepared lab setup in `~/aljabnouni/franka_teleoperation_cameras`.
Requires the existing `franka-demonstration-recording:phase5` image,
`install/hardware_demo`, and versioned `scripts/data_collection/` runtime files.
The Docker image and build/install folders must be prepared for a fresh clone; see
[deployment details](src/recording/REAL_DEMONSTRATION_TEST.md) for building them.

New demonstration episodes use **lossless Zstd Fast compression**. Camera resolution,
FPS, original messages and timestamps are unchanged. The validated sample used
about **1.3GB/min** after compression; other scenes may differ. See
[compression results](src/recording/COMPRESSION_RESULTS.md).

## 1. Prepare

- Panda must be cleared for motion, FCI enabled, and ready (not **User stopped**).
- Empty the gripper and clear its full finger travel and the arm workspace.
- Connect both cameras and an ADB-authorized Quest by USB. Close **RealSense Viewer**.
- Keep Quest trigger and grip released. Startup moves the fingers for calibration.

## 2. Start — terminal 1

```bash
cd ~/aljabnouni/franka_teleoperation_cameras

# Optional: keep the Quest app active without wearing the headset.
docker exec docker-quest-adb-server-1 adb shell am broadcast \
  -a com.oculus.vrpowermanager.prox_close

bash scripts/data_collection/start_hardware.sh
```

This starts **real arm control**, gripper, both RGB-D cameras, Quest bridge/mapper,
and the Menu recorder. Wait for **Arm control ready**, **Hand calibration ready**,
and **RealSense Node Is Up!** for both cameras. Controller tracking must be valid;
keep the unworn headset upright with its cameras facing the controllers.

If you encounter the following error: Port 45454 is already bound ( due a stale ADB’s Quest-side port forwarding entry from a previous session)
With the previous pipeline stopped, clear only this forwarding entry:
```bash
docker exec docker-quest-adb-server-1 \
  adb -s 2G0YC5ZH1L00T2 reverse --remove tcp:45454

# If it reports “listener not found” try:
docker exec docker-quest-adb-server-1 \
  adb -s 2G0YC5ZH1L00T2 reverse --list

ss -ltnp 'sport = :45454'
```
## 3. Record exactly once

1. Press and release **left Menu once**. Confirm a **new episode** reports
   `phase: recording` using the status command below. Wait two seconds.
2. Perform the demonstration. Keep the headset able to track the controllers.
3. Release trigger/grip, then press and release **left Menu once** to stop.
4. Wait for `readable: true` and `validation_errors: []` before stopping the launch.
   Validation can take several minutes. Do not press Menu again: it starts another episode.

| Right controller | Action |
| --- | --- |
| Release index trigger, then hold it | Engage arm movement |
| Release index trigger | Hold measured joint position |
| Hold side grip / release it | Close gripper / stop closing |
| Click thumbstick once, with grip released | Open to the configured gap (currently 25 mm) |
| **A button** | Requests homing; avoid during collection |

## 4. Check status — terminal 2

Run again as needed. This reads files and does not control the robot.

```bash
cd ~/aljabnouni/franka_teleoperation_cameras
python3 - <<'PY'
import json
from pathlib import Path
files = Path('log/real_demonstrations').glob('*.manifest.json')
latest = max(files, key=lambda p: p.stat().st_mtime, default=None)
if latest is None:
    print('No episode yet.')
else:
    data = json.loads(latest.read_text())
    print('episode:', data.get('episode_id', latest.name))
    for key in ('phase', 'readable', 'validation_errors'):
        print(f'{key}:', json.dumps(data.get(key)))
PY
```

`readable: null` means validation results are not available yet. `phase: idle` alone is not
proof of completed validation. If validation reports errors, preserve the raw bag
and inspect the episode's `.validation.log`.

## 5. Stop and find the data

After validation passes, press **Ctrl+C in terminal 1**. To stop from another terminal:

```bash
docker kill --signal=SIGINT real-demonstration
```

Episodes are saved under `log/real_demonstrations/quest_<UTC-time>_<id>/`:

- `*.mcap`: original RGB/depth, camera metadata, robot, Quest and gripper messages.
- `episode_manifest.json`: topic counts, validation results and warnings.
- `timing_records.jsonl` and `alignment_provisional.json`: offline timing/alignment.
- `videos/`: playback MP4s and timestamp CSVs **only if separately exported**.

Sparse gripper measurements retain missing-data masks. Provisional timestamp
matching does not establish camera exposure synchronization; extrinsics remain uncalibrated.

To restore normal headset-wearing behavior:

```bash
docker exec docker-quest-adb-server-1 adb shell am broadcast \
  -a com.oculus.vrpowermanager.automation_disabled
```

## Export a dataset after collection

To remove trigger-release idle periods while keeping gripper activity, create a
separate strict LeRobot dataset without changing raw recordings:

```bash
bash scripts/data_collection/export_lerobot.sh
```

For all-recording, outcome-aware exporter revision 3 and grouped dataset assembly:

```bash
bash scripts/data_collection/reprocess_all_demonstrations.sh \
  --output ~/aljabnouni/data/panda_cable_reprocessed_v3_20261030_r1
```

See [LeRobot export](src/recording/LEROBOT_EXPORT.md) and [Reprocessing](src/recording/REPROCESSING.md)
for dependencies, feature masks, source timestamps and output locations. Review success labels before training.
