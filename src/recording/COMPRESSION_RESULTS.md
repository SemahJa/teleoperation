# Lossless storage optimization — 2026-10-26

Applied: existing MenuRecorder supports validated `mcap_compression` (`None`/`Zstd`)
and `mcap_compression_level` (`Fastest`, `Fast`, `Default`, `Slow`, `Slowest`).
The demonstration profile and prepared hardware/mock YAML select **Zstd Fast**.
MCAP chunk CRCs and per-message indexes stay enabled. Unspecified legacy settings
retain no compression; sqlite3 behavior is unchanged. No image resizing,
quantization, lossy image codec, frame-rate reduction, telemetry removal or robot
control changes. Native and Docker installation prefixes rebuilt; images/SDKs unchanged.

## Measured saved-episode benchmark

Source: `log/real_demonstrations/quest_20261026T132905Z_24a62bd9` (144.296s).

| Artifact | Original bytes | Compressed bytes | Method |
| --- | ---: | ---: | --- |
| MCAP |13,606,802,906|3,105,739,195|MCAP chunk Zstd Fast|
| timing_records.jsonl |1,528,704,671|103,177,374|Zstd CLI level3|
| alignment_provisional.json |59,424,124|6,053,493|Zstd CLI level3|

MCAP reduction **77.175%**, ratio **4.381:1**. Conversion took **41.756s** offline,
approximately326MB/s of source data. This is an offline conversion measurement,
not a guarantee of live-camera recording performance. Scene texture/noise changes
compression ratios. The existing uncompressed camera tests do not establish loss
performance with compression enabled. A short live compressed loss test remains
necessary; a running RealSense Viewer was left undisturbed during this task.

## Verification

All **337,880messages /28topics** retained. Compared each topic's message count,
message type, serialization format, ordered SHA256 over bag receipt timestamp,
serialized length and complete raw CDR bytes. Every digest matched. This covers
all original image pixels, depth values, source/header timestamps, robot/action/
gripper fields and camera metadata within those messages. Offered QoS metadata
also matches. Compressed JSON files were fully decompressed and hashed against
their originals. All passed. No source data deleted or rewritten.

Initial conversion without the custom ROS message overlay omitted custom topics;
the equality check rejected it. That incomplete output was removed. The accepted
conversion ran in the existing Docker environment with all original custom types.
Never treat a converter exit code alone as proof that every topic was preserved.

Verification and compressed artifacts:
`log/storage_optimization_20261026/verified/benchmark.json`, `episode_zstd/`,
`timing_records.jsonl.zst`, `alignment_provisional.json.zst`.
`episode_zstd/source_episode_manifest.json` preserves source timing/provenance;
it explicitly describes the original recording. The original complete episode,
MP4s and plain JSON files remain in their original directory. This benchmark
therefore adds an archive copy; removing originals to reclaim space is a separate
choice. Existing Phase3 readers still consume the plain JSON exports.

## Changes

- `recording/storage_options.py`: validated MCAP options with CRC/index preservation.
- `recording/configuration.py`, `recording/menu_recorder.py`: pass configured options
  to the existing rosbag2 subprocess and retain them in the episode manifest.
- `quest_teleop/runtime_configuration.py`: validate settings before child launch.
- `recording/config/demonstration.yaml`: opt-in profile now selects Zstd Fast.
- Corresponding recording/runtime tests and documentation updated.
- Prepared hardware/mock runtime YAML updated, with `.pre_zstd.yaml` copies preserved.

Tests: **24 recording pytest tests +6 runtime-configuration tests passed**.
This includes an actual compressed MCAP image round-trip preserving CDR bytes and
timestamps, configuration propagation, invalid-option rejection and legacy defaults.
Docker package results also passed after updating the optional-default expectations.
Installed native configuration resolves to Zstd Fast; every non-recording field in
the prepared hardware YAML matches its saved prior version.

## Reproduce archive conversion

Use the existing Docker image with the original checkout mounted read-only at `/ws`
and this worktree at `/camera_ws`; source
`/camera_ws/log/real_demonstration_20261026/environment.sh` so custom message types
are available. The actual YAML used is preserved in
`log/storage_optimization_20261026/verified/{mcap_zstd,convert}.yaml`.
For a new output directory, change `uri` in a COPY of convert.yaml, then run:

```bash
ros2 bag convert -i /camera_ws/log/real_demonstrations/quest_20261026T132905Z_24a62bd9 mcap -o /path/to/copied_convert.yaml
```

Do not overwrite the existing verified archive. To unpack the derived JSON while
keeping compressed files:

```bash
zstd -dk log/storage_optimization_20261026/verified/timing_records.jsonl.zst
zstd -dk log/storage_optimization_20261026/verified/alignment_provisional.json.zst
```

Future collection uses the same startup command documented in DATA_COLLECTION.md.
Inspect each new `.storage.yaml` / manifest for `Zstd`, `Fast`, CRCs and indexes.
To opt out, set `recording.mcap_compression: "None"` before restarting the recorder.
