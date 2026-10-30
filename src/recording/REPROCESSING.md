# All cable recordings: exporter revision 3

This workflow reads original MCAPs, preserves earlier datasets, and writes a new
versioned bundle. It never launches hardware or training. LeRobot's **file format
remains v3.0**; revision 3 identifies this exporter and its supervision rules.

```bash
cd ~/aljabnouni/franka_teleoperation_cameras
bash scripts/data_collection/reprocess_all_demonstrations.sh \
  --output ~/aljabnouni/data/panda_cable_reprocessed_v3_20261030_r1
```

Choose a **new output name** for an independent repeat, for example `_r2`.
`--resume` continues an interrupted bundle without replacing validated source
outputs. An unfinished source/group directory is preserved and reported rather
than overwritten. Raw MCAPs, source metadata/manifests, older datasets and videos
are never changed. The existing private LeRobot 0.4.4 environment is required;
see [dependency setup](LEROBOT_EXPORT.md). No dependencies are installed into ROS.

The wrapper requires hardware collection to be stopped. Per-source exceptions
are inventoried and other recordings continue. Corrupted metadata is also retained
as an inventory error. Archive-only identities cause an explicit discovery error
rather than being mistaken for already-extracted recordings.

## Inventory, labels and outputs

Discovery searches recording roots in the camera checkout, original checkout,
policy checkout and `~/aljabnouni/data/log`. ZIP indexes are inspected for extra
MCAP episode identities. Synthetic/non-Menu MCAPs and duplicate locations are
reported separately. Extra recording roots can be passed with `--roots`.

Historical indices are resolved through the earlier 20261028 export's source-ID
provenance, not modification times or directory enumeration. The resolved mapping
is also versioned in `config/historical_cable_outcomes_20261028.json` as a fallback
when that export is unavailable. The operator's direct-success, recovery,
failure and partial labels remain distinct. Historical 6, 7 and additional 22
have unknown outcomes; unknown does not mean failure.

The eight 30 October recordings are ordered by the exact MCAP start timestamps.
The first and second chronological groups retain separate identities. The idle
fourth recording stays in the inventory and produces no invented frames.

The bundle contains:

- `inventory.json`: every source path, ID, exact start time, original duration,
  camera serials, original validation status, outcome, source hashes, conversion
  status, trimming summary and supervision audit.
- `source_datasets/<original-ID>/`: independently validated LeRobot datasets.
  A bad source cannot poison another source's export.
- `historical/`, `oct30_first_four/`, `oct30_second_four/`: official LeRobot
  aggregations of the usable sources, without merging episode boundaries.
- `combined_index.json`: deterministic membership across all groups, including
  idle/failed sources and their reasons. **No default training subset is selected.**
- `validation_summary.json`: source integrity, numeric-plan reproducibility,
  converted/idle/failed counts and unrepresented native transitions.
- `work/`: hash-checked parsed caches, source plans, numeric arrays and error
  evidence. Replanning compares numeric content/dtypes/shapes, independent of
  ZIP/MP4 filesystem timestamps. Byte-identical container files are not promised.

Original source numbers in this workflow are global inventory indices, preserving
historical 0–22 and assigning the new recordings 23–30. They differ from the local
0–3 numbering used by the earlier first-four-only export. Use original IDs and
`combined_index.json`, not legacy hardcoded trainer labels, when selecting data.

Aggregation uses LeRobot's own 0.4.4 aggregation API. Depth sidecars and native
ledgers are linked/copied between immutable derived datasets. No links are used
to modify raw MCAP content. Treat published datasets as immutable.

## Independent arm and gripper supervision

The policy schema is unchanged:

- `observation.state`: 14 float32 values: seven measured joint positions in rad,
  measured base-frame EE xyz in m, quaternion **xyzw**.
- `action`: eight float32 values: base-frame Cartesian target xyz/xyzw followed
  by raw right-thumbstick horizontal **intent**, never width or measured force.
- `action_valid`: eight independent component masks. The first seven share arm
  validity; the eighth expresses gripper-input validity.
- `observation.images.wrist`: RGB 640×480; the existing trainer resizes it to
  224×224 with INTER_AREA and applies ImageNet normalization/history stacking.

Recorded controller output/receipt timestamps determine valid arm targets. A
later disarmed watchdog repeat cannot erase an earlier applied target. Ambiguous
command payloads sharing the same source identity/timestamp remain masked.

A valid gripper input does not require an arm trigger, valid controller pose
tracking, an active arm reference, or the arm to be outside homing. Measured arm
observations remain available during gripper-only activity. Missing arm targets
use explicitly masked placeholders; **those placeholders are not hold commands**.
Unavailable gripper intent similarly has a false mask, rather than a fabricated
neutral command. Auxiliary arm/gripper-valid fields repeat the mask semantics.

Activity is observed arm motion, non-neutral gripper intent, or an observed
unfinished open/close operation. Keep 100 ms before and 200 ms after activity;
remove other idle time. Split at removed intervals and timestamp/session faults.
Never join segments to build an action horizon.

The gripper convention is audited against the recorded request-status evidence:

- Right x **negative closes**, positive opens.
- Engage magnitude 0.4, return-to-neutral magnitude 0.2, with hysteresis.
- Neutral stops unfinished finger motion; it **does not open** the gripper.
- A confirmed grasp persists through neutral and input loss until explicit open.
- A recorded closing intent can be rejected by the worker, for example when
  grasp force was unconfigured. That is a data-quality flag, not evidence of an
  executed close. Provided task outcome labels are not overwritten by this flag.

Every decoded Quest input, Cartesian command, gripper status, joint-state message
and camera frame-metadata message is preserved in compressed native JSONL under
`provenance/native_inputs/`. These contain native source/header and bag-receipt
timestamps. GripperStatus counters/operation/busy fields provide request evidence,
but there is no dedicated complete SDK-request/physical-execution timestamp
contract. Neither operator intent nor request status is a width measurement.

Sparse completed-operation width readbacks remain separate observations with
validity and age. No interpolation fabricates continuous width or contact force.
The legacy `source.gripper_status_ros_ns` refers to the recent width readback's
status timestamp. Busy/holding snapshots have their own validity/age and their
exact status messages remain in the native ledger. Positive commanded-force
fields are commands, not force-sensor readings.

## Timing and missing data

The action grid is 30 Hz on the host-monotonic reference. Arm states/issued
commands are past-only and bounded to 50 ms; raw Quest inputs are past-only and
bounded to 100 ms. Valid command source ≤ receipt ≤ issue ≤ anchor is checked.
ROS camera headers use bounded measured clock maps; no extrapolation is allowed.
Matching within 50 ms is provisional and does **not** establish exposure sync.
Capture uncertainty and extrinsic calibration remain unverified.

Physical camera roles are established by recorded serials: wrist D435i
`238222073675`, scene D435 `934222072657`. Old topic names are reversed; newer
names are corrected. The exporter maps serials to roles rather than blindly
swapping names.

Wrist RGB and valid measured robot state are required for a vision-training row.
Scene RGB is optional and explicitly masked via `observation.rgb_valid`; a missing
scene placeholder is not a recorded image. Missing aligned depth is separately
masked, with original uint16 values saved losslessly in indexed NPZ sidecars.
Metric depth scale is not assumed.

Native non-neutral runs and return-to-neutral transitions are checked against
retained rows. Runs without eligible camera/state observations, or sub-frame
pulses that cannot be represented faithfully at 30 Hz, remain in native ledgers
and are **reported**, never silently changed to zero or assigned a fabricated
image. In particular, historical episode 8 contains gripper operations before
wrist RGB began. Input/header gaps and frame-number metadata are reported; they
do not alone distinguish publisher losses from recorder losses.

## Read-only trainer validation

After conversion, using the same private environment and ROS paths from the
wrapper:

```bash
# No policy is constructed and no training runs.
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/src/recording:$PWD/src/demonstration_timing:${PYTHONPATH:-}"
HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 PYTHONDONTWRITEBYTECODE=1 \
  ~/aljabnouni/data/.venvs/lerobot-0.4.4/bin/python \
  scripts/data_collection/audit_reprocessed_datasets.py \
  ~/aljabnouni/data/panda_cable_reprocessed_v3_20261030_r1
```

This writes `supervision_validation.json` and `inventory.csv`, checks the actual
`robot_policy_learning` Catalog/Windows code, and samples its exact wrist-image
preprocessing/state/action dimensions. It also reports per-component statistics,
legacy H16 eligibility, non-neutral labels lost by that filtering, and operation
sign/force-configuration evidence. The trainer directory is read-only; resized
validation caches are written inside the new bundle.

Required trainer/inference work is **not implemented here**:

1. Replace all-eight-components-valid window filtering with independent target
   masks and component-masked diffusion loss, also honoring episode padding.
   Keep gripper-only targets. Do not make unknown arm targets valid.
2. Recompute normalization from valid targets **per component**, including
   gripper-only frames. All-valid-window statistics can collapse gripper to zero.
3. Select explicit IDs/outcomes/groups. Update legacy two-source fine-tune CLI
   assumptions. Compare the selected policy feature contract rather than every
   auxiliary feature against the older schema.
4. The inference adapter currently rejects non-neutral gripper predictions and
   requires exclusive ownership. Add a commissioned policy-to-Hand input path
   with timestamp/freshness checks, neutral gating, stop semantics and persistent
   grasp behavior. Do not merely remove the neutral-only safety guard.

The dataset can be format-compatible while the existing trainer still cannot
learn gripper-only actions. No such training claim is made by these exports.

## Deployment dependency and publication scope

The lab wrapper reuses `install/upstream_integration_20261028` for the updated
QuestControllerState/GripperStatus ABI. Prepare that interface overlay before a
fresh replay installation. This data-tooling publication does not finalize or
include the active worktree's pending controller/integration merge. The tested
publication branch is isolated so unrelated staged work remains intact.
