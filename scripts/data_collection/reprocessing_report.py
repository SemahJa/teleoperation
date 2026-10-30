"""Small reproducibility artifacts and human-readable inventory; no training."""
from datetime import datetime,timezone
import hashlib,importlib.metadata,json,subprocess,sys
from pathlib import Path


def write_report(bundle,repo,publication_commit=None):
    from recording import export_lerobot as core
    inventory=json.loads((bundle/'inventory.json').read_text());summary=json.loads((bundle/'validation_summary.json').read_text())
    audit_path=bundle/'supervision_validation.json';audit=json.loads(audit_path.read_text()) if audit_path.exists() else {}
    groups=summary['groups'];source_reports={r['episode_id']:r for r in audit.get('source_reports',[])}
    # Validate labels and describe lossless depth independently of legacy static report wording.
    roots=[Path(r['dataset_path']) for r in inventory if r['conversion_status']=='converted']+[Path(p) for p in groups.values() if isinstance(p,str)]
    for root in roots:
        plans=[json.loads(p.read_text()) for p in (root/'provenance/sources').glob('*.json')]
        segments=json.loads((root/'provenance/segments.json').read_text());labels=core.validate_source_labels(segments,plans)
        report=json.loads((root/'export_validation.json').read_text());report.update(outcome_labels_preserved=True,outcomes_by_contributing_source=labels,success_labels='operator-provided where available; unknown otherwise; never an automatic training selection')
        core.dump(root/'export_validation.json',report)
        core.dump(root/'auxiliary/aligned_depth/schema.json',dict(encoding='16UC1',dtype='uint16',roles=['wrist','scene'],metric_scale_m_per_unit=None,missing_value=0,require_valid_mask=True,extrinsics_calibrated=False))
    code_paths=['src/recording/recording/export_lerobot.py','src/recording/recording/supervision.py','src/recording/recording/clutch_filter.py','scripts/data_collection/reprocess_all_demonstrations.py','scripts/data_collection/audit_reprocessed_datasets.py','scripts/data_collection/reprocessing_report.py','src/recording/config/historical_cable_outcomes_20261028.json']
    code_hashes={p:hashlib.sha256((repo/p).read_bytes()).hexdigest() for p in code_paths}
    packages={name:importlib.metadata.version(name) for name in ['lerobot','numpy','torch','torchvision','datasets','pyarrow','pandas','av','opencv-python-headless']}
    def git(*args):
        p=subprocess.run(['git','-C',str(repo),*args],text=True,capture_output=True)
        return p.stdout.strip() if p.returncode==0 else None
    environment=dict(exporter_revision=3,lerobot_file_format='v3.0',python=sys.version,packages=packages,code_sha256=code_hashes,
        working_tree_HEAD=git('rev-parse','HEAD'),preexisting_pending_merge=git('rev-parse','--verify','MERGE_HEAD'),publication_commit=publication_commit,
        timestamp_utc=datetime.now(timezone.utc).isoformat(),training_performed=False,
        actual_command='bash scripts/data_collection/reprocess_all_demonstrations.sh --output '+str(bundle),
        reproducibility_scope='Source membership, outcomes, action/state numeric arrays, masks, timestamps, image references and segment boundaries. Containers may contain varying timestamps.')
    core.dump(bundle/'reproducibility.json',environment)
    frozen=subprocess.check_output([sys.executable,'-m','pip','freeze','--local'],text=True)
    (bundle/'requirements-local.txt').write_text(frozen)
    for root in roots:core.dump(root/'provenance/environment.json',environment)
    total_frames=sum(r.get('summary',{}).get('retained_frames',0) for r in inventory)
    counts={k:sum(r.get('supervision',{}).get('samples_30hz',{}).get(k,0) for r in inventory) for k in ['opening','closing','neutral','invalid']}
    lines=['# Cable reprocessing report — exporter revision 3','',f"Recordings: **{len(inventory)}**; converted: **{summary['counts'].get('converted',0)}**; idle: **{summary['counts'].get('idle',0)}**; conversion failures: **{summary['counts'].get('failed',0)}**.",
        f"Retained: **{total_frames:,} frames at 30 Hz ({total_frames/30:.3f} seconds)**. Gripper intent: {counts['opening']} opening, {counts['closing']} closing, {counts['neutral']} neutral, {counts['invalid']} invalid grid samples.",
        f"Gripper-only non-neutral frames retained: **{summary['gripper_only_frames']}**. Original file hashes unchanged: **{summary['all_original_files_unchanged']}**. Numeric plans reproducible: **{summary['all_numeric_plans_reproducible']}**.",'',
        'Opening/closing/neutral counts use 0.4 engage / 0.2 neutral hysteresis. The actual exported action channel remains the unchanged continuous raw right-thumbstick x. Intent is not measured width, force or proof of execution.','',
        '| Source | Original episode ID | Outcome | Group | Conversion | Before → after (s) | Frames | Open / close / neutral |',
        '| --- | --- | --- | --- | --- | --- | ---: | --- |']
    for row in inventory:
        s=row.get('summary',{});c=row.get('supervision',{}).get('samples_30hz',{})
        lines.append(f"| {row['source_number']} | `{row['episode_id']}` | {row['outcome']} | {row['group']} | {row['conversion_status']} | {row['original_duration_s']:.3f} → {s.get('retained_seconds',0):.3f} | {s.get('retained_frames',0)} | {c.get('opening',0)} / {c.get('closing',0)} / {c.get('neutral',0)} |")
    lines += ['', '## Group datasets','']
    for name,path in groups.items():lines.append(f'- {name}: `{path}`')
    lines += ['','`combined_index.json` covers all sources and selects none automatically. `inventory.csv` and `inventory.json` retain exact timestamps, paths, labels, masks, gap evidence and errors.','',
        '## Data-quality and supervision findings','',
        '- Historical 6 and 7, and the fourth 30 October recording (global source 26), are idle. They remain inventoried without fake frames or replacements.',
        '- Historical episode 8 lacks wrist RGB before approximately 28.32 s. Three gripper intent runs at 18.87–20.89, 21.37–21.92 and 24.53–25.94 s cannot appear in a wrist-aligned visual row. All native inputs remain in its native ledger; no images or commands were fabricated.',
        '- Historical episode 0 reports closing disabled because grasp force was unconfigured. Its closing intent is preserved and explicitly distinguished from accepted operation evidence.',
        '- Native gripper statuses expose operation counters and status construction times, not a dedicated complete SDK request or physical execution timestamp contract. Commanded force is not a force measurement.',
        '- Source gaps/frame numbers are diagnostic evidence; publisher versus recorder loss attribution is not established by a gap alone. Camera exposure sync, extrinsics and metric depth scale remain unverified.',
        '- Outcome labels are preserved exactly where supplied. Unknown outcomes, failures and recoveries are not automatically selected for training.','',
        '## Required trainer and inference work','',
        '- Existing `robot_policy_learning` Catalog/Windows read the datasets with 14D state, 8D actions, 640×480 wrist RGB and 224×224 ResNet input preprocessing.',
        '- The legacy all-components-valid H16 window filter retains **zero non-neutral gripper target frames** in these recordings. It must be replaced with independent per-component validity and padding-aware loss/windowing; never mark unknown arm targets valid.',
        '- Normalize each action component from its valid targets, including gripper-only intervals. The legacy fully-valid normalization can collapse the gripper channel to neutral.',
        '- Select explicit original IDs/source groups; update old DIRECT/two-new-source assumptions and whole-auxiliary-schema equality checks.',
        '- The current inference adapter rejects non-neutral gripper predictions. A commissioned policy-to-Hand path must preserve sign, neutral/stop, persistent grasp, freshness and exclusive ownership. Do not just remove its safety guard.',
        '- No policy was constructed, no training ran, and no physical robot or gripper command was issued.','',
        '## Reproduce','',
        '```bash','cd ~/aljabnouni/franka_teleoperation_cameras',
        'bash scripts/data_collection/reprocess_all_demonstrations.sh \\',
        '  --output ~/aljabnouni/data/panda_cable_reprocessed_v3_20261030_r2','```','',
        'Use a new version name. Existing validated outputs and original recordings are not overwritten. See `src/recording/REPROCESSING.md` for dependencies, limits and source-selection semantics.','']
    (bundle/'REPROCESSING_REPORT.md').write_text('\n'.join(lines))
    handoff=dict(dataset_paths=groups,combined_index=str(bundle/'combined_index.json'),default_training_selection=None,policy_state_dimension=14,policy_action_dimension=8,
        wrist_feature='observation.images.wrist',image_shape=[3,480,640],trainer_image_shape=[3,224,224],history=2,horizon=16,
        action_representation='absolute_base_xyz_quaternion_xyzw_raw_right_thumbstick_x',gripper_sign='negative_close_positive_open',gripper_neutral='stops unfinished motion; preserves confirmed grasp',
        native_ledger='provenance/native_inputs/*.jsonl.gz',required_changes=audit.get('required_trainer_changes',[]),inference_blockers=audit.get('inference_blockers',[]),training_performed=False)
    core.dump(bundle/'TRAINER_HANDOFF.json',handoff)
    return environment
