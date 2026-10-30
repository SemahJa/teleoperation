#!/usr/bin/env python3
"""Versioned all-recording inventory, independent conversion, and grouped LeRobot export."""
import argparse
from collections import Counter
from copy import deepcopy
from datetime import datetime,timezone
import hashlib,json,os,shutil,sys,traceback,zipfile
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import yaml
from recording import export_lerobot as core

LABELS={'direct_success':[2,3,4,5,8,10,12,13,14,16,17,18,21], 'success_after_recovery':[1,9,11,19], 'failure':[0,15], 'partial_success':[20], 'unknown':[6,7]}
SCHEMA=3


def historical_mapping(reference):
    mapping={}
    for p in sorted((reference/'provenance/sources').glob('*.json')):
        d=json.loads(p.read_text());i=int(d['source_number']);identity=d['source_id']
        if identity in mapping and mapping[identity]!=i:raise ValueError('Conflicting historical identity')
        mapping[identity]=i
    if not mapping:
        published=Path(__file__).resolve().parents[2]/'src/recording/config/historical_cable_outcomes_20261028.json'
        if published.exists():mapping={r['episode_id']:int(r['historical_index']) for r in json.loads(published.read_text())['episodes']}
    if not mapping:raise ValueError('Historical identity reference missing; filesystem order is not label evidence')
    if len(set(mapping.values()))!=len(mapping):raise ValueError('Historical indices ambiguous')
    return mapping


def discover(roots,reference):
    old=historical_mapping(reference);found={};duplicates=[];archives=[]
    for root in roots:
        if not root.exists():continue
        for base,dirs,files in os.walk(root,followlinks=False):
            dirs[:]=sorted(d for d in dirs if d not in ['.git','build','install','node_modules','.export_work','.venvs'])
            names=sorted(f for f in files if f.endswith('.mcap') and not (Path(base)/f).is_symlink())
            if not names:continue
            p=Path(base);identity=p.name
            # Only real Menu-recorder cable episodes; unrelated fixtures stay in discovery evidence.
            if not identity.startswith('quest_'):duplicates.append(dict(path=str(p),classification='non_demonstration_mcap'));continue
            files_to_hash=[p/n for n in names]
            files_to_hash += [f for f in [p/'metadata.yaml',p/'episode_manifest.json',Path(str(p)+'.manifest.json')] if f.exists()]
            inventory={str(f):dict(bytes=f.stat().st_size,sha256=core.sha(f)) for f in files_to_hash}
            discovery_error=None
            try:
                md=yaml.safe_load((p/'metadata.yaml').read_text())['rosbag2_bagfile_information'] if (p/'metadata.yaml').exists() else {}
                manifest=json.loads((p/'episode_manifest.json').read_text()) if (p/'episode_manifest.json').exists() else {}
            except Exception as exc:
                md={};manifest={};discovery_error='Unreadable recording metadata: '+str(exc)
            start=int(md.get('starting_time',{}).get('nanoseconds_since_epoch',manifest.get('first_bag_receipt_ros_ns',0)))
            seconds,ns=divmod(start,10**9);stamp=datetime.fromtimestamp(seconds,timezone.utc).strftime('%Y-%m-%dT%H:%M:%S')+f'.{ns:09d}Z'
            row=dict(episode_id=identity,path=str(p),recorded_start_ns=start,recorded_start_utc=stamp,
                original_duration_s=md.get('duration',{}).get('nanoseconds',0)/1e9,camera_device_info=manifest.get('camera_device_info',{}),
                original_validation_status=manifest.get('validation_status','unknown'),inventory=inventory,conversion_status='pending',outcome='unknown',success=None,default_training_selected=False)
            if discovery_error:row.update(conversion_status='failed',error=discovery_error,timestamp_valid=False,group='unclassified_corrupt')
            if identity in found:
                before=found[identity];a=sorted(v['sha256'] for k,v in before['inventory'].items() if k.endswith('.mcap'));b=sorted(v['sha256'] for k,v in inventory.items() if k.endswith('.mcap'))
                if a!=b:raise ValueError('Same episode ID has different MCAP content: '+identity)
                duplicates.append(dict(path=str(p),duplicate_of=before['path'],episode_id=identity));continue
            if identity in old:
                i=old[identity];row['historical_index']=i
                row['outcome']=next((label for label,indices in LABELS.items() if i in indices),'unknown')
                row['success']=True if row['outcome'] in ['direct_success','success_after_recovery'] else False if row['outcome']=='failure' else None
                row['group']='historical'
            found[identity]=row
    rows=sorted(found.values(),key=lambda r:(r['recorded_start_ns'],r['episode_id']))
    newest=[r for r in rows if r['recorded_start_utc'].startswith('2026-10-30') and 'historical_index' not in r]
    for i,r in enumerate(newest):r.update(new_oct30_chronological_index=i,group='oct30_first_four' if i<4 else 'oct30_second_four' if i<8 else 'additional')
    for i,row in enumerate(rows):row.update(source_number=i,group=row.get('group','additional'))
    for archive in [reference.parent.parent/'franka_teleoperation_cameras.zip',reference.parent/'franka_teleoperation_cameras.zip']:
        if not archive.exists():continue
        with zipfile.ZipFile(archive) as z:
            entries=[f for f in z.infolist() if f.filename.endswith('.mcap')]
            ids=sorted({Path(f.filename).parent.name for f in entries})
            unseen=set(ids)-set(found)
            archives.append(dict(path=str(archive),mcap_count=len(entries),episode_ids=ids,archive_only_episode_ids=sorted(unseen)))
            if unseen:raise ValueError('Archive-only recordings discovered; extract them to a separate immutable source directory before conversion: '+str(sorted(unseen)))
    return rows,dict(roots=[str(r) for r in roots],duplicate_or_other_mcap_locations=duplicates,archives=archives,historical_index_source=str(reference/'provenance/sources'))


def array_digest(arrays):
    h=hashlib.sha256()
    for key in sorted(arrays):
        a=arrays[key];h.update(key.encode());h.update(str(a.dtype).encode());h.update(str(a.shape).encode());h.update(a.tobytes())
    return h.hexdigest()


def link_tree(source,destination):
    for p in source.rglob('*'):
        if not p.is_file():continue
        target=destination/p.relative_to(source);target.parent.mkdir(parents=True,exist_ok=True)
        if target.exists():continue
        try:os.link(p,target)
        except OSError:shutil.copy2(p,target)


def finish_metadata(root,args,plans,segments):
    result=core.validate_dataset(args,segments,plans);core.dump(root/'export_validation.json',result)
    core.dump(root/'auxiliary/aligned_depth/schema.json',dict(encoding='16UC1',dtype='uint16',roles=['wrist','scene'],metric_scale_m_per_unit=None,missing_value=0,require_valid_mask=True,extrinsics_calibrated=False))
    core.dump(root/'provenance/export_parameters.json',dict(exporter_version=SCHEMA,fps=30,tolerance_ms=50,pre_context_ms=100,post_context_ms=200,
        state_dimension=14,action_dimension=8,default_training_selected=False,source_summaries=[p['summary']|dict(source_id=p['source_id'],outcome=p['outcome']) for p in plans]))
    (root/'README.md').write_text('# Versioned cable dataset\n\nIndependent arm/gripper masks; 14D measured state and 8D absolute Cartesian/joystick actions.\n'
        'Negative right-thumbstick x closes; positive opens; neutral stops unfinished motion and preserves confirmed grasp.\n'
        'Never train on masked placeholders. Existing all-components-valid H16 filtering drops gripper-only labels.\n'
        'Source outcomes are metadata; nothing is selected automatically for training. Camera capture alignment is provisional.\n'
        'See the parent inventory, validation and trainer-handoff reports. Optional absent scene/depth data are masked.\n')
    return result


def aggregate_group(bundle,group,rows):
    from lerobot.datasets.aggregate import aggregate_datasets
    selected=[r for r in rows if r['group']==group and r['conversion_status']=='converted']
    if not selected:return None
    root=bundle/group
    if (root/'export_validation.json').exists():return root
    if root.exists():raise ValueError('Preserving unfinished group output; choose a new output version: '+str(root))
    paths=[Path(r['dataset_path']) for r in selected]
    aggregate_datasets(['local/cable-source']*len(paths),'local/cable-'+group,roots=paths,aggr_root=root)
    provenance=root/'provenance';provenance.mkdir(exist_ok=True);segments=[];plans=[];offset=0
    for row,source in zip(selected,paths):
        records=json.loads((source/'provenance/segments.json').read_text())
        for record in records:
            record=dict(record);previous=record['dataset_episode_index'];record['dataset_episode_index']+=offset
            link_tree(source/'auxiliary/aligned_depth'/f'episode_{previous:06d}',root/'auxiliary/aligned_depth'/f"episode_{record['dataset_episode_index']:06d}")
            segments.append(record)
        offset+=len(records)
        for directory in ['sources','source_manifests','native_inputs']:link_tree(source/'provenance'/directory,provenance/directory)
        plan=json.loads((bundle/'work'/(row['episode_id']+'.json')).read_text());plan.update(outcome=row['outcome'],success=row['success'],source_group=row['group']);plans.append(plan)
    core.dump(provenance/'segments.json',segments)
    args=SimpleNamespace(output_root=root,fps=30);finish_metadata(root,args,plans,segments)
    return root


def run(args):
    bundle=args.output.resolve()
    if bundle.exists() and not args.resume:raise ValueError('Output already exists; choose a new version or --resume. No files overwritten.')
    bundle.mkdir(parents=True,exist_ok=True);work=bundle/'work';work.mkdir(exist_ok=True)
    rows,discovery=discover(args.roots,args.historical_reference)
    if (bundle/'inventory.json').exists():
        before=json.loads((bundle/'inventory.json').read_text());old={r['episode_id']:r for r in before}
        for row in rows:
            if row['episode_id'] in old:
                previous=old[row['episode_id']]
                if row['inventory']!=previous['inventory']:raise ValueError('Source content changed: '+row['episode_id'])
                for key in ['conversion_status','dataset_path','summary','supervision','numeric_digest','error','native_evidence','reproducible_numeric_plan']:
                    if key in previous:row[key]=previous[key]
    core.dump(bundle/'discovery.json',discovery);core.dump(bundle/'inventory.json',rows)
    kwargs=dict(fps=30,tolerance_ms=50.,pre_ms=100.,post_ms=200.,min_frames=2)
    for row in rows:
        if row['conversion_status'] in ['converted','idle'] or row.get('timestamp_valid') is False:continue
        identity=row['episode_id'];print(json.dumps({'processing':identity,'group':row['group'],'outcome':row['outcome']}),flush=True)
        try:
            # Reuse immutable v2 parsed state/image caches only if core.prepare verifies every source hash.
            parsed=work/(identity+'.parsed.json.gz')
            if not parsed.exists():
                for cache in args.cache_roots:
                    previous=cache/(identity+'.parsed.json.gz')
                    if previous.exists():parsed.symlink_to(previous);break
            settings=SimpleNamespace(**kwargs)
            plan=core.prepare(Path(row['path']),row['source_number'],work,settings)
            plan.update(outcome=row['outcome'],success=row['success'],source_group=row['group'])
            core.dump(work/(identity+'.json'),plan)
            with np.load(plan['npz']) as arrays:row['numeric_digest']=array_digest(arrays)
            row.update(summary=plan['summary'],supervision=plan['supervision'],native_evidence=plan['native_evidence'])
            if not plan['summary']['retained_frames']:
                activity=len(plan['supervision']['native_gripper_runs'])+plan['supervision']['native_arm_enabled_samples']+plan['supervision']['native_gripper_busy_samples']
                # No retained frames is not automatically called idle if native activity was lost.
                if activity:row.update(conversion_status='failed',error='Native gripper activity has no eligible observations; ledger retained')
                else:row.update(conversion_status='idle',error='No eligible teleoperation activity; no synthetic frames generated')
            else:
                root=bundle/'source_datasets'/identity
                if root.exists():raise ValueError('Preserving unfinished source dataset: '+str(root))
                settings.output_root=root;segments=core.build_dataset([plan],settings)
                finish_metadata(root,settings,[plan],segments)
                row.update(conversion_status='converted',dataset_path=str(root))
        except Exception as exc:
            row.update(conversion_status='failed',error=str(exc));(work/(identity+'.error.txt')).write_text(traceback.format_exc());print('SOURCE FAILED '+identity+': '+str(exc),flush=True)
        core.dump(bundle/'inventory.json',rows)
    groups={}
    for group in sorted({r['group'] for r in rows}):
        try:
            root=aggregate_group(bundle,group,rows)
            if root:groups[group]=str(root)
        except Exception as exc:
            groups[group]=dict(error=str(exc));(bundle/(group+'.error.txt')).write_text(traceback.format_exc())
    # Independent semantic replan: no use of the saved plan/NPZ, only native parsed evidence.
    for row in rows:
        if row['conversion_status']!='converted':continue
        try:
            other=work/'reproducibility';other.mkdir(exist_ok=True)
            cached=other/(row['episode_id']+'.parsed.json.gz')
            if not cached.exists():cached.symlink_to(work/(row['episode_id']+'.parsed.json.gz'))
            fresh=core.prepare(Path(row['path']),row['source_number'],other,SimpleNamespace(**kwargs))
            with np.load(fresh['npz']) as arrays:digest=array_digest(arrays)
            if digest!=row['numeric_digest']:raise ValueError('Non-deterministic numeric plan')
            row['reproducible_numeric_plan']=True
        except Exception as exc:row['reproducible_numeric_plan']=False;row['reproducibility_error']=str(exc)
    changed=[]
    for row in rows:
        for path,original in row['inventory'].items():
            if core.sha(Path(path))!=original['sha256']:changed.append(path)
    core.dump(bundle/'inventory.json',rows)
    core.dump(bundle/'combined_index.json',dict(schema_version=SCHEMA,datasets=groups,default_training_selection=None,
        entries=[{k:r.get(k) for k in ['source_number','episode_id','historical_index','new_oct30_chronological_index','group','outcome','success','conversion_status','dataset_path','summary','numeric_digest']} for r in rows]))
    counts=Counter(r['conversion_status'] for r in rows)
    summary=dict(recordings=len(rows),counts=dict(counts),groups=groups,all_original_files_unchanged=not changed,changed_original_files=changed,
        all_numeric_plans_reproducible=all(r.get('reproducible_numeric_plan',False) for r in rows if r['conversion_status']=='converted'),
        gripper_only_frames=sum(r.get('supervision',{}).get('gripper_only_non_neutral_frames',0) for r in rows),
        unrepresented_native_gripper_runs=sum(r.get('supervision',{}).get('unrepresented_gripper_runs',0) for r in rows),
        unrepresented_neutral_edges=sum(r.get('supervision',{}).get('unrepresented_neutral_edges',0) for r in rows))
    core.dump(bundle/'validation_summary.json',summary)
    trainer=getattr(args,'trainer_repo',None)
    if trainer is not None and trainer.is_dir() and counts['converted']:
        try:
            from audit_reprocessed_datasets import audit
            from reprocessing_report import write_report
            audit(bundle,trainer);write_report(bundle,Path(__file__).resolve().parents[2])
        except Exception as exc:
            summary['post_validation_error']=str(exc);core.dump(bundle/'validation_summary.json',summary)
            (bundle/'post_validation.error.txt').write_text(traceback.format_exc());return 2
    print(json.dumps(summary,indent=2),flush=True)
    return 0 if not counts['failed'] and not changed and summary['all_numeric_plans_reproducible'] else 2


def main():
    p=argparse.ArgumentParser(description=__doc__)
    home=Path.home()/'aljabnouni';repo=Path(__file__).resolve().parents[2]
    p.add_argument('--trainer-repo',type=Path,default=home/'robot_policy_learning');p.add_argument('--output',type=Path,required=True);p.add_argument('--resume',action='store_true')
    p.add_argument('--roots',nargs='+',type=Path,default=[repo/'log',home/'franka_teleoperation/log',home/'data/log',home/'franka_teleoperation_policy/log'])
    p.add_argument('--historical-reference',type=Path,default=home/'data/panda_cable_clutch_trimmed_lerobot_v3_20261028')
    p.add_argument('--cache-roots',nargs='*',type=Path,default=[home/'data/.export_work/panda_cable_clutch_trimmed_lerobot_v3_20261028',home/'data/.export_work/panda_cable_20261030_first4_clutch_trimmed_lerobot_v3'])
    raise SystemExit(run(p.parse_args()))


if __name__=='__main__':main()
