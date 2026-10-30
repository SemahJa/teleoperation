#!/usr/bin/env python3
"""Read-only trainer compatibility audit; never construct a policy or train."""
import argparse
from collections import Counter
import csv,gzip,hashlib,json,sys
from pathlib import Path
import numpy as np


def dump(path,value):path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def conventions(parsed,plan):
    from recording.clutch_filter import TemporalIndex
    qs=parsed['streams']['quest'];index=TemporalIndex(qs);previous=None;counts=Counter();evidence=[];counter_gaps=0
    for g in parsed['streams']['gripper']:
        new=previous is not None and g.get('started',0)>previous.get('started',0)
        if new:
            counter_gaps+=max(0,g['started']-previous['started']-1)
            counts[g['operation']]+=1
            if g['operation'] in ['open','close']:
                q=index.pick(g['t'],100_000_000,causal=True);expected=1 if g['operation']=='open' else -1
                direction=q['stick_direction'] if q and q['valid'] and q['stick'] else None
                result='consistent' if direction==expected else 'opposite_direction' if direction==-expected else 'not_observable_from_status_delay_or_input'
                evidence.append(dict(operation=g['operation'],status_header_ros_ns=g['header'],status_host_ns=g['t'],quest_source_ns=q['t'] if q else None,
                    raw_quest_x=q['x'] if q else None,check=result,semantics='Status construction follows issue; not exact SDK dispatch/physical execution'))
        previous=g
    runs=[];wrist=parsed['streams'].get('wrist_rgb',[])
    for r in plan['supervision']['native_gripper_runs']:
        r=dict(r)
        if not r['covered_by_grid'] and wrist and (r['end_ns']<wrist[0]['t'] or r['start_ns']>wrist[-1]['t']):
            r['unrepresented_reason']='No recorded/mappable physical wrist RGB covers this native intent run. Original native action preserved; no image was fabricated.'
        runs.append(r)
    valid_q=[q for q in qs if q['valid']]
    seq_gaps=0;seq_nonmonotonic=0;time_gaps=0
    for a,b in zip(valid_q,valid_q[1:]):
        if a['identity']!=b['identity']:continue
        seq_gaps+=max(0,b.get('sequence',0)-a.get('sequence',0)-1)
        seq_nonmonotonic+=int(b.get('sequence',0)<=a.get('sequence',0))
        time_gaps+=int(b['t']-a['t']>100_000_000)
    return dict(observed_quest_sequence_gaps=seq_gaps,observed_quest_sequence_nonmonotonic=seq_nonmonotonic,quest_source_gaps_over_100ms=time_gaps,issued_operation_start_counts=dict(counts),unobserved_counter_increments=counter_gaps,request_sign_evidence=evidence,
        opposite_direction_status_evidence=sum(e['check']=='opposite_direction' for e in evidence),
        closing_disabled_statuses=sum('closing disabled' in g.get('message','').lower() for g in parsed['streams']['gripper']),
        failed_operation_statuses=sum('failed' in g.get('message','').lower() for g in parsed['streams']['gripper']),
        recorded_commanded_force_n=sorted({g.get('force_command_n',0.) for g in parsed['streams']['gripper']}),
        confirmed_grasp_statuses=sum(g['grasped'] for g in parsed['streams']['gripper']),native_gripper_runs=runs)


def audit(bundle,trainer_repo,partial=False):
    sys.dont_write_bytecode=True
    sys.path.insert(0,str(trainer_repo/'src'))
    from robot_policy_learning.data import Catalog,Windows,normalization,IMAGE,STATE,window_starts
    rows=json.loads((bundle/'inventory.json').read_text());reports=[];combined_action=[[] for _ in range(8)];combined_state=[];fingerprints={}
    for row in rows:
        identity=row['episode_id'];plan_file=bundle/'work'/(identity+'.json')
        result={k:row.get(k) for k in ['source_number','episode_id','group','historical_index','outcome','success','conversion_status','original_duration_s','error']}
        if not plan_file.exists():reports.append(result);continue
        plan=json.loads(plan_file.read_text())
        with gzip.open(bundle/'work'/(identity+'.parsed.json.gz'),'rt') as f:parsed=json.load(f)
        result['convention_audit']=conventions(parsed,plan)
        result.update(samples_30hz=plan['supervision']['samples_30hz'],frames=plan['summary']['retained_frames'],trimmed_duration_s=plan['summary']['retained_seconds'],segments=plan['summary']['segments'],gripper_only_non_neutral_frames=plan['supervision']['gripper_only_non_neutral_frames'],invalid_grid_reasons=plan['summary']['invalid_reasons'],native_decode_errors=plan['native_evidence']['decode_errors'])
        if row['conversion_status']!='converted':reports.append(result);continue
        catalog=Catalog(row['dataset_path']);source=row['source_number'];fingerprints[identity]=catalog.fingerprint
        assert set(catalog.source_to_segments)=={source}
        assert {s['source_episode_id'] for s in catalog.segments}=={identity}
        assert all(s['outcome']==row['outcome'] and s['success']==row['success'] for s in catalog.segments)
        assert catalog.info['features'][STATE]['shape']==[14] and catalog.info['features']['action']['shape']==[8]
        state_by_time={r['t']:r for r in parsed['streams']['state']}
        app_by_issue={r['issue']:r for r in parsed['streams']['state'] if r['issue']>0}
        masks=[];actions=[];states=[];full_windows=[];candidate_windows=[];short=0;identity_checks=0
        command_frames=set()
        with gzip.open(plan['native_evidence']['ledger'],'rt') as ledger:
            for line in ledger:
                if '"topic":"/panda_1/cartesian_teleop_command"' not in line:continue
                message=json.loads(line)['message'];command_frames.add(message['header']['frame_id'])
        if command_frames and command_frames!={'panda_1/panda_link0'}:raise ValueError('Unexpected Cartesian frame for '+identity+': '+str(command_frames))
        for e,frames in catalog.rows.items():
            mask=np.array([r['action_valid'] for r in frames],np.uint8);a=np.array([r['action'] for r in frames],np.float32);state=np.array([r[STATE] for r in frames],np.float32)
            assert np.all(mask[:,:7]==mask[:,0,None])
            for r in frames:
                if not r['action_valid'][0]:continue
                measured=state_by_time[int(r['source.robot_state_time_ns'])];applied=app_by_issue[int(r['source.command_issue_time_ns'])]
                assert measured['identity']==applied['identity'],'Cross-session target assignment'
                assert applied['source']==int(r['source.command_source_time_ns'])
                assert applied['key'][2]==int(r['source.command_sequence'])
                assert applied['key'][1]==int(r['source.command_reference_generation'])
                identity_checks+=1
            assert all(int(r['observation.arm_action_valid'])==int(r['action_valid'][0]) and int(r['observation.gripper_action_valid'])==int(r['action_valid'][7]) for r in frames)
            assert all(r['observation.rgb_valid'][0]==1 for r in frames)
            assert np.isfinite(a).all() and np.isfinite(state).all()
            assert np.allclose(np.linalg.norm(state[:,10:14],axis=1),1,atol=.01)
            arm=mask[:,0].astype(bool);assert np.allclose(np.linalg.norm(a[arm,3:7],axis=1),1,atol=.01)
            good=window_starts(mask,2,16);full_windows += [(e,s) for s in good]
            any_valid=mask.any(axis=1);starts=[s for s in range(len(frames)-15) if any_valid[s:s+16].all()];candidate_windows += [(e,s) for s in starts]
            short+=int(len(frames)<16)
            masks.append(mask);actions.append(a);states.append(state)
        mask=np.concatenate(masks);actions=np.concatenate(actions);states=np.concatenate(states);combined_state.extend(states)
        for k in range(8):combined_action[k].extend(actions[mask[:,k].astype(bool),k].tolist())
        retained={(e,j) for e,s in full_windows for j in range(s,s+16)}
        active_retained=[catalog.rows[e][j]['action'][7] for e,j in retained]
        result.update(trainer_catalog_passed=True,applied_command_identity_checks=identity_checks,cartesian_frame_ids=sorted(command_frames),legacy_full_valid_H16_windows=len(full_windows),candidate_independent_mask_H16_windows=len(candidate_windows),segments_shorter_than_H16=short,
            legacy_window_non_neutral_gripper_action_frames=sum(abs(v)>.2 for v in active_retained),legacy_window_gripper_range=[float(min(active_retained)),float(max(active_retained))] if active_retained else None,
            action_valid_component_counts=mask.sum(axis=0).astype(int).tolist())
        image_reader=Windows(catalog,[],dict(image_size=[224,224]),bundle/'audit_cache')
        for e in sorted({min(catalog.episodes),max(catalog.episodes)}):
            images=image_reader.images(e);assert images.shape==(catalog.episodes[e]['length'],224,224,3)
        result['trainer_image_decode_passed']=True
        if full_windows:
            config=dict(history=2,horizon=16);sn,an=normalization(catalog,full_windows,config)
            contract=dict(history=2,horizon=16,image_size=[224,224],image_mean=[.485,.456,.406],image_std=[.229,.224,.225],state_normalization=sn,action_normalization=an)
            data=Windows(catalog,full_windows,contract,bundle/'audit_cache')
            for i in sorted({0,len(data)-1}):
                item=data[i];assert tuple(item[STATE].shape)==(2,14) and tuple(item[IMAGE].shape)==(2,3,224,224) and tuple(item['action'].shape)==(16,8)
                assert not item['action_is_pad'].any()
            result['trainer_window_decode_passed']=True
        reports.append(result)
    group_reports={}
    for group in sorted({r['group'] for r in rows}):
        root=bundle/group
        if not (root/'export_validation.json').exists():continue
        catalog=Catalog(root)
        expected={r['episode_id']:r for r in rows if r['group']==group and r['conversion_status']=='converted'}
        assert {s['source_episode_id'] for s in catalog.segments}==set(expected)
        assert all(s['outcome']==expected[s['source_episode_id']]['outcome'] and s['success']==expected[s['source_episode_id']]['success'] for s in catalog.segments)
        group_reports[group]=dict(trainer_catalog_passed=True,frames=catalog.info['total_frames'],segments=catalog.info['total_episodes'],sources=sorted(catalog.source_to_segments),fingerprint=catalog.fingerprint)
    action_stats=[]
    for values in combined_action:
        a=np.asarray(values,np.float32)
        action_stats.append(dict(count=len(a),min=float(a.min()),max=float(a.max()),mean=float(a.mean()),std=float(a.std())) if len(a) else dict(count=0,min=None,max=None,mean=None,std=None))
    contract_files=[trainer_repo/'src/robot_policy_learning/data.py',trainer_repo/'configs/baseline.json']
    trainer_hashes={str(p.relative_to(trainer_repo)):hashlib.sha256(p.read_bytes()).hexdigest() for p in contract_files}
    result=dict(trainer_contract_files_sha256=trainer_hashes,schema_version=3,partial=partial,trainer_read_only=True,training_performed=False,policy_constructed=False,source_reports=reports,groups=group_reports,
        independent_component_action_statistics=action_stats,source_fingerprints=fingerprints,
        required_trainer_changes=['Build windows without requiring all eight components valid; retain gripper-only targets and apply action_valid per component and action_is_pad per timestep. Never join segments.',
            'Recompute action normalization per component using valid targets, including gripper-only rows. Existing all-valid normalization can collapse the gripper range to neutral.',
            'Select explicit source IDs/outcomes/groups. Legacy DIRECT indices and two-source fine-tune CLI must not silently determine the new experiment.',
            'Validate selected feature contracts rather than requiring every auxiliary feature to equal older dataset schemas. Keep state 14D, action 8D and wrist preprocessing unchanged.'],
        inference_blockers=['franka_policy_inference/actions.py currently rejects abs(action[7]) > .001; a commissioned policy-gripper command path is required, not deletion of the guard.',
            'Preserve negative-close/positive-open, .4 engage/.2 neutral hysteresis, stop of unfinished motion, persistent confirmed grasp, freshness and exclusive ownership. Do not treat neutral as open or intent as measured width/force.'])
    name='audit_preview.json' if partial else 'supervision_validation.json';dump(bundle/name,result)
    if not partial:
        columns=['source_number','episode_id','recorded_start_utc','group','outcome','conversion_status','original_duration_s','trimmed_duration_s','frames','segments','opening','closing','neutral','invalid','gripper_only_non_neutral_frames','unrepresented_gripper_runs','legacy_full_valid_H16_windows','legacy_non_neutral_gripper_frames','closing_disabled_statuses','failed_operation_statuses','confirmed_grasp_statuses','opposite_direction_status_evidence','observed_quest_sequence_gaps']
        byid={r['episode_id']:r for r in reports}
        with (bundle/'inventory.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=columns);w.writeheader()
            for row in rows:
                report=byid[row['episode_id']];s=row.get('supervision',{});counts=s.get('samples_30hz',{})
                entry={k:row.get(k,'') for k in columns};entry.update({k:report.get(k,'') for k in ['trimmed_duration_s','frames','segments','gripper_only_non_neutral_frames']})
                entry.update({k:counts.get(k,0) for k in ['opening','closing','neutral','invalid']});entry['unrepresented_gripper_runs']=s.get('unrepresented_gripper_runs',0)
                convention=report.get('convention_audit',{})
                entry.update({k:convention.get(k,0) for k in ['closing_disabled_statuses','failed_operation_statuses','confirmed_grasp_statuses','opposite_direction_status_evidence','observed_quest_sequence_gaps']})
                entry['legacy_full_valid_H16_windows']=report.get('legacy_full_valid_H16_windows',0);entry['legacy_non_neutral_gripper_frames']=report.get('legacy_window_non_neutral_gripper_action_frames',0);w.writerow(entry)
    print(json.dumps(dict(partial=partial,audited=len(reports),converted=sum(r.get('trainer_catalog_passed',False) for r in reports),groups=group_reports),indent=2),flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bundle',type=Path);p.add_argument('--trainer-repo',type=Path,default=Path.home()/'aljabnouni/robot_policy_learning');p.add_argument('--partial',action='store_true');a=p.parse_args();audit(a.bundle,a.trainer_repo,a.partial)
