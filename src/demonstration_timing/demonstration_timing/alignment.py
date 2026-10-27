"""Offline joining with clock compatibility, uncertainty, gaps and action causality."""
import argparse
from bisect import bisect_left, bisect_right
import json
import math
from .clocks import ClockModel, ClockError, map_record

OBSERVATIONS=('wrist_rgb','wrist_depth','scene_rgb','scene_depth','robot_state','gripper_state')


def uncertainty(record):
    u=record.get('uncertainty_ns')
    return None if u is None else u+record.get('clock_mapping_uncertainty_ns',0)


def admissible(row, anchor, tolerance_ns, allow_unknown):
    if not row.get('valid',False): return False
    ru,au=uncertainty(row),uncertainty(anchor)
    if (ru is None or au is None) and not allow_unknown: return False
    return abs(row['common_ns']-anchor['common_ns'])+(ru or 0)+(au or 0)<=tolerance_ns


def pick(rows,anchor,tolerance_ns,allow_unknown,interpolate=False,max_gap_ns=100_000_000,causal=False):
    """Do not sort away arrival-order resets. Callers supply segmented mapped rows."""
    t=anchor['common_ns']
    local=[r for r in rows if (not causal or r['common_ns']<=t) and abs(r['common_ns']-t)<=tolerance_ns]
    if local and not min(local,key=lambda r:abs(r['common_ns']-t)).get('valid',False): return None
    candidates=[r for r in local if admissible(r,anchor,tolerance_ns,allow_unknown)]
    if not candidates: return None
    nearest=min(candidates,key=lambda r:abs(r['common_ns']-t))
    # Nearest matching is local; state interpolation requires consecutive valid neighbors
    # in acquisition order, same validity/session segment, and a short bounded gap.
    if interpolate:
        for a,b in zip(rows,rows[1:]):
            gap=b['common_ns']-a['common_ns']
            if not (a['common_ns']<t<b['common_ns'] and 0<gap<=max_gap_ns): continue
            if a.get('_stream_order',0)+1 != b.get('_stream_order',1): continue
            if a['segment']!=b['segment'] or not all(admissible(x,anchor,tolerance_ns,allow_unknown) for x in (a,b)): continue
            av,bv=a.get('payload',{}).get('joint_position'),b.get('payload',{}).get('joint_position')
            if not av or not bv or len(av)!=len(bv) or not all(math.isfinite(x) for x in av+bv): continue
            fraction=(t-a['common_ns'])/gap
            return {'method':'linear_joint_position','joint_position':[x+(y-x)*fraction for x,y in zip(av,bv)],
                    'source_record_ids':[a['record_id'],b['record_id']], 'fraction':fraction,
                    'provisional':any(uncertainty(x) is None for x in (anchor,a,b))}
    payload=nearest.get('payload',{})
    if 'frame_ref' in payload:
        payload={k:v for k,v in payload.items() if k!='native_metadata_json'}
    return {'method':'nearest_past' if causal else 'nearest','record_id':nearest['record_id'],
            'delta_ns':nearest['common_ns']-t,'payload':payload,
            'provisional':uncertainty(nearest) is None or uncertainty(anchor) is None}


def align(records,models,reference,anchor_stream='wrist_rgb',tolerance_ns=50_000_000,
          allow_unknown=False,interpolate=False,max_gap_ns=100_000_000):
    if not reference.startswith('host_monotonic:'): raise ClockError('action receipt/output fields require a host_monotonic reference')
    mapped=[]; rejected=[]
    previous={}; epochs={}; orders={}
    for original in records:
        try:
            row=map_record(original,models,reference)
        except ClockError as e:
            rejected.append({'record_id':original.get('record_id'),'reason':str(e)}); continue
        stream=row['stream']; old=previous.get(stream)
        identity=(row['segment'],row['clock_segment'])
        if old and (row['common_ns']<=old[0] or identity!=old[1] or not row.get('valid') or not old[2]):
            epochs[stream]=epochs.get(stream,0)+1
        row=dict(row,segment=f"{row['segment']}:{epochs.get(stream,0)}")
        previous[stream]=(row['common_ns'],identity,row.get('valid'))
        row['_stream_order']=orders.get(stream,0)
        orders[stream]=row['_stream_order']+1
        mapped.append(row)
    streams={name:[r for r in mapped if r['stream']==name] for name in {r['stream'] for r in mapped}}
    indexed={name:sorted(rows,key=lambda r:r['common_ns']) for name,rows in streams.items()}
    times={name:[r['common_ns'] for r in rows] for name,rows in indexed.items()}
    def window(name,t):
        stamps=times.get(name,[])
        return sorted(indexed.get(name,[])[bisect_left(stamps,t-tolerance_ns):bisect_right(stamps,t+tolerance_ns)],
                      key=lambda r:r['_stream_order'])
    commands_by_key={}; quest_by_key={}
    for name,destination in (('teleoperation_command',commands_by_key),('quest_state',quest_by_key)):
        for row in streams.get(name,[]): destination.setdefault(tuple(row.get('command_key',[])),[]).append(row)
    outputs=[]
    for anchor in streams.get(anchor_stream,[]):
        if not anchor.get('valid',False): continue
        t=anchor['common_ns']; observations={}
        # Ambiguous overlapping timestamps from a reset epoch are rejected instead of
        # arbitrarily choosing one recording session with the same numerical stamp.
        for name in OBSERVATIONS:
            rows=window(name,t)
            selected_stream=name
            if name=='robot_state' and not any(admissible(r,anchor,tolerance_ns,allow_unknown) for r in rows):
                rows=window('joint_states',t)
                selected_stream='joint_states'
            near=[r for r in rows if admissible(r,anchor,tolerance_ns,allow_unknown)]
            if len({r['segment'] for r in near})>1:
                observations[name]=None
            else:
                observations[name]=pick(rows,anchor,tolerance_ns,allow_unknown,
                    interpolate=interpolate and name=='robot_state',max_gap_ns=max_gap_ns,
                    causal=name=='gripper_state')
            if observations[name] is not None: observations[name]['source_stream']=selected_stream
        end=bisect_right(times.get('command_application',[]),t)
        applications=indexed.get('command_application',[])[max(0,end-1):end]
        action=None
        if applications:
            app=max(applications,key=lambda r:r['common_ns'])
            commands=[r for r in commands_by_key.get(tuple(app.get('command_key',[])),[]) if r['common_ns']<=t and r.get('valid',False)]
            if len(commands)==1 and admissible(app,anchor,tolerance_ns,allow_unknown):
                command=commands[0]
                receipt=app.get('receipt_monotonic_ns')
                if receipt is not None and command['common_ns']+(uncertainty(command) or 0)<=receipt<=app['common_ns'] and app['common_ns']+(uncertainty(app) or 0)<=t-(uncertainty(anchor) or 0):
                    if allow_unknown or (uncertainty(command) is not None and uncertainty(anchor) is not None):
                        action={'record_id':command['record_id'],'application_record_id':app['record_id'],
                                'command':command.get('payload',{}),'source_timestamp':command['timestamp_ns'],
                                'source_clock_id':command['clock_id'],'applicable_execution_timestamp':app['common_ns'],
                                'execution_semantics':'host output-issuance observation; not physical actuation acknowledgement',
                                'provisional':any(uncertainty(x) is None for x in (command,app,anchor))}
        if action:
            matching_quest=[r for r in quest_by_key.get(tuple(command.get('command_key',[])),[]) if r.get('valid',False) and r['common_ns']<=t]
            action['quest_source_record_id']=matching_quest[0]['record_id'] if len(matching_quest)==1 else None
            action['quest_controller_input']=matching_quest[0]['payload'] if len(matching_quest)==1 else None
        missing_reasons={name: ('unrecorded_or_unmapped_clock' if not (streams.get(name) or (name=='robot_state' and streams.get('joint_states'))) else
                              'unknown_uncertainty_or_tolerance_or_invalid_segment')
                         for name,value in observations.items() if value is None}
        if action is None: missing_reasons['action']='no_valid_causal_command_with_observed_receipt_and_output'
        outputs.append({'anchor_record_id':anchor['record_id'],'timestamp_ns':t,'reference_clock':reference,
                        'observation_t':observations,'action_t':action,
                        'missing_reasons':missing_reasons,'missing_data_mask':{**{k:v is None for k,v in observations.items()},'action':action is None},
                        'provisional':any(v and v.get('provisional') for v in observations.values()) or bool(action and action['provisional'])})
    return {'schema_version':1,'reference_clock':reference,'rejected_records':rejected,'observations':outputs,
            'policy':{'tolerance_ns':tolerance_ns,'allow_unknown_uncertainty':allow_unknown,
                      'state_interpolation':interpolate,'max_interpolation_gap_ns':max_gap_ns,
                      'actions':'causal source + observed controller receipt + output issuance; never future commands'}}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('input',help='small normalized timing JSONL, or diagnostic JSON containing records/models')
    p.add_argument('--models',help='JSON list of explicitly measured ClockModel objects')
    p.add_argument('--reference',required=True)
    p.add_argument('--output',required=True)
    p.add_argument('--anchor',default='wrist_rgb')
    p.add_argument('--tolerance-ms',type=float,default=50)
    p.add_argument('--max-interpolation-gap-ms',type=float,default=100)
    p.add_argument('--interpolate-state',action='store_true')
    p.add_argument('--allow-unknown-uncertainty',action='store_true')
    args=p.parse_args()
    with open(args.input) as f:
        content=f.read()
    try: document=json.loads(content)
    except json.JSONDecodeError: document=None
    records=document['records'] if isinstance(document,dict) else [json.loads(line) for line in content.splitlines() if line.strip()]
    if args.models:
        with open(args.models) as f: raw_models=json.load(f)
    else: raw_models=document.get('clock_models',[]) if isinstance(document,dict) else []
    if args.tolerance_ms<=0 or args.max_interpolation_gap_ms<=0: p.error('tolerances must be positive')
    result=align(records,[ClockModel(**m) for m in raw_models],args.reference,args.anchor,
                 round(args.tolerance_ms*1e6),args.allow_unknown_uncertainty,args.interpolate_state,
                 round(args.max_interpolation_gap_ms*1e6))
    with open(args.output,'w') as f: json.dump(result,f,separators=(',',':'),allow_nan=False)
