import json
from types import SimpleNamespace
import numpy as np
import pytest
from recording import export_lerobot as core
from recording.supervision import choose_command,summarize_supervision
from demonstration_timing.clocks import ClockModel


def test_later_disarmed_repeat_does_not_erase_applied_target():
    key=('session',0,1)
    command=dict(source=100,valid=True,pose=[0.,0.,.3,0.,0.,0.,1.])
    assert choose_command({key:[command,dict(command,valid=False)]},key,100)==command
    assert choose_command({key:[command,command]},key,100)==command
    assert choose_command({key:[command,dict(command,pose=[1.,0.,.3,0.,0.,0.,1.])]},key,100) is None
    assert choose_command({key:[command]},key,101) is None


@pytest.mark.parametrize('homing,motion',[(True,False),(False,True)])
def test_gripper_survives_arm_homing_pose_invalid_and_missing_scene(tmp_path,monkeypatch,homing,motion):
    bag=tmp_path/'episode';bag.mkdir();(bag/'metadata.yaml').write_text('test');(bag/'x.mcap').write_bytes(b'test')
    manifest=dict(status='bag_finalized',camera_device_info={'wrist':{'serial_number':'238222073675'},'scene':{'serial_number':'934222072657'}})
    (bag/'episode_manifest.json').write_text(json.dumps(manifest));work=tmp_path/'plans';work.mkdir()
    times=1_000_000_000+np.arange(20,dtype=np.int64)*10**9//30
    state=[];quest=[];images=[]
    for i,t in enumerate(times):
        state.append(dict(t=int(t),identity=('session',0),key=('session',0,i),issue=int(t)-1,receipt=int(t)-2,source=int(t)-3,control=int(t),cycle=i,motion=motion,requested_valid=motion,reference_valid=motion,homing=homing,fallback=False,hardware=True,q=[0.]*7,dq=[0.]*7,pose=[.3,0.,.3,0.,0.,0.,1.]))
        direction=-1 if 4<=i<8 else 1 if 12<=i<16 else 0
        quest.append(dict(t=int(t),identity=('session',0),valid=True,pose_valid=False,trigger=0.,trigger_active=True,x=float(direction),stick=True,stick_direction=direction,acquisition=int(t),location=int(t),uncertainty=100))
        images.append(dict(t=int(t),header=int(t),receipt=int(t),ordinal=i,frame_id='wrist'))
    streams=dict(state=state,quest=quest,gripper=[],wrist_rgb=images,wrist_depth=[],scene_rgb=[],scene_depth=[])
    model=ClockModel('ros','mono','0',int(times[0]),int(times[0]),1.,0,int(times[0]),int(times[-1]),'test')
    monkeypatch.setattr(core,'read_episode',lambda *a:(streams,{},[model],{},{}))
    def native(*a):
        ledger=a[2];ledger.write_bytes(b'native-test')
        return quest,[],{},dict(ledger=str(ledger),decode_errors={})
    monkeypatch.setattr(core,'read_native',native)
    args=SimpleNamespace(fps=30,tolerance_ms=50,pre_ms=100,post_ms=200,min_frames=2)
    plan=core.prepare(bag,0,work,args)
    with np.load(plan['npz']) as a:
        assert not a['action_valid'][:,:7].any()
        assert a['action_valid'][:,7].all()
        assert (a['action'][:,7]<0).sum()==4 and (a['action'][:,7]>0).sum()==4
        assert not a['observation.rgb_valid'][:,1].any()
        assert a['observation.state'].shape[1]==14
        assert a['action'].shape[1]==8
    assert plan['supervision']['unrepresented_gripper_runs']==0
    assert plan['supervision']['gripper_only_non_neutral_frames']==8
    first={k:v.copy() for k,v in np.load(plan['npz']).items()}
    other=tmp_path/'replan';other.mkdir();(other/'episode.parsed.json.gz').symlink_to(work/'episode.parsed.json.gz')
    repeated=core.prepare(bag,0,other,args)
    with np.load(repeated['npz']) as arrays:
        for key in first:np.testing.assert_array_equal(first[key],arrays[key])


def test_unrepresented_native_transition_is_reported_not_fabricated():
    q=[dict(t=100,identity=('a',0),valid=True,stick=True,stick_direction=-1,x=-1.),dict(t=110,identity=('a',0),valid=True,stick=True,stick_direction=0,x=0.)]
    result=summarize_supervision(dict(quest=q,gripper=[]),{},[])
    assert result['unrepresented_gripper_runs']==1
    assert result['native_gripper_runs'][0]['unrepresented_reason']
    assert result['samples_30hz']['closing']==0
