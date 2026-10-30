import numpy as np
from recording.clutch_filter import latched_axis, TemporalIndex, context_mask,continuous_segments


def test_release_gate_and_hysteresis():
    values=[-.8,0,-.8,-.3,0,.8,.3,0]
    direction=latched_axis(values,[True]*8,['a']*8,np.arange(8)*20_000_000)
    assert direction.tolist()==[0,0,-1,-1,0,1,1,0]


def test_disconnect_and_new_session_require_neutral():
    values=[0,-.8,-.8,-.8,0,-.8]
    result=latched_axis(values,[True,True,False,True,True,True],['a']*6,np.arange(6)*20_000_000)
    assert result.tolist()==[0,-1,0,0,0,-1]
    result=latched_axis([0,-.8,-.8],[True]*3,['a','a','b'],np.arange(3)*20_000_000)
    assert result.tolist()==[0,-1,0]


def test_pause_splits_and_no_delta_window_crossing():
    keep=[True,True,False,False,True,True,True]
    assert continuous_segments(keep,['same']*7)==[(0,2),(4,7)]
    assert continuous_segments([True]*4,['a','a','b','b'])==[(0,2),(2,4)]


def test_gripper_context_can_be_kept_without_arm_trigger():
    t=np.arange(10)*100_000_000
    active=np.array([False]*4+[True]+[False]*5)
    keep=context_mask(t,active,100_000_000,200_000_000)
    assert np.flatnonzero(keep).tolist()==[3,4,5,6]


def test_time_reset_ambiguity_and_past_only_actions():
    idx=TemporalIndex([{'t':100,'identity':'a'},{'t':200,'identity':'a'},{'t':100,'identity':'b'},{'t':300,'identity':'b'}])
    assert idx.pick(150,100) is None
    idx=TemporalIndex([{'t':100},{'t':200}])
    assert idx.pick(180,100,causal=True)['t']==100
    assert idx.pick(180,100)['t']==200
    assert idx.pick(400,50) is None


def test_bag_can_start_during_valid_motion_but_not_after_disconnect():
    from recording.clutch_filter import recorded_arm_gate
    assert recorded_arm_gate(True,1.,True)
    assert recorded_arm_gate(True,.5,True)  # Recorded controller holds hysteresis.
    assert not recorded_arm_gate(True,0.,True)
    assert not recorded_arm_gate(False,1.,True)
    assert not recorded_arm_gate(True,1.,False)
    result=latched_axis([-.8,-.8,-.8,0,-.8],[True,False,True,True,True],['a']*5,np.arange(5)*20_000_000,initial_neutral_required=False)
    assert result.tolist()==[-1,0,0,0,-1]
