import json
import struct
import pytest
from demonstration_timing.clocks import ClockError, ClockModel, HostClockBridge, fit_clock, map_record
from demonstration_timing.metrics import stream_metrics, nearest_differences, SegmentTracker
from demonstration_timing.alignment import align
from demonstration_timing.adapters import adapt
from demonstration_timing.diagnostics import image_header

REF='host_monotonic:test:boot'

def sample(stream,t,**kw):
    return dict(record_id=kw.pop('record_id',stream+str(t)),stream=stream,timestamp_ns=t,clock_id=REF,
                clock_segment='0',segment=kw.pop('segment','0'),valid=kw.pop('valid',True),
                uncertainty_ns=kw.pop('uncertainty_ns',0),payload=kw.pop('payload',{}),**kw)


def test_known_offset_and_drift_preserves_native():
    pairs=[(10**18+i*10**9,7*10**12+round(i*10**9*1.00002)) for i in range(8)]
    model=fit_clock(pairs,'camera:measured',REF,'a',100)
    assert model.rate==pytest.approx(1.00002)
    row=sample('wrist_rgb',pairs[3][0]); row.update(clock_id='camera:measured',clock_segment='a')
    mapped=map_record(row,[model],REF)
    assert mapped['common_ns']==pairs[3][1]
    assert mapped['timestamp_ns']==pairs[3][0]
    assert row.get('common_ns') is None
    assert mapped['clock_mapping_uncertainty_ns']==100


@pytest.mark.parametrize('pairs',[[(0,10),(1,11),(0,12)],[(0,0),(1,1),(2,10**9)]])
def test_clock_reset_or_step_refused(pairs):
    with pytest.raises(ClockError): fit_clock(pairs,'x',REF,'0',1)


def test_unmeasured_clock_and_extrapolation_refused():
    row=sample('wrist_rgb',100); row['clock_id']='different_device'
    with pytest.raises(ClockError): map_record(row,[],REF)
    model=fit_clock([(0,10),(10,20),(20,30)],'different_device',REF,'0',0)
    with pytest.raises(ClockError): model.convert(21,'different_device','0')
    with pytest.raises(ClockError): model.convert(10,'different_device','reset')
    with pytest.raises(ClockError): fit_clock([(0,0),(1,1),(2,2)],'x',REF,'0',None)


def test_host_clock_bracket_discontinuity():
    b=HostClockBridge()
    a=b.observe(100,1000,110)
    assert a['uncertainty_ns']==5 and a['offset_monotonic_minus_ros_ns']==-895
    assert b.observe(200,1100,210)['segment']=='0'
    assert b.observe(300,10,310)['discontinuity']


def test_different_camera_fps_nearest_and_missing_frames():
    wrist=list(range(0,1_000_000_000,33_333_333)); scene=list(range(0,1_000_000_000,66_666_666))
    assert len(nearest_differences(wrist,scene,34_000_000))==len(wrist)
    rows=[dict(header_ros_ns=t,arrival_ros_ns=t+1000,frame_number=n) for t,n in [(0,1),(33_333_333,2),(99_999_999,4),(99_999_999,4),(50,1)]]
    m=stream_metrics(rows,30)
    assert m['estimated_missing_from_header_gaps']==1
    assert m['counter_gaps']==1 and m['counter_duplicates']==1 and m['counter_resets_or_reorder']==1
    assert m['non_monotonic']==1 and m['duplicated_stamps']==1


def test_segment_boundaries():
    s=SegmentTracker(max_gap_ns=100)
    assert s.observe('x',1,'session1')[0]=='0'
    assert s.observe('x',2,'session1')[0]=='0'
    assert s.observe('x',1,'session1')[1]=='timestamp_reset_or_reorder'
    assert s.observe('x',200,'session1')[1]=='gap'
    assert s.observe('x',201,'session2')[1]=='identity_or_domain_change'
    assert s.observe('x',202,'session2',False)[1]=='invalid_boundary'


def test_sparse_joint_interpolation():
    rows=[sample('robot_state',0,payload={'joint_position':[0.,2.]}),
          sample('robot_state',100,payload={'joint_position':[10.,4.]}),sample('wrist_rgb',50)]
    result=align(rows,[],REF,tolerance_ns=100,interpolate=True,max_gap_ns=200)['observations'][0]
    assert result['observation_t']['robot_state']['joint_position']==[5.,3.]
    assert result['missing_data_mask']['gripper_state']


@pytest.mark.parametrize('change',[{'segment':'disconnect'},{'valid':False}])
def test_no_interpolation_across_invalid_or_disconnected(change):
    a=sample('robot_state',10,payload={'joint_position':[1.]})
    b=sample('robot_state',100,payload={'joint_position':[2.]},**change)
    result=align([a,b,sample('wrist_rgb',70)],[],REF,tolerance_ns=100,interpolate=True)['observations'][0]
    match=result['observation_t']['robot_state']
    assert match is None or match['method']!='linear_joint_position'


def test_no_interpolation_across_reset():
    rows=[sample('robot_state',10,payload={'joint_position':[1.]}),sample('robot_state',5,payload={'joint_position':[2.]}),sample('wrist_rgb',7)]
    assert align(rows,[],REF,tolerance_ns=100,interpolate=True)['observations'][0]['observation_t']['robot_state'] is None


def test_unknown_gripper_and_future_gripper_masked():
    rows=[sample('wrist_rgb',100),sample('gripper_state',95,uncertainty_ns=None,payload={'width':0.01})]
    assert align(rows,[],REF,tolerance_ns=10)['observations'][0]['missing_data_mask']['gripper_state']
    provisional=align(rows,[],REF,tolerance_ns=10,allow_unknown=True)['observations'][0]
    assert provisional['observation_t']['gripper_state']['provisional']
    rows[1]['timestamp_ns']=101
    assert align(rows,[],REF,tolerance_ns=10,allow_unknown=True)['observations'][0]['missing_data_mask']['gripper_state']


def test_uncertainty_budget_and_incompatible_clock():
    rows=[sample('wrist_rgb',100,uncertainty_ns=3),sample('scene_rgb',105,uncertainty_ns=3)]
    assert align(rows,[],REF,tolerance_ns=10)['observations'][0]['missing_data_mask']['scene_rgb']
    rows[1]['clock_id']='scene_hardware_clock'
    result=align(rows,[],REF,tolerance_ns=10)
    assert len(result['rejected_records'])==1


def test_causal_actions_require_observed_output_and_exact_identity():
    command=sample('teleoperation_command',50,command_key=['s',1,0])
    app=sample('command_application',90,command_key=['s',1,0],receipt_monotonic_ns=70)
    anchor=sample('wrist_rgb',100)
    result=align([command,app,anchor],[],REF,tolerance_ns=100)['observations'][0]
    assert result['action_t']['source_timestamp']==50
    assert result['action_t']['applicable_execution_timestamp']==90
    for change in ({'timestamp_ns':101},{'valid':False},{'command_key':['other',1,0]}):
        invalid=dict(app,**change)
        assert align([command,invalid,anchor],[],REF,tolerance_ns=100)['observations'][0]['action_t'] is None
    assert align([command,anchor],[],REF,tolerance_ns=100)['observations'][0]['action_t'] is None


def test_latest_invalid_application_blocks_old_action():
    rows=[sample('teleoperation_command',10,command_key=['s',1,0]),
          sample('command_application',20,command_key=['s',1,0],receipt_monotonic_ns=15),
          sample('command_application',30,command_key=['s',1,0],receipt_monotonic_ns=15,valid=False),sample('wrist_rgb',40)]
    assert align(rows,[],REF,tolerance_ns=100)['observations'][0]['action_t'] is None


def test_raw_image_header_no_pixels_needed():
    encoded=b'\x00\x01\x00\x00'+struct.pack('<iII',123,456,4)+b'cam\x00'
    assert image_header(encoded)==(123000000456,'cam')
    with pytest.raises(ValueError): image_header(b'bad')


def test_gripper_adapter_uncertain_no_motion_invented():
    ctx={'record_id':'1','host_clock_id':REF,'ros_clock_id':'ros','clock_segment':'0'}
    msg={'header':{'stamp':{'sec':1,'nanosec':2}},'calibrated':True,'busy':False,'state':'closed','width':0.01}
    row=adapt('/panda_1/gripper_events',msg,ctx)[0]
    assert row['valid'] and row['uncertainty_ns'] is None
    assert row['payload']==msg and row['timestamp_ns']==1000000002
    assert not adapt('/panda_1/gripper_events',dict(msg,busy=True),ctx)[0]['valid']


def test_joint_states_fallback_is_labelled():
    rows=[sample('joint_states',95,payload={'joint_position':[1.,2.],'original_joint_state':{'position':[1.,2.]}}),sample('wrist_rgb',100)]
    out=align(rows,[],REF,tolerance_ns=10)['observations'][0]['observation_t']['robot_state']
    assert out['source_stream']=='joint_states' and out['payload']['original_joint_state']['position']==[1.,2.]


def test_negative_uncertainty_rejected():
    with pytest.raises(ClockError): map_record(sample('wrist_rgb',1,uncertainty_ns=-1),[],REF)


def test_gap_disables_interpolation():
    rows=[sample('robot_state',0,payload={'joint_position':[0.]}),sample('robot_state',1000,payload={'joint_position':[10.]}),sample('wrist_rgb',500)]
    out=align(rows,[],REF,tolerance_ns=1000,interpolate=True,max_gap_ns=100)['observations'][0]['observation_t']['robot_state']
    assert out['method']=='nearest'


def test_metadata_domains_are_not_mixed():
    from demonstration_timing.diagnostics import analyze
    rows=[{'channel':'wrist/color_metadata','header_ros_ns':1,'arrival_ros_ns':2,'native_frame_ns':3,'clock_domain':'hardware_clock','metadata':{}},
          {'channel':'wrist/depth_metadata','header_ros_ns':1,'arrival_ros_ns':2,'native_frame_ns':3,'clock_domain':'global_time','metadata':{}}]
    assert analyze(rows,{'wrist':30},100)['native_rgb_depth_relationships']['wrist']['sdk_frame_depth_minus_color_ns']['count']==0


def test_tracking_application_masks_homing_and_fallback():
    ctx={'record_id':'1','host_clock_id':REF,'ros_clock_id':'ros','clock_segment':'0'}
    message={'state_time_pc_monotonic_ns':100,'joint_position':[1.], 'command_issue_time_pc_monotonic_ns':110,
             'command_receipt_time_pc_monotonic_ns':80,'motion_enabled':True,'requested_pose_valid':True,
             'cartesian_reference_valid':True,'homing_active':False,'safety_fallback':False}
    assert adapt('/panda_1/tracking_samples',message,ctx)[1]['valid']
    assert not adapt('/panda_1/tracking_samples',dict(message,homing_active=True),ctx)[1]['valid']
    assert not adapt('/panda_1/tracking_samples',dict(message,safety_fallback=True),ctx)[1]['valid']


def test_model_cannot_claim_unknown_or_negative_uncertainty():
    for u in (None,-1):
        with pytest.raises(ClockError): ClockModel('source',REF,'0',0,0,1.,u,0,10,'invalid')


def test_recorded_joint_fallback_outside_tolerance_is_not_reported_unrecorded():
    rows=[sample('joint_states',100,payload={'joint_position':[1.]}),sample('wrist_rgb',1000)]
    out=align(rows,[],REF,tolerance_ns=10)['observations'][0]
    assert out['missing_data_mask']['robot_state']
    assert out['missing_reasons']['robot_state']=='unknown_uncertainty_or_tolerance_or_invalid_segment'
