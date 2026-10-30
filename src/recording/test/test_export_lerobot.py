import pytest
from demonstration_timing.clocks import ClockModel
from recording.export_lerobot import camera_topics,convert_ros,features


def test_historical_names_are_corrected_by_serial():
    manifest={'camera_device_info':{
        'wrist':{'serial_number':'934222072657'},
        'scene':{'serial_number':'238222073675'}}}
    topics=camera_topics(manifest)
    assert topics['scene_rgb']=='/cameras/wrist/color/image_raw'
    assert topics['wrist_depth']=='/cameras/scene/aligned_depth_to_color/image_raw'


def test_camera_roles_must_be_unambiguous():
    with pytest.raises(ValueError):camera_topics({'camera_device_info':{}})
    with pytest.raises(ValueError):camera_topics({'camera_device_info':{
        'a':{'serial_number':'934222072657'},'b':{'serial_number':'934222072657'}}})


def test_clock_maps_are_bounded_and_ambiguous_epochs_rejected():
    model=ClockModel('ros','mono','0',100,1000,1.,10,100,200,'test')
    assert convert_ros(120,[model])==1020
    assert convert_ros(99,[model]) is None
    assert convert_ros(201,[model]) is None
    assert convert_ros(120,[model,model]) is None


def test_action_and_image_feature_contract():
    specs=features()
    assert specs['action']['shape']==(8,)
    assert specs['action_valid']['shape']==(8,)
    assert specs['source.time_ns']['dtype']=='int64'
    assert specs['observation.images.scene']['shape']==(3,480,640)
    assert specs['observation.gripper_width_valid']['shape']==(1,)


def test_exact_selection_uses_recorded_start_and_excludes_other_bags(tmp_path):
    from recording.export_lerobot import select_bags
    import yaml
    for name,start in [('old',1),('selected_a',300),('selected_b',200),('reserved',400)]:
        d=tmp_path/name;d.mkdir()
        (d/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information':{'starting_time':{'nanoseconds_since_epoch':start}}}))
    assert [p.name for p in select_bags(tmp_path,['selected_a','selected_b'])]==['selected_b','selected_a']
    with pytest.raises(ValueError):select_bags(tmp_path,['missing'])
    with pytest.raises(ValueError):select_bags(tmp_path,['selected_a','selected_a'])
    with pytest.raises(ValueError):select_bags(tmp_path,['selected_a'],1)


def test_output_outcomes_must_match_source_labels():
    from recording.export_lerobot import validate_source_labels
    plans=[dict(source_id='failure-id',outcome='failure',success=False,summary={'retained_frames':5}),dict(source_id='idle-id',outcome='unknown',success=None,summary={'retained_frames':0})]
    segment=dict(source_episode_id='failure-id',outcome='failure',success=False)
    assert validate_source_labels([segment],plans)=={'failure':1}
    with pytest.raises(ValueError):validate_source_labels([dict(segment,outcome='direct_success',success=True)],plans)
