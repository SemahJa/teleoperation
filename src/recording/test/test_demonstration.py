from pathlib import Path
import hashlib
import tempfile
import yaml
import pytest
from sensor_msgs.msg import Image
from rclpy.serialization import serialize_message
from recording.image_tools import image_view
from recording.profiles import qos_overrides,selected_topics
from recording.configuration import recording_plan
from recording.validate_episode import loss_comparison
from quest_teleop.configuration import load_arms


def test_image_pixel_view_preserves_payload():
    m=Image();m.header.frame_id='optical';m.header.stamp.sec=12;m.height=2;m.width=2;m.encoding='16UC1';m.step=4;m.data=bytes([1,0,2,0,3,0,4,0])
    h,pixels=image_view(serialize_message(m))
    assert h['stamp_ns']==12000000000 and h['width']==2 and h['height']==2
    assert h['encoding']=='16UC1' and bytes(pixels)==bytes(m.data)
    assert hashlib.sha256(pixels).hexdigest()==hashlib.sha256(bytes(m.data)).hexdigest()


def test_bad_image_length_is_rejected():
    m=Image();m.height=2;m.width=2;m.encoding='rgb8';m.step=6;m.data=b'bad'
    with pytest.raises(ValueError):image_view(serialize_message(m))


def test_late_topics_and_inactive_arm_exclusion():
    topics=['/cameras/wrist/color/image_raw','/cameras/scene/color/image_raw','/inactive/state']
    assert selected_topics(topics,['/inactive'],set(),True)==topics[:2]
    assert selected_topics(topics,['/inactive'],{topics[0]},False)==topics[:1]


def test_sensor_qos_and_latched_tf():
    qos=qos_overrides(['/cameras/wrist/color/image_raw','/tf_static','/panda_1/gripper_events','/panda_1/tracking_samples'])
    assert qos['/cameras/wrist/color/image_raw']['reliability']=='best_effort'
    assert qos['/tf_static']['durability']=='transient_local'
    assert qos['/panda_1/gripper_events']['durability']=='transient_local'
    assert qos['/panda_1/tracking_samples']['reliability']=='best_effort'


def test_comparison_excludes_start_stop_boundary_frames():
    result=loss_comparison({'1':1,'2':1,'3':1,'4':1},{2:1,4:1,5:1})
    assert result['observer_only']==1 and result['bag_only']==0


def test_demonstration_plan_and_sqlite_compatibility(tmp_path):
    arms_path=tmp_path/'arms.yaml';arms_path.write_text(yaml.safe_dump({'arms':{'panda_1':{'namespace':'/panda_1','controller':'right','robot_ip':''}}}))
    arms=load_arms(arms_path)
    config=tmp_path/'record.yaml';config.write_text(yaml.safe_dump({'profile':'demonstration','output_root':str(tmp_path),'storage_id':'mcap'}))
    plan=recording_plan(config,arms)
    assert '/cameras/scene/depth/metadata' in plan.topics and '/panda_1/gripper_events' in plan.topics
    assert '/tf_static' in plan.topics and plan.command[4]=='mcap' and plan.options['record_late_topics']
    data=yaml.safe_load(config.read_text());data['storage_id']='sqlite3';config.write_text(yaml.safe_dump(data))
    assert recording_plan(config,arms).command[4]=='sqlite3'


@pytest.mark.parametrize('change',[{'profile':'typo'},{'record_late_topics':'true'},{'max_cache_size':-1},{'clock_provenance':'guessed'}])
def test_invalid_recording_options_rejected(tmp_path,change):
    p=tmp_path/'record.yaml';p.write_text(yaml.safe_dump(dict(output_root=str(tmp_path),**change)))
    with pytest.raises(ValueError):recording_plan(p,[])


@pytest.mark.parametrize("algorithm", ["None", "Zstd"])
def test_storage_options_preserve_integrity_settings(algorithm):
    from recording.storage_options import mcap_storage_options
    options = mcap_storage_options(algorithm, "Fast")
    assert options["compression"] == algorithm
    assert options["noChunkCRC"] is False
    assert options["noMessageIndex"] is False


@pytest.mark.parametrize("algorithm,level", [("jpeg", "Fast"), ("Zstd", "typo"), (None, "Fast")])
def test_invalid_storage_options_rejected(algorithm, level):
    from recording.storage_options import mcap_storage_options
    with pytest.raises(ValueError):
        mcap_storage_options(algorithm, level)


def test_compressed_mcap_roundtrip_preserves_cdr_and_timestamps(tmp_path):
    import rosbag2_py
    from recording.storage_options import mcap_storage_options
    if "mcap" not in rosbag2_py.get_registered_writers():
        pytest.skip("MCAP plugin is not installed")
    options_path = tmp_path / "storage.yaml"
    options_path.write_text(yaml.safe_dump(mcap_storage_options("Zstd", "Fast")))
    bag = tmp_path / "compressed"
    options = rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap")
    options.storage_config_uri = str(options_path)
    writer = rosbag2_py.SequentialWriter()
    writer.open(options, rosbag2_py.ConverterOptions("", ""))
    writer.create_topic(rosbag2_py.TopicMetadata(name="/rgb", type="sensor_msgs/msg/Image", serialization_format="cdr"))
    expected = []
    for i in range(4):
        m = Image()
        m.header.stamp.sec = 123
        m.header.stamp.nanosec = i * 33333333
        m.header.frame_id = "camera_color_optical_frame"
        m.width, m.height, m.encoding, m.step = 320, 240, "rgb8", 960
        m.data = bytes(range(256)) * 900
        raw = serialize_message(m)
        stamp = 123000000000 + i * 33333333 + 1000000
        writer.write("/rgb", raw, stamp)
        expected.append(("/rgb", raw, stamp))
    del writer
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    actual = []
    while reader.has_next():
        actual.append(reader.read_next())
    assert actual == expected
    assert sum(p.stat().st_size for p in bag.glob("*.mcap")) < sum(len(r[1]) for r in expected) / 4


def test_compression_plan_propagates_options(tmp_path):
    p = tmp_path / "record.yaml"
    p.write_text(yaml.safe_dump(dict(profile="demonstration", output_root=str(tmp_path),
        storage_id="mcap", mcap_compression="Zstd", mcap_compression_level="Fast")))
    plan = recording_plan(p, [])
    assert plan.options["mcap_compression"] == "Zstd"
    assert plan.options["mcap_compression_level"] == "Fast"
