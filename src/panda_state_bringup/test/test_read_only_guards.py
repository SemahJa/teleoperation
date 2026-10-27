"""Startup refusal tests: no robot address capable of connecting is supplied."""
import os
import subprocess
import uuid
from pathlib import Path
import rclpy
from sensor_msgs.msg import JointState


def binary():
    return os.environ['READ_ONLY_TEST_BINARY']


def invoke(*args):
    return subprocess.run([binary(),'--ros-args',*args],capture_output=True,text=True,timeout=7)


def test_invalid_parameters_refuse_before_robot_connection():
    p=invoke('-p','robot_ip:=unresolvable-read-only-test.invalid','-p','duration_seconds:=-1.0')
    assert p.returncode==1 and 'duration_seconds' in p.stderr


def test_sim_clock_refuses_before_robot_connection():
    p=invoke('-p','robot_ip:=unresolvable-read-only-test.invalid','-p','use_sim_time:=true')
    assert p.returncode==1 and 'simulation clock rejected' in p.stderr


def test_duplicate_measurement_source_refuses_before_robot_connection():
    rclpy.init()
    prefix='/readonly_guard_'+uuid.uuid4().hex
    node=rclpy.create_node('measurement_source',namespace=prefix)
    pub=node.create_publisher(JointState,'joint_states',10)
    try:
        p=invoke('-r','__ns:='+prefix,'-p','robot_ip:=unresolvable-read-only-test.invalid')
        assert p.returncode==1 and 'measurement topic already has a publisher' in p.stderr
    finally:
        node.destroy_node();rclpy.shutdown()
