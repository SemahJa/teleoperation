#!/usr/bin/env python3
"""Export coherent TrackingSample topics from rosbag2 to portable, lossless CSV."""
import argparse
import csv
import json
from pathlib import Path
import re

import yaml


SCALARS = (
    "control_cycle",
    "state_time_pc_monotonic_ns",
    "control_time_pc_monotonic_ns",
    "command_issue_time_pc_monotonic_ns",
    "command_source_time_pc_monotonic_ns",
    "command_receipt_time_pc_monotonic_ns",
    "openxr_location_time_ns",
    "sequence",
    "reference_space_generation",
    "session_id",
    "reason",
    "hardware_backend",
    "motion_enabled",
    "requested_pose_valid",
    "requested_twist_valid",
    "cartesian_reference_valid",
    "homing_active",
    "period_seconds",
    "command_age_ms",
    "lambda_translation",
    "lambda_rotation",
    "gain_translation",
    "gain_rotation",
    "robot_torque_valid",
    "gravity_valid",
    "previous_command_valid",
    "previous_control_cycle",
    "previous_command_issue_time_pc_monotonic_ns",
    "maximum_torque_rate",
    "torque_rate_limited",
    "torque_magnitude_limited",
    "cbf_enabled",
    "safety_fallback",
    "solver_succeeded",
    "normalized_torque_intervention",
    "monitor_queue_drops",
    "position_error_norm",
    "rotation_error_angle",
    "linear_velocity_error_norm",
    "angular_velocity_error_norm",
)
ARRAYS = {
    "position_error": ("x", "y", "z"),
    "rotation_error": ("x", "y", "z"),
    **{
        name: tuple(str(i) for i in range(1, 8))
        for name in (
            "joint_position",
            "joint_velocity",
            "homing_joint_position",
            "homing_joint_velocity",
            "nominal_torque",
            "commanded_torque",
            "robot_desired_torque",
            "measured_torque",
            "gravity_torque",
            "previous_nominal_torque",
            "previous_commanded_torque",
            "torque_limits",
        )
    },
}


def sample_row(msg, origin_ns):
    """Subtract integer clocks BEFORE conversion to seconds; retain absolute uint64s."""
    row = {
        "time_s": (int(msg.state_time_pc_monotonic_ns) - origin_ns) * 1e-9,
        "frame_id": msg.header.frame_id,
    }
    for name in SCALARS:
        value = getattr(msg, name)
        row[name] = int(value) if isinstance(value, bool) else value
    for prefix in ("requested", "reference", "measured"):
        pose = getattr(msg, prefix + "_pose")
        twist = getattr(msg, prefix + "_twist")
        for part, fields in (("position", "xyz"), ("orientation", "xyzw")):
            for axis in fields:
                row[f"{prefix}_{part}_{axis}"] = getattr(getattr(pose, part), axis)
        for part in ("linear", "angular"):
            for axis in "xyz":
                row[f"{prefix}_{part}_velocity_{axis}"] = getattr(
                    getattr(twist, part), axis
                )
    for name, suffixes in ARRAYS.items():
        for suffix, value in zip(suffixes, getattr(msg, name)):
            row[f"{name}_{suffix}"] = float(value)
    return row


def export_bag(bag, output, topics=()):
    # ROS imports are isolated so CSV consumers do not require ROS or its tools.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from quest_teleop_interfaces.msg import TrackingSample

    bag = Path(bag).resolve()
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text())
    storage = metadata["rosbag2_bagfile_information"]["storage_identifier"]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage),
        rosbag2_py.ConverterOptions("", ""),
    )
    selected = [
        item.name
        for item in reader.get_all_topics_and_types()
        if item.type == "quest_teleop_interfaces/msg/TrackingSample"
        and (item.name in topics if topics else item.name.endswith("/tracking_samples"))
    ]
    if not selected or set(topics) - set(selected):
        raise ValueError(
            "Requested TrackingSample topics not found; new recordings need tracking_samples"
        )
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    names = {
        topic: re.sub(r"[^A-Za-z0-9_.-]", "_", topic.strip("/")) + ".csv"
        for topic in selected
    }
    if len(set(names.values())) != len(names):
        raise ValueError("Topic names collide as CSV filenames; export them separately")
    paths = [output / name for name in names.values()] + [output / "manifest.json"]
    if any(path.exists() for path in paths):
        raise FileExistsError(
            "Use a new output directory; existing exports are never overwritten"
        )
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    files = {}
    writers = {}
    counts = dict.fromkeys(selected, 0)
    origins = {}
    drops = dict.fromkeys(selected, 0)
    try:
        while reader.has_next():
            topic, serialized, _ = reader.read_next()
            msg = deserialize_message(serialized, TrackingSample)
            origin = origins.setdefault(topic, int(msg.state_time_pc_monotonic_ns))
            row = sample_row(msg, origin)
            if topic not in writers:
                files[topic] = (output / names[topic]).open("x", newline="")
                writers[topic] = csv.DictWriter(files[topic], fieldnames=list(row))
                writers[topic].writeheader()
            writers[topic].writerow(row)
            counts[topic] += 1
            drops[topic] = max(drops[topic], int(msg.monitor_queue_drops))
    finally:
        for file in files.values():
            file.close()
    manifest = {
        "schema_version": 1,
        "bag": str(bag),
        "storage_id": storage,
        "clock": "PC CLOCK_MONOTONIC; time_s is relative state time",
        "torque_conventions": {
            "nominal_and_commanded": "current callback output, without gravity/friction compensation",
            "robot_desired": "incoming RobotState tau_J_d, without gravity",
            "measured": "incoming link-side torque sensor tau_J, includes gravity",
            "previous_command": "previous callback output; no exact robot acknowledgement implied",
        },
        "topics": {
            topic: {
                "csv": names[topic],
                "samples": counts[topic],
                "origin_ns": str(origins[topic]) if topic in origins else None,
                "monitor_queue_drops": drops[topic],
            }
            for topic in selected
        },
    }
    with (output / "manifest.json").open("x") as file:
        json.dump(manifest, file, indent=2)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "bag", help="Finalized rosbag2 directory containing metadata.yaml"
    )
    parser.add_argument("output", help="New export directory")
    parser.add_argument(
        "--topic",
        action="append",
        default=[],
        help="Exact topic; repeat for multiple arms",
    )
    args = parser.parse_args()
    manifest = export_bag(args.bag, args.output, args.topic)
    for topic, info in manifest["topics"].items():
        print(f"{topic}: {info['samples']} samples -> {info['csv']}")


if __name__ == "__main__":
    main()
