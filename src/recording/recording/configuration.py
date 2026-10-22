"""Recording topic profiles and rosbag command planning."""

from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import uuid
from quest_teleop.configuration import full_topic, read_mapping, boolean, _TOKEN


def default_recording_topics(arms):
    topics = []
    for arm in arms:
        p = arm.parameters
        topics.extend(
            [
                p["controller_state_topic"],
                p["command_topic"],
                p["measured_pose_topic"],
                full_topic("tracking_samples", arm.namespace),
            ]
        )
    return list(dict.fromkeys(topics))


@dataclass
class RecordingPlan:
    topics: list
    output: str
    command: list


def recording_plan(path, arms, topics_override="", output_root_override=""):
    config = read_mapping(path)
    from quest_teleop.runtime_configuration import (
        unified_settings,
        bridge_controller_topic,
    )

    source = config
    if "pipeline" in config:
        recording = unified_settings(config)[2]
        config = {
            k: v
            for k, v in recording.items()
            if k
            not in (
                "mode",
                "namespace",
                "left_controller_state_topic",
                "menu_debounce_seconds",
            )
        }
    config = dict(config)
    config.pop("menu_debounce_seconds", None)
    if set(config) - {"include_default_topics", "topics", "output_root", "storage_id"}:
        raise ValueError("unknown recording configuration key")
    if topics_override:
        topics = json.loads(topics_override)
    else:
        topics = config.get("topics", [])
        if not isinstance(topics, list):
            raise ValueError("recording topics must be a list")
        if boolean(
            config.get("include_default_topics", True), "include_default_topics"
        ):
            topics = (
                [bridge_controller_topic(source, "left")]
                + default_recording_topics(arms)
                + topics
            )
    if not isinstance(topics, list) or not topics:
        raise ValueError(
            "enabled recording needs at least one explicitly selected topic"
        )
    topics = list(dict.fromkeys(full_topic(topic) for topic in topics))
    root = output_root_override or config.get("output_root", "")
    if (
        not isinstance(root, str)
        or not root
        or not Path(root).expanduser().is_absolute()
    ):
        raise ValueError("recording output_root must be an absolute container path")
    storage = config.get("storage_id", "sqlite3")
    if not isinstance(storage, str) or not _TOKEN.fullmatch(storage):
        raise ValueError("invalid rosbag storage plugin name")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = str(Path(root).expanduser() / f"quest_{stamp}_{uuid.uuid4().hex[:8]}")
    # Arguments are passed directly; no shell or topic glob expansion.
    return RecordingPlan(
        topics,
        output,
        ["ros2", "bag", "record", "--storage", storage, "--output", output, *topics],
    )
