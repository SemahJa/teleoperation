"""Recording topic profiles and rosbag command planning."""

from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
import uuid
from recording.profiles import demonstration_topics
from recording.storage_options import mcap_storage_options
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
    options: dict = field(default_factory=dict)


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
    if set(config) - {"include_default_topics", "topics", "output_root", "storage_id", "profile", "record_late_topics", "max_cache_size", "clock_provenance", "mcap_compression", "mcap_compression_level"}:
        raise ValueError("unknown recording configuration key")
    profile = config.get("profile", "teleop")
    if profile not in ("teleop", "demonstration"):
        raise ValueError("recording profile must be teleop/demonstration")
    late = config.get("record_late_topics", profile == "demonstration")
    late = boolean(late, "record_late_topics")
    cache = config.get("max_cache_size", 104857600)
    if type(cache) is not int or cache < 0:
        raise ValueError("max_cache_size must be a nonnegative integer")
    provenance = config.get("clock_provenance", "unverified")
    if provenance not in ("unverified", "same_host"):
        raise ValueError("clock_provenance must be unverified/same_host")
    compression = config.get("mcap_compression", "None")
    compression_level = config.get("mcap_compression_level", "Fast")
    mcap_storage_options(compression, compression_level)
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
                + (demonstration_topics(arms)
                   if profile == "demonstration" else [])
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
        dict(profile=profile,record_late_topics=late,max_cache_size=cache,clock_provenance=provenance,
             mcap_compression=compression,mcap_compression_level=compression_level),
    )
