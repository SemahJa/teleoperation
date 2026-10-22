"""Validated per-arm mapper configuration."""

from dataclasses import dataclass
import math
from pathlib import Path
import re

import yaml


_TOKEN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def full_topic(value, namespace="/"):
    if not isinstance(value, str) or not value or value.endswith("/"):
        raise ValueError(f"invalid ROS topic: {value!r}")
    resolved = value if value.startswith("/") else namespace.rstrip("/") + "/" + value
    if not all(_TOKEN.fullmatch(part) for part in resolved[1:].split("/")):
        raise ValueError(f"invalid ROS topic: {resolved!r}")
    return resolved


def read_mapping(path):
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a YAML mapping")
    return data


def number(value, field, minimum=0.0, inclusive=False):
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ValueError(f"{field} must be finite numeric data")
    if value < minimum or (not inclusive and value == minimum):
        raise ValueError(f'{field} must be {">=" if inclusive else ">"} {minimum}')
    return float(value)


def boolean(value, field):
    if not isinstance(value, bool):
        raise ValueError(f"{field} must be true or false")
    return value


def launch_bool(value):
    if value.lower() not in ("true", "false"):
        raise ValueError(f"expected true/false launch argument, got {value!r}")
    return value.lower() == "true"


@dataclass
class ArmConfig:
    name: str
    namespace: str
    parameters: dict
    robot_ip: str
    state_topic: str


_DEFAULTS = dict(
    translation_scale=1.0,
    rotation_scale=1.0,
    feedforward_scale=1.0,
    twist_filter_cutoff_hz=10.0,
    pose_filter="none",
    velocity_filter="low_pass",
    pose_interpolation="none",
    velocity_interpolation="none",
    pose_filter_cutoff_hz=10.0,
    velocity_filter_cutoff_hz=10.0,
    pose_interpolation_duration_ms=40.0,
    velocity_interpolation_duration_ms=40.0,
    command_timeout_ms=100,
    max_clock_uncertainty_ms=10.0,
    max_source_future_ms=10.0,
    min_derivative_dt_ms=1.0,
    max_derivative_dt_ms=100.0,
    trigger_engage=0.75,
    trigger_release=0.25,
)


def load_arms(path):
    data = read_mapping(path)
    from quest_teleop.runtime_configuration import (
        unified_settings,
        bridge_controller_topic,
    )

    source = data
    if "pipeline" in data:
        unified_settings(data)
        data = {"arms": data["arms"]}
    if set(data) != {"arms"} or not isinstance(data["arms"], dict) or not data["arms"]:
        raise ValueError("arms configuration must contain one nonempty arms mapping")
    result = []
    namespaces, frames, outputs, inputs, ips = set(), set(), set(), set(), set()
    allowed = set(_DEFAULTS) | {
        "namespace",
        "base_frame",
        "controller",
        "robot_ip",
        "controller_enabled",
        "operator_confirm",
        "command_topic",
        "target_pose_topic",
        "measured_pose_topic",
        "controller_state_topic",
        "state_topic",
        "reference_to_robot_xyzw",
    }
    for name, raw in data["arms"].items():
        if (
            not isinstance(name, str)
            or not _TOKEN.fullmatch(name)
            or not isinstance(raw, dict)
        ):
            raise ValueError("each arm needs a valid name and parameter mapping")
        unknown = set(raw) - allowed
        if unknown:
            raise ValueError(f"{name}: unknown configuration keys: {sorted(unknown)}")
        namespace = raw.get("namespace", "/" + name)
        namespace = full_topic(namespace)
        if namespace in namespaces:
            raise ValueError("arm namespaces must be unique")
        namespaces.add(namespace)
        frame = raw.get("base_frame", namespace.lstrip("/") + "/panda_link0")
        if (
            not isinstance(frame, str)
            or not frame
            or frame.startswith("/")
            or frame in frames
        ):
            raise ValueError(
                "base frames must be distinct nonempty frame IDs without leading slash"
            )
        frames.add(frame)
        controller = raw.get("controller", "right")
        if controller not in ("left", "right"):
            raise ValueError(f"{name}: controller must be left or right")
        for key in ("controller_enabled", "operator_confirm"):
            if boolean(raw.get(key, False), key):
                raise ValueError(
                    f"{name}: robot-free launch does not activate hardware; {key} must be false"
                )
        ip = raw.get("robot_ip", "")
        if not isinstance(ip, str) or (ip and ip in ips):
            raise ValueError("nonempty robot_ip values must be unique strings")
        if ip:
            ips.add(ip)
        p = dict(_DEFAULTS)
        p.update({key: raw[key] for key in _DEFAULTS if key in raw})
        if "velocity_filter_cutoff_hz" not in raw:
            p["velocity_filter_cutoff_hz"] = p["twist_filter_cutoff_hz"]
        modes = {"pose_filter", "velocity_filter"}
        interpolations = {"pose_interpolation", "velocity_interpolation"}
        for key in modes:
            if p[key] not in ("none", "low_pass", "four_pole"):
                raise ValueError(f"{name}.{key}: choose none, low_pass or four_pole")
        for key in interpolations:
            if p[key] not in ("none", "ptp_polynomial", "spline"):
                raise ValueError(f"{name}.{key}: choose none, ptp_polynomial or spline")
        for key in set(_DEFAULTS) - modes - interpolations:
            p[key] = number(
                p[key],
                f"{name}.{key}",
                inclusive=key
                in ("feedforward_scale", "max_source_future_ms", "trigger_release"),
            )
        if p["feedforward_scale"] > 1.0 or not (
            p["trigger_release"] < p["trigger_engage"] <= 1.0
        ):
            raise ValueError(f"{name}: invalid feedforward scale or trigger hysteresis")
        if p["min_derivative_dt_ms"] > p["max_derivative_dt_ms"]:
            raise ValueError(f"{name}: derivative interval limits reversed")
        if not p["command_timeout_ms"].is_integer():
            raise ValueError(
                "command_timeout_ms must be an integer number of milliseconds"
            )
        p["command_timeout_ms"] = int(p["command_timeout_ms"])
        basis = raw.get("reference_to_robot_xyzw", [0.0, 0.0, 0.0, 1.0])
        if (
            not isinstance(basis, list)
            or len(basis) != 4
            or any(
                isinstance(x, bool)
                or not isinstance(x, (float, int))
                or not math.isfinite(x)
                for x in basis
            )
        ):
            raise ValueError(
                f"{name}: reference_to_robot_xyzw needs four finite numbers"
            )
        norm = math.sqrt(sum(x * x for x in basis))
        if not math.isfinite(norm) or norm < 1e-9:
            raise ValueError(f"{name}: invalid calibration quaternion")
        p.update(
            controller_id=controller,
            base_frame=frame,
            reference_to_robot_xyzw=[float(x / norm) for x in basis],
        )
        for key, default in [
            ("measured_pose_topic", "teleop_anchor_pose"),
            ("command_topic", "cartesian_teleop_command"),
            ("monitor_topic", "target_pose"),
        ]:
            config_key = "target_pose_topic" if key == "monitor_topic" else key
            topic = full_topic(raw.get(config_key, default), namespace)
            if key == "measured_pose_topic":
                if topic in inputs:
                    raise ValueError("measured pose topics must be unique per arm")
                inputs.add(topic)
            else:
                if topic in outputs:
                    raise ValueError(
                        "command and monitor topics must be unique per arm"
                    )
                outputs.add(topic)
            p[key] = topic
        p["controller_state_topic"] = full_topic(
            raw.get(
                "controller_state_topic", bridge_controller_topic(source, controller)
            ),
            namespace,
        )
        result.append(
            ArmConfig(
                name,
                namespace,
                p,
                ip,
                full_topic(raw.get("state_topic", "robot_state"), namespace),
            )
        )
    if outputs & inputs:
        raise ValueError("mapper outputs must not alias measured-pose inputs")
    if any(
        arm.parameters["controller_state_topic"] in outputs | inputs for arm in result
    ):
        raise ValueError(
            "raw controller input must not alias mapper output or measured pose"
        )
    return result


def select_arms(arms, mode="single", single_controller="right"):
    """Select active owners; a one-arm configuration can use either hand."""
    from dataclasses import replace

    if mode not in ("single", "dual") or single_controller not in ("left", "right"):
        raise ValueError(
            "mode must be single/dual; single_controller must be left/right"
        )
    if mode == "dual":
        if len(arms) != 2 or {a.parameters["controller_id"] for a in arms} != {
            "left",
            "right",
        }:
            raise ValueError(
                "dual mode needs exactly two arms assigned to left and right"
            )
        return sorted(arms, key=lambda a: a.parameters["controller_id"])
    matches = [a for a in arms if a.parameters["controller_id"] == single_controller]
    if len(arms) == 1:
        arm = arms[0]
    elif len(matches) == 1:
        arm = matches[0]
    else:
        raise ValueError(
            "single mode needs one unambiguous configured arm for the selected controller"
        )
    params = dict(arm.parameters)
    old = params["controller_id"]
    params["controller_id"] = single_controller
    if params["controller_state_topic"].endswith(
        f"/quest_usb_bridge/{old}/controller_state"
    ):
        params["controller_state_topic"] = (
            params["controller_state_topic"].rsplit(f"/{old}/", 1)[0]
            + f"/{single_controller}/controller_state"
        )
    elif old != single_controller:
        raise ValueError(
            "set the custom controller_state_topic for the selected single controller in arms_config"
        )
    return [replace(arm, parameters=params)]
