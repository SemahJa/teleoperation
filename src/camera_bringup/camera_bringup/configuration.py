"""Validate all selected camera settings before starting either process."""
import re
import yaml


def load_camera(path, role, profile):
    with open(path) as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or set(data) != {"serial", "profiles"}:
        raise ValueError(f"{role}: expected serial and profiles")
    serial = data["serial"]
    if not isinstance(serial, str) or not re.fullmatch(r"[0-9]+", serial):
        raise ValueError(f"{role}: serial must be a quoted numeric string")
    profiles = data["profiles"]
    if not isinstance(profiles, dict) or profile not in profiles:
        raise ValueError(f"{role}: unknown profile {profile}")
    settings = profiles[profile]
    required = {"color_profile", "depth_profile", "enable_color", "enable_depth", "align_depth"}
    if not isinstance(settings, dict) or set(settings) != required:
        raise ValueError(f"{role}: invalid stream settings")
    for name in ("color_profile", "depth_profile"):
        if not isinstance(settings[name], str) or not re.fullmatch(r"[1-9][0-9]*,[1-9][0-9]*,[1-9][0-9]*", settings[name]):
            raise ValueError(f"{role}: {name} must be width,height,fps")
    for name in ("enable_color", "enable_depth", "align_depth"):
        if type(settings[name]) is not bool:
            raise ValueError(f"{role}: {name} must be boolean")
    if settings["align_depth"] and not (settings["enable_color"] and settings["enable_depth"]):
        raise ValueError(f"{role}: aligned depth requires both streams")
    if settings["enable_color"] and settings["enable_depth"]:
        if settings["color_profile"].split(",")[-1] != settings["depth_profile"].split(",")[-1]:
            raise ValueError(f"{role}: synchronized streams require matching FPS")
    return {
        "serial_no": "_" + serial,
        "camera_name": role,
        "rgb_camera.color_profile": settings["color_profile"],
        "depth_module.depth_profile": settings["depth_profile"],
        "rgb_camera.color_format": "RGB8", "depth_module.depth_format": "Z16",
        "enable_color": settings["enable_color"], "enable_depth": settings["enable_depth"],
        "align_depth.enable": settings["align_depth"],
        "enable_sync": True, "publish_tf": True,
        "enable_infra": False, "enable_infra1": False, "enable_infra2": False,
        "enable_gyro": False, "enable_accel": False, "enable_motion": False,
        "pointcloud.enable": False, "enable_rgbd": False,
        "initial_reset": False, "wait_for_device_timeout": -1.0,
        "reconnect_timeout": 6.0,
    }
