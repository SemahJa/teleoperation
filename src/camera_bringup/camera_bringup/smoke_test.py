#!/usr/bin/env python3
"""Bounded camera-only RGB-D check. No robot or gripper publishers."""
import argparse
import json
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--roles", nargs="+", choices=["wrist", "scene"], default=["wrist", "scene"])
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--expected-fps", type=float, default=30.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("camera_smoke_test")
    data, frames, subscriptions = {}, set(), []
    streams = {"rgb": ("color/image_raw", Image), "depth": ("aligned_depth_to_color/image_raw", Image),
               "rgb_info": ("color/camera_info", CameraInfo), "depth_info": ("aligned_depth_to_color/camera_info", CameraInfo)}
    for role in args.roles:
        for kind, (suffix, msg_type) in streams.items():
            key = role + "/" + kind
            data[key] = []
            def collect(msg, key=key):
                data[key].append((time.monotonic(), msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec,
                                  msg.width, msg.height, msg.header.frame_id, getattr(msg, "encoding", "")))
            subscriptions.append(node.create_subscription(msg_type, "/cameras/" + role + "/" + suffix, collect, qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(TFMessage, "/tf_static",
        lambda msg: frames.update(t.child_frame_id for t in msg.transforms),
        QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    results, failures = {}, []
    for key, rows in data.items():
        rate = (len(rows)-1)/(rows[-1][0]-rows[0][0]) if len(rows)>1 else 0.0
        results[key] = {"count": len(rows), "arrival_hz": round(rate, 2),
                        "coverage_seconds": round(rows[-1][0]-rows[0][0], 3) if len(rows)>1 else 0.0,
                        "sizes": sorted({(x[2], x[3]) for x in rows}),
                        "frames": sorted({x[4] for x in rows}), "encodings": sorted({x[5] for x in rows})}
        if len(rows)<2 or any(x[2:4] != (640,480) or x[1]<=0 for x in rows):
            failures.append(key + ": missing data, invalid stamp or unexpected resolution")
        if key.endswith(("/rgb", "/depth")) and (rate < args.expected_fps * 0.7 or len(rows) < args.seconds * args.expected_fps * 0.7):
            failures.append(key + ": rate or full-window frame count below 70% of requested FPS")
    for role in args.roles:
        rgb = data[role + "/rgb"]
        depth = data[role + "/depth"]
        matching = len({x[1] for x in rgb} & {x[1] for x in depth})
        results[role + "/matching_rgb_depth_stamps"] = matching
        if not matching:
            failures.append(role + ": no matching RGB/depth stamps")
        if any(x[4] != role + "_color_optical_frame" for x in rgb + depth):
            failures.append(role + ": unexpected RGB/aligned depth optical frame")
        for frame in (role + "_color_optical_frame", role + "_depth_optical_frame"):
            if frame not in frames:
                failures.append("missing static optical TF: " + frame)
    print(json.dumps({"seconds": args.seconds, "results": results, "static_frames": sorted(frames), "failures": failures}, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(bool(failures))


if __name__ == "__main__":
    main()
