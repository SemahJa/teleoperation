import csv
import json
import math
from pathlib import Path
import tempfile
import unittest

import rosbag2_py
from rclpy.serialization import serialize_message
from quest_teleop_interfaces.msg import TrackingSample
from recording.export_tracking import export_bag, sample_row


class TrackingExportTest(unittest.TestCase):
    def test_actual_bag_roundtrip_preserves_clocks_arms_and_torque_conventions(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bag = root / "bag"
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""),
            )
            for side in ("left", "right"):
                topic = "/" + side + "/tracking_samples"
                writer.create_topic(
                    rosbag2_py.TopicMetadata(
                        name=topic,
                        type="quest_teleop_interfaces/msg/TrackingSample",
                        serialization_format="cdr",
                    )
                )
                for n in range(3):
                    msg = TrackingSample()
                    msg.header.frame_id = side + "/base"
                    msg.state_time_pc_monotonic_ns = 9007199254740993 + n * 1000000
                    msg.control_cycle = n + 1
                    msg.nominal_torque[0] = 10.0
                    msg.commanded_torque[0] = 9.0
                    msg.robot_desired_torque[0] = 8.0
                    msg.measured_torque[0] = 20.0
                    msg.robot_torque_valid = True
                    msg.requested_pose.position.x = 0.4
                    msg.requested_twist.linear.x = 0.2
                    writer.write(
                        topic, serialize_message(msg), 1000000000 + n * 1000000
                    )
            del writer  # Finalize rosbag metadata before reading it.
            manifest = export_bag(bag, root / "export")
            self.assertEqual(
                set(manifest["topics"]),
                {"/left/tracking_samples", "/right/tracking_samples"},
            )
            for topic, info in manifest["topics"].items():
                with (root / "export" / info["csv"]).open() as file:
                    rows = list(csv.DictReader(file))
                self.assertEqual(len(rows), 3)
                self.assertEqual(
                    rows[0]["state_time_pc_monotonic_ns"], "9007199254740993"
                )
                self.assertAlmostEqual(float(rows[1]["time_s"]), 0.001)
                self.assertEqual(float(rows[0]["nominal_torque_1"]), 10.0)
                self.assertEqual(float(rows[0]["commanded_torque_1"]), 9.0)
                self.assertEqual(float(rows[0]["robot_desired_torque_1"]), 8.0)
                self.assertEqual(float(rows[0]["measured_torque_1"]), 20.0)
                self.assertEqual(rows[0]["robot_torque_valid"], "1")
                self.assertEqual(rows[0]["frame_id"], topic.split("/")[1] + "/base")
            with self.assertRaises(FileExistsError):
                export_bag(bag, root / "export")
            selected = export_bag(bag, root / "one", ["/right/tracking_samples"])
            self.assertEqual(list(selected["topics"]), ["/right/tracking_samples"])
            with self.assertRaises(ValueError):
                export_bag(bag, root / "bad", ["/absent"])
            self.assertIn("includes gravity", json.dumps(manifest))

    def test_relative_time_before_origin_and_invalid_values_are_preserved(self):
        msg = TrackingSample()
        msg.state_time_pc_monotonic_ns = 1000000
        msg.measured_torque[0] = float("nan")
        row = sample_row(msg, 2000000)
        self.assertAlmostEqual(row["time_s"], -0.001)
        self.assertFalse(row["robot_torque_valid"])
        self.assertTrue(math.isnan(row["measured_torque_1"]))


if __name__ == "__main__":
    unittest.main()
