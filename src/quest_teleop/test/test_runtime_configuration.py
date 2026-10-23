"""Unified configuration routing, strict validation and standalone compatibility."""

import copy
from pathlib import Path
import tempfile
import unittest
import yaml
from quest_teleop.configuration import load_arms, select_arms
from quest_teleop.runtime_configuration import (
    ALLOWED,
    BRIDGE_DEFAULTS,
    PIPELINE_DEFAULTS,
    RECORDING_DEFAULTS,
    load_runtime,
    unified_settings,
)


class RuntimeConfigurationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "runtime.yaml"
        self.data = {
            "pipeline": {},
            "arms": {"arm": {"namespace": "/arm", "controller": "right"}},
        }

    def write(self, data):
        self.path.write_text(yaml.safe_dump(data))
        return self.path

    def test_package_defaults_and_unified_sections(self):
        arms = load_arms(self.write(self.data))
        settings = load_runtime(self.path, arms)
        self.assertEqual(settings, {})
        self.assertEqual(
            arms[0].parameters["controller_state_topic"],
            "/quest_usb_bridge/right/controller_state",
        )
        standalone = {"controller": {"gain_translation": 7.0}, "arm_settings": {}}
        self.assertEqual(load_runtime(self.write(standalone), arms), standalone)
        self.assertEqual(len(load_arms(self.write({"arms": self.data["arms"]}))), 1)
        pipeline, _, _ = unified_settings(self.data)
        self.assertEqual(pipeline["backend"], "mock")

    def test_single_template_exposes_only_mode_relevant_settings(self):
        path = Path(__file__).parents[2] / "teleop_bringup/config/single_panda.yaml"
        data = yaml.safe_load(path.read_text())
        arms = load_arms(path)
        load_runtime(path, arms)
        pipeline, bridge, recording = unified_settings(data)
        self.assertEqual(set(data["pipeline"]), set(PIPELINE_DEFAULTS))
        self.assertEqual(set(data["quest"]["bridge"]), set(BRIDGE_DEFAULTS))
        self.assertEqual(set(data["recording"]), set(RECORDING_DEFAULTS))
        for section in ALLOWED:
            expected = ALLOWED[section] - (
                {"inter_arm_minimum_clearance"} if section == "cbf" else set()
            )
            if section == "hand":
                expected -= {
                    "grasp_force_n",
                    "grasp_width_m",
                    "epsilon_inner_m",
                    "epsilon_outer_m",
                    "grip_click_max_ms",
                    "grip_double_click_ms",
                }
            self.assertEqual(set(data[section]), expected)
        # The template is edited for deployment; schema/selection checks must
        # not require an operator's saved backend to remain the mock default.
        self.assertEqual(pipeline["mode"], "single")
        self.assertTrue(pipeline["start_bridge"])
        self.assertEqual(bridge["listen_port"], 45454)
        self.assertEqual(recording["mode"], "menu")
        self.assertNotIn("arm_settings", data)
        self.assertNotIn("controller", data["arms"]["panda_1"])
        self.assertTrue(
            all(
                "controller_enabled" not in a and "operator_confirm" not in a
                for a in data["arms"].values()
            )
        )
        left = select_arms(arms, "single", "left")[0]
        self.assertEqual(left.parameters["controller_id"], "left")

    def test_dual_template_requires_no_physical_geometry_for_mock(self):
        path = Path(__file__).parents[2] / "teleop_bringup/config/dual_panda.yaml"
        data = yaml.safe_load(path.read_text())
        # Installation YAML is operator-editable. Exercise mock selection using
        # a temporary copy, without changing saved IPs or surveyed transforms.
        data["pipeline"]["backend"] = "mock"
        for arm in data["arms"].values():
            arm["robot_ip"] = ""
        for settings in data["arm_settings"].values():
            settings.pop("world_from_base_xyz_xyzw", None)
        path = self.write(data)
        arms = select_arms(load_arms(path), "dual", "right")
        self.assertEqual(
            [a.parameters["controller_id"] for a in arms], ["left", "right"]
        )
        self.assertEqual(data["pipeline"]["mode"], "dual")
        self.assertNotIn("single_controller", data["pipeline"])
        self.assertEqual(
            set(data["pipeline"]), set(PIPELINE_DEFAULTS) - {"single_controller"}
        )
        self.assertTrue(
            all(
                "controller_enabled" not in a and "operator_confirm" not in a
                for a in data["arms"].values()
            )
        )
        self.assertEqual(data["pipeline"]["backend"], "mock")
        self.assertEqual(len({a.namespace for a in arms}), 2)
        self.assertEqual(len({a.parameters["base_frame"] for a in arms}), 2)
        self.assertTrue(all(not a.robot_ip for a in arms))
        self.assertTrue(
            all(
                "world_from_base_xyz_xyzw" not in v
                for v in data["arm_settings"].values()
            )
        )
        load_runtime(path, arms)
        for section in ALLOWED:
            expected = ALLOWED[section] - (
                {
                    "grasp_force_n",
                    "grasp_width_m",
                    "epsilon_inner_m",
                    "epsilon_outer_m",
                    "grip_click_max_ms",
                    "grip_double_click_ms",
                }
                if section == "hand"
                else set()
            )
            self.assertEqual(set(data[section]), expected)

    def test_namespaced_bridge_and_per_arm_settings(self):
        self.data.update(
            quest={"bridge": {"namespace": "/input"}},
            controller={"gain_translation": 5.0},
            arm_settings={
                "arm": {
                    "controller": {"gain_translation": 3.0},
                    "world_from_base_xyz_xyzw": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                }
            },
        )
        arms = load_arms(self.write(self.data))
        self.assertEqual(
            arms[0].parameters["controller_state_topic"],
            "/input/quest_usb_bridge/right/controller_state",
        )
        left = select_arms(arms, "single", "left")[0]
        self.assertEqual(
            left.parameters["controller_state_topic"],
            "/input/quest_usb_bridge/left/controller_state",
        )
        self.assertEqual(
            load_runtime(self.path, arms)["arm_settings"]["arm"]["controller"][
                "gain_translation"
            ],
            3.0,
        )

    def test_actuator_scales_are_global_and_independent_of_cbf_switch(self):
        self.data.update(
            cbf={"torque_limit_scale": 0.2, "torque_rate_limit_scale": 0.1}
        )
        for enabled in (False, True):
            self.data["pipeline"]["cbf_enabled"] = enabled
            arms = load_arms(self.write(self.data))
            self.assertEqual(load_runtime(self.path, arms)["cbf"], self.data["cbf"])
        for key in ("torque_limit_scale", "torque_rate_limit_scale"):
            for invalid in (0.0, -1.0, 1.01, float("nan"), float("inf"), True, "0.35"):
                with self.subTest(key=key, invalid=invalid):
                    self.data["cbf"] = {key: invalid}
                    self.write(self.data)
                    with self.assertRaises(ValueError):
                        load_runtime(self.path, arms)

    def test_invalid_schema_and_config_confirmation_are_rejected(self):
        changes = [
            dict(extra={}),
            dict(pipeline={"operator_confirm": True}),
            dict(pipeline={"start_hand": "true"}),
            dict(safety={"torque_limit_scale": 0.2}),
            dict(pipeline={"backend": "other"}),
            dict(quest={"bridge": {"typo": 3}}),
            dict(quest={"bridge": {"listen_port": 0}}),
            dict(quest={"bridge": {"listen_port": True}}),
            dict(quest={"bridge": {"namespace": "bad name"}}),
            dict(recording={"typo": 3}),
            dict(recording={"menu_debounce_seconds": float("nan")}),
            dict(controller={"typo": 3}),
            dict(arm_settings={"unknown": {}}),
        ]
        for change in changes:
            with self.subTest(change=change):
                data = copy.deepcopy(self.data)
                data.update(change)
                with self.assertRaises(ValueError):
                    path = self.write(data)
                    arms = load_arms(path)
                    load_runtime(path, arms)


if __name__ == "__main__":
    unittest.main()
