import copy
import json
from pathlib import Path
import tempfile
import unittest

import yaml
from quest_teleop.configuration import load_arms, recording_plan, select_arms


class ConfigurationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = {'arms': {
            'left_arm': {'namespace': '/left_arm', 'controller': 'left', 'robot_ip': ''},
            'right_arm': {'namespace': '/right_arm', 'controller': 'right', 'robot_ip': ''}}}

    def write(self, name, value):
        path = self.root / name
        path.write_text(yaml.safe_dump(value))
        return path

    def test_independent_topics_frames_and_controller_assignment(self):
        arms = load_arms(self.write('arms.yaml', self.config))
        self.assertEqual([a.parameters['controller_id'] for a in arms], ['left', 'right'])
        self.assertEqual(arms[0].parameters['controller_state_topic'], '/quest_usb_bridge/left/controller_state')
        self.assertEqual(arms[1].parameters['command_topic'], '/right_arm/cartesian_teleop_command')
        self.assertNotEqual(arms[0].parameters['base_frame'], arms[1].parameters['base_frame'])
        self.assertEqual(arms[1].parameters['reference_to_robot_xyzw'], [0.0, 0.0, 0.0, 1.0])

    def test_reject_collisions_bad_calibration_and_invalid_settings(self):
        cases = [
            ('namespace', '/left_arm'), ('base_frame', 'left_arm/panda_link0'),
            ('command_topic', '/left_arm/cartesian_teleop_command'),
            ('target_pose_topic', '/left_arm/target_pose'),
            ('measured_pose_topic', '/left_arm/teleop_anchor_pose'),
            ('command_topic', '/right_arm/teleop_anchor_pose'),
            ('controller_state_topic', '/right_arm/cartesian_teleop_command'),
            ('controller', 'both'), ('reference_to_robot_xyzw', [0.0]*4),
            ('reference_to_robot_xyzw', [0.0, 0.0, float('nan'), 1.0]),
            ('feedforward_scale', 1.1), ('translation_scale', 0),
            ('twist_filter_cutoff_hz', float('inf')), ('command_timeout_ms', 1.5),
            ('trigger_release', 0.8), ('min_derivative_dt_ms', 101.0),
            ('controller_enabled', True), ('operator_confirm', True), ('trigger_typo', 0.5)]
        for key, value in cases:
            with self.subTest(key=key, value=value):
                bad = copy.deepcopy(self.config)
                bad['arms']['right_arm'][key] = value
                with self.assertRaises(ValueError):
                    load_arms(self.write('bad.yaml', bad))
        bad = copy.deepcopy(self.config)
        for arm in bad['arms'].values():
            arm['robot_ip'] = '192.0.2.1'
        with self.assertRaises(ValueError):
            load_arms(self.write('duplicate_ip.yaml', bad))

    def test_single_defaults_right_and_left_override_and_dual_assignments(self):
        arms = load_arms(self.write('arms.yaml', self.config))
        self.assertEqual(select_arms(arms)[0].parameters['controller_id'], 'right')
        self.assertEqual(select_arms(arms, single_controller='left')[0].name, 'left_arm')
        self.assertEqual([a.parameters['controller_id'] for a in select_arms(arms, 'dual')], ['left','right'])
        one = [arms[1]]
        chosen = select_arms(one, single_controller='left')[0]
        self.assertEqual(chosen.name, 'right_arm')
        self.assertEqual(chosen.parameters['controller_state_topic'], '/quest_usb_bridge/left/controller_state')
        self.assertEqual(one[0].parameters['controller_id'], 'right')
        with self.assertRaises(ValueError): select_arms(one, 'dual')
        with self.assertRaises(ValueError): select_arms(arms, 'other')

    def test_recording_can_replace_extend_and_deduplicate_topics(self):
        arms = load_arms(self.write('arms.yaml', self.config))
        config = self.write('recording.yaml', {'include_default_topics': True,
            'topics': ['/custom', '/custom'], 'output_root': str(self.root)})
        default = recording_plan(config, arms)
        self.assertIn('/left_arm/cartesian_teleop_command', default.topics)
        self.assertIn('/right_arm/gripper_events', default.topics)
        self.assertEqual(default.topics.count('/custom'), 1)
        override = recording_plan(config, arms, json.dumps(['/selected', '/selected']))
        self.assertEqual(override.topics, ['/selected'])
        self.assertNotIn('/custom', override.command)
        self.assertNotEqual(default.output, override.output)
        self.assertFalse(Path(override.output).exists())
        replacement = self.write('replacement.yaml', {'include_default_topics': False,
            'topics': ['/only'], 'output_root': str(self.root)})
        self.assertEqual(recording_plan(replacement, arms).topics, ['/only'])
        with self.assertRaises(ValueError):
            recording_plan(config, arms, '[]')
        with self.assertRaises(ValueError):
            recording_plan(config, arms, '["--all"]')
        with self.assertRaises(ValueError):
            recording_plan(config, arms, output_root_override='relative')


if __name__ == '__main__':
    unittest.main()
