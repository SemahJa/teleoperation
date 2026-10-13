import copy
import json
from pathlib import Path
import tempfile
import unittest

import yaml
from quest_teleop.configuration import load_arms
from recording.configuration import recording_plan


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

    def test_unified_recording_profile_and_namespaced_menu_input(self):
        data = dict(self.config, pipeline={}, quest={'bridge':{'namespace':'/input'}},
            recording={'include_default_topics':True,'topics':['/custom'],
                'output_root':str(self.root),'mode':'menu','namespace':'/episodes','menu_debounce_seconds':.4})
        path = self.write('runtime.yaml',data)
        plan = recording_plan(path,load_arms(path))
        self.assertIn('/input/quest_usb_bridge/left/controller_state',plan.topics)
        self.assertIn('/input/quest_usb_bridge/right/controller_state',plan.topics)
        self.assertNotIn('/quest_usb_bridge/left/controller_state',plan.topics)
        self.assertIn('/custom',plan.topics)


if __name__ == '__main__':
    unittest.main()
