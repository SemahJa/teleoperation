import unittest

from quest_teleop.trajectory_processing import TrajectoryProcessor


class TrajectoryProcessingTest(unittest.TestCase):
    def test_modes_are_independent_and_reset(self):
        for mode in ("none", "low_pass", "four_pole"):
            processor = TrajectoryProcessor(mode)
            self.assertEqual(processor.update((0.0,), 1_000_000_000), (0.0,))
            value = processor.update((1.0,), 1_010_000_000)[0]
            if mode == "none":
                self.assertEqual(value, 1.0)
            else:
                self.assertGreater(value, 0.0)
                self.assertLess(value, 1.0)
            processor.reset()
            self.assertEqual(processor.update((2.0,), 2_000_000_000), (2.0,))

    def test_interpolators_start_continuously_and_reach_target(self):
        for interpolation in ("ptp_polynomial", "spline"):
            processor = TrajectoryProcessor(interpolation=interpolation,
                                            duration_ms=40.0)
            processor.update((0.0,), 1_000_000_000)
            self.assertEqual(processor.update((1.0,), 1_010_000_000), (0.0,))
            self.assertAlmostEqual(processor.update((1.0,), 1_050_000_000)[0], 1.0)

    def test_invalid_options(self):
        for args in ({"filter_mode": "bad"}, {"interpolation": "bad"},
                     {"cutoff_hz": 0}, {"duration_ms": 0}):
            with self.assertRaises(ValueError):
                TrajectoryProcessor(**args)


if __name__ == "__main__":
    unittest.main()
