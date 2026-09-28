import math
import sys
import tempfile
import unittest
from pathlib import Path

from generate_vertical_land import Config, row_at, sample_times, write_csv


class VerticalLandGeneratorTest(unittest.TestCase):
    def test_climb_and_hover_are_continuous(self):
        config = Config(start_north=0.2, start_east=-0.3,
                        start_down=0.4, height=0.8)
        for t in (0.0, 4.0, 6.0):
            point = row_at(t, config)
            self.assertEqual(point[1:3], (0.2, -0.3))
            self.assertTrue(all(abs(value) < 1e-8 for value in point[4:10]))
        self.assertAlmostEqual(row_at(0.0, config)[3], 0.4)
        self.assertAlmostEqual(row_at(4.0, config)[3], -0.4)
        self.assertAlmostEqual(row_at(6.0, config)[3], -0.4)

    def test_optional_return_to_low_hover(self):
        config = Config(start_north=0.2, start_east=-0.3,
                        start_down=0.4, start_yaw=0.6, height=0.8,
                        descent_duration=5.0, return_height=0.2,
                        final_hover_duration=3.0)
        for t, expected_z in ((0.0, 0.4), (4.0, -0.4),
                              (6.0, -0.4), (11.0, 0.2), (14.0, 0.2)):
            point = row_at(t, config)
            self.assertEqual(point[1:3], (0.2, -0.3))
            self.assertAlmostEqual(point[3], expected_z)
            self.assertAlmostEqual(point[10], 0.6)
            self.assertTrue(all(abs(value) < 1e-8 for value in point[4:10]))
        self.assertAlmostEqual(row_at(8.5, config)[3], -0.1)
        self.assertGreater(row_at(8.5, config)[6], 0.0)
        self.assertEqual(sample_times(config, 50)[-1], 14.0)

    def test_descent_derivatives_and_csv_parser(self):
        config = Config(descent_duration=5.0, return_height=0.2)
        t, step = 8.0, 1e-4
        before, centre, after = (row_at(t + offset, config)
                                 for offset in (-step, 0.0, step))
        self.assertAlmostEqual((after[3] - before[3])/(2*step),
                               centre[6], places=5)
        self.assertAlmostEqual((after[6] - before[6])/(2*step),
                               centre[9], places=5)
        package_root = Path(__file__).resolve().parents[1] / 'src' / 'trajectory_follower'
        sys.path.insert(0, str(package_root))
        from trajectory_follower.trajectory import load_trajectory
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'return.csv'
            count = write_csv(path, config)
            points = load_trajectory(path)
            self.assertEqual(len(points), count)
            self.assertAlmostEqual(points[-1].z, -0.2)
            self.assertEqual(points[-1].vz, 0.0)

    def test_vertical_derivatives_and_csv_parser(self):
        config = Config()
        t, step = 1.2, 1e-4
        before, centre, after = (row_at(t + offset, config)
                                 for offset in (-step, 0.0, step))
        self.assertAlmostEqual((after[3] - before[3])/(2*step),
                               centre[6], places=5)
        self.assertAlmostEqual((after[6] - before[6])/(2*step),
                               centre[9], places=5)
        package_root = Path(__file__).resolve().parents[1] / 'src' / 'trajectory_follower'
        sys.path.insert(0, str(package_root))
        from trajectory_follower.trajectory import load_trajectory
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'vertical.csv'
            count = write_csv(path, config)
            points = load_trajectory(path)
            self.assertEqual(len(points), count)
            self.assertEqual(points[-1].z, -1.0)
            self.assertEqual(points[-1].vz, 0.0)

    def test_rejects_invalid_inputs(self):
        with self.assertRaises(ValueError):
            Config(height=-1).validate()
        with self.assertRaises(ValueError):
            Config(start_yaw=math.nan).validate()
        with self.assertRaises(ValueError):
            sample_times(Config(), 5)
        with self.assertRaises(ValueError):
            Config(descent_duration=5.0).validate()
        with self.assertRaises(ValueError):
            Config(descent_duration=5.0, return_height=1.0).validate()
        with self.assertRaises(ValueError):
            Config(return_height=0.2).validate()


if __name__ == '__main__':
    unittest.main()
