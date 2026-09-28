import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from generate_trajectory import Config, flight_at, row_at, write_csv


class TrajectoryTemplateTest(unittest.TestCase):
    def test_flight_equations_use_seconds(self):
        config = Config(flight_duration=8.0, north_radius=1.5)
        start, _, _ = flight_at(0.0, config)
        middle, _, _ = flight_at(4.0, config)
        finish, _, _ = flight_at(8.0, config)
        self.assertAlmostEqual(start[0], 0.0)
        self.assertAlmostEqual(middle[0], 1.5)
        self.assertAlmostEqual(finish[0], 0.0)
        self.assertEqual((start[2], middle[2], finish[2]), (0.0, 0.0, 0.0))

    def test_velocity_acceleration_follow_position_equation(self):
        with patch('generate_trajectory.position_at',
                   side_effect=lambda t, config: (t**2, 0, 0)):
            position, velocity, acceleration = flight_at(2.0, Config())
        self.assertEqual(position, (4.0, 0.0, 0.0))
        self.assertEqual(velocity, (4.0, 0.0, 0.0))
        self.assertEqual(acceleration, (2.0, 0.0, 0.0))

    def test_start_finish_and_continuous_segment_boundaries(self):
        config = Config(start_north=0.2, start_east=-0.1, start_down=0.3)
        for t in (0.0, config.takeoff_duration,
                  config.takeoff_duration + config.flight_duration,
                  config.takeoff_duration + config.flight_duration + config.hold_duration):
            row = row_at(t, config)
            self.assertAlmostEqual(row[1], config.start_north)
            self.assertAlmostEqual(row[2], config.start_east)
            self.assertTrue(all(abs(value) < 1e-8 for value in row[4:10]))
        self.assertAlmostEqual(row_at(0.0, config)[3], 0.3)
        self.assertAlmostEqual(row_at(config.takeoff_duration +
                                      config.flight_duration +
                                      config.hold_duration, config)[3], -0.7)

    def test_derivatives_and_follower_compatibility(self):
        config = Config()
        t, step = 7.3, 1e-4
        before, centre, after = (row_at(t + offset, config)
                                 for offset in (-step, 0.0, step))
        for axis in range(3):
            self.assertAlmostEqual((after[axis + 1] - before[axis + 1]) / (2*step),
                                   centre[axis + 4], places=5)
            self.assertAlmostEqual((after[axis + 4] - before[axis + 4]) / (2*step),
                                   centre[axis + 7], places=5)
        package_root = Path(__file__).resolve().parents[1] / 'src' / 'trajectory_follower'
        sys.path.insert(0, str(package_root))
        from trajectory_follower.trajectory import load_trajectory
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'trajectory.csv'
            count = write_csv(path, config)
            self.assertEqual(len(load_trajectory(path)), count)

    def test_rejects_invalid_input(self):
        with self.assertRaises(ValueError):
            Config(flight_duration=0).validate()
        with self.assertRaises(ValueError):
            Config(start_yaw=math.nan).validate()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'unsafe.csv'
            with patch('generate_trajectory.position_at',
                       side_effect=lambda t, config: (t + 1, 0, 0)):
                with self.assertRaisesRegex(ValueError, 'join takeoff and hover'):
                    write_csv(path, Config())
            self.assertFalse(path.exists())


if __name__ == '__main__':
    unittest.main()
