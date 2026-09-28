import math
import tempfile
import unittest
from pathlib import Path

from trajectory_follower.trajectory import FIELDS, Setpoint, load_trajectory, sample


class TrajectoryTest(unittest.TestCase):
    def test_quintic_position_velocity_and_acceleration(self):
        start = Setpoint(0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        end = Setpoint(2.0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        midpoint = sample([start, end], [0.0, 2.0], 1.0)
        self.assertAlmostEqual(midpoint.x, 0.5)
        self.assertAlmostEqual(midpoint.vx, 0.9375)
        self.assertAlmostEqual(midpoint.ax, 0.0)
        self.assertEqual(sample([start, end], [0.0, 2.0], 3.0), end)

    def test_yaw_uses_shortest_path_with_consistent_rate(self):
        start = Setpoint(0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, math.radians(170), 0)
        end = Setpoint(2.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, math.radians(-170), 0)
        midpoint = sample([start, end], [0.0, 2.0], 1.0)
        self.assertAlmostEqual(abs(midpoint.yaw), math.pi)
        self.assertGreater(midpoint.yawspeed, 0)

    def test_csv_requires_finite_increasing_times(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'trajectory.csv'
            path.write_text(','.join(FIELDS) + '\n' +
                            '0,0,0,0,0,0,0,0,0,0,0,0\n' +
                            '1,1,0,0,0,0,0,0,0,0,0,0\n')
            self.assertEqual(len(load_trajectory(path)), 2)
            path.write_text(','.join(FIELDS) + '\n' +
                            '0,0,0,0,0,0,0,0,0,0,0,0\n' +
                            '0,nan,0,0,0,0,0,0,0,0,0,0\n')
            with self.assertRaises(ValueError):
                load_trajectory(path)


if __name__ == '__main__':
    unittest.main()
