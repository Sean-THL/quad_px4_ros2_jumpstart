import math
import tempfile
import unittest
from pathlib import Path

from generate_figure8 import FigureEight, row_at, sample_times, write_csv


class FigureEightTest(unittest.TestCase):
    def test_takeoff_loop_and_hold_join_at_rest(self):
        config = FigureEight()
        rows = [row_at(t, config) for t in (0.0, 4.0, 16.0, 18.0)]
        self.assertEqual(rows[0][1:4], (0.0, 0.0, 0.0))
        for row in rows:
            self.assertAlmostEqual(row[4], 0.0, places=8)
            self.assertAlmostEqual(row[5], 0.0, places=8)
            self.assertAlmostEqual(row[6], 0.0, places=8)
            self.assertAlmostEqual(row[7], 0.0, places=8)
            self.assertAlmostEqual(row[8], 0.0, places=8)
            self.assertAlmostEqual(row[9], 0.0, places=8)
        self.assertAlmostEqual(rows[1][3], -1.0)
        self.assertAlmostEqual(rows[2][3], -1.0)
        for before, after in zip(rows[2][1:4], rows[3][1:4]):
            self.assertAlmostEqual(before, after)

    def test_two_crossings_have_different_altitudes(self):
        config = FigureEight(takeoff_duration=2.0, loop_duration=8.0,
                             height_variation=0.4)
        first = row_at(2.0, config)
        second = row_at(6.0, config)  # ease(0.5) = 0.5, so theta = pi
        self.assertAlmostEqual(first[1], second[1])
        self.assertAlmostEqual(first[2], second[2])
        self.assertAlmostEqual(second[3] - first[3], -0.4)

    def test_tangent_yaw_rate_and_csv(self):
        config = FigureEight(yaw_mode='tangent', start_yaw=0.0)
        middle = row_at(10.0, config)
        self.assertTrue(all(math.isfinite(value) for value in middle))
        times = sample_times(config, 50)
        self.assertEqual(times[0], 0.0)
        self.assertEqual(times[-1], 18.0)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'figure8.csv'
            self.assertEqual(write_csv(path, config, 50), len(times))
            from sys import path as sys_path
            sys_path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' /
                                   'trajectory_follower'))
            from trajectory_follower.trajectory import load_trajectory
            self.assertEqual(len(load_trajectory(path)), len(times))

    def test_analytic_derivatives_match_position_changes(self):
        config = FigureEight(yaw_mode='tangent')
        time = 7.3
        step = 1e-4
        before = row_at(time - step, config)
        centre = row_at(time, config)
        after = row_at(time + step, config)
        for axis in range(3):
            self.assertAlmostEqual((after[1 + axis] - before[1 + axis]) /
                                   (2*step), centre[4 + axis], places=5)
            self.assertAlmostEqual((after[4 + axis] - before[4 + axis]) /
                                   (2*step), centre[7 + axis], places=5)
        yaw_delta = (after[10] - before[10] + math.pi) % (2*math.pi) - math.pi
        self.assertAlmostEqual(yaw_delta/(2*step), centre[11], places=5)


if __name__ == '__main__':
    unittest.main()
