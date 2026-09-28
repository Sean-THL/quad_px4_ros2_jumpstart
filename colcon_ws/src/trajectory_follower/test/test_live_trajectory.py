import math
import unittest

from trajectory_follower.live_trajectory import (
    TrajectoryConfig, desired_state, figure_eight_offset,
    lissajous_figure_eight_offset)


class LiveTrajectoryTest(unittest.TestCase):
    def setUp(self):
        self.config = TrajectoryConfig()
        self.origin = (2.0, -1.0, 0.3)

    def test_starts_at_origin_at_rest(self):
        point = desired_state(0.0, self.origin, 0.4, self.config)
        self.assertEqual((point.x, point.y, point.z), self.origin)
        self.assertEqual((point.vx, point.vy, point.vz), (0.0, 0.0, 0.0))
        self.assertEqual((point.ax, point.ay, point.az), (0.0, 0.0, 0.0))

    def test_joins_are_continuous_through_acceleration(self):
        for t in (self.config.takeoff_duration_s,
                  self.config.takeoff_duration_s +
                  self.config.flight_duration_s):
            before = desired_state(t - 1e-5, self.origin, 0.4, self.config)
            at = desired_state(t, self.origin, 0.4, self.config)
            after = desired_state(t + 1e-5, self.origin, 0.4, self.config)
            fields = ('x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az')
            for name in fields:
                self.assertAlmostEqual(
                    getattr(before, name), getattr(at, name), places=3)
                self.assertAlmostEqual(
                    getattr(after, name), getattr(at, name), places=3)

    def test_final_state_is_cruise_hover_and_clamps(self):
        end = desired_state(
            self.config.duration_s, self.origin, -0.2, self.config)
        later = desired_state(self.config.duration_s + 100.0,
                              self.origin, -0.2, self.config)
        self.assertEqual(end, later)
        self.assertAlmostEqual(
            end.z, self.origin[2] - self.config.takeoff_height_m)
        self.assertTrue(all(math.isclose(value, 0.0) for value in
                            (end.vx, end.vy, end.vz,
                             end.ax, end.ay, end.az)))

    def test_rejects_bad_configuration(self):
        with self.assertRaises(ValueError):
            desired_state(0.0, self.origin, 0.0,
                          TrajectoryConfig(flight_duration_s=0.0))

    def test_alternative_lissajous_has_smooth_endpoints(self):
        path = lissajous_figure_eight_offset
        for t in (0.0, self.config.flight_duration_s):
            position, velocity, acceleration = path(t, self.config)
            for values in (position, velocity, acceleration):
                for value in values:
                    self.assertAlmostEqual(value, 0.0, places=10)

    def test_alternative_lissajous_differs_at_center_crossing(self):
        t = self.config.flight_duration_s / 2.0
        original_position = figure_eight_offset(t, self.config)[0]
        new_position = lissajous_figure_eight_offset(t, self.config)[0]
        self.assertAlmostEqual(original_position[2], 0.0, places=10)
        self.assertAlmostEqual(new_position[2],
                               -self.config.vertical_amplitude_m, places=10)

    def test_alternative_lissajous_derivatives_match_position(self):
        path = lissajous_figure_eight_offset
        t = self.config.flight_duration_s * 0.37
        dt = 1e-4
        before = path(t - dt, self.config)
        at = path(t, self.config)
        after = path(t + dt, self.config)
        for axis in range(3):
            numerical_velocity = (after[0][axis] - before[0][axis]) / (2*dt)
            numerical_acceleration = (after[1][axis] - before[1][axis]) / (2*dt)
            self.assertAlmostEqual(at[1][axis], numerical_velocity, places=5)
            self.assertAlmostEqual(at[2][axis], numerical_acceleration, places=4)


if __name__ == '__main__':
    unittest.main()
