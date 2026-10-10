import math
import unittest

from trajectory_follower.live_trajectory import (
    DesiredState, TrajectoryConfig, desired_state, double_spiral_offset,
    figure_eight_offset, lissajous_figure_eight_offset, offset_spiral_offset,
    spiral_offset, velocity_aligned_state)


class LiveTrajectoryTest(unittest.TestCase):
    def setUp(self):
        self.config = TrajectoryConfig()
        self.origin = (2.0, -1.0, 0.3)

    def test_starts_at_origin_at_rest(self):
        point = desired_state(0.0, self.origin, 0.4, self.config)
        self.assertEqual((point.x, point.y, point.z), self.origin)
        self.assertEqual((point.vx, point.vy, point.vz), (0.0, 0.0, 0.0))
        self.assertEqual((point.ax, point.ay, point.az), (0.0, 0.0, 0.0))

    def test_prearmed_hover_target_is_constant_until_path_starts(self):
        config = TrajectoryConfig(takeoff_profile='prearmed_hover')
        target_z = config.takeoff_target_z_m
        for origin in (self.origin, (2.0, -1.0, -0.17)):
            for t in (0.0, config.takeoff_duration_s / 2.0,
                      config.takeoff_duration_s):
                point = desired_state(t, origin, 0.4, config)
                self.assertEqual((point.x, point.y, point.z),
                                 (origin[0], origin[1], target_z))
                self.assertEqual((point.vx, point.vy, point.vz),
                                 (0.0, 0.0, 0.0))
        after = desired_state(config.takeoff_duration_s + 1e-4,
                              self.origin, 0.4, config)
        self.assertAlmostEqual(after.z, target_z, places=6)

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
        with self.assertRaises(ValueError):
            desired_state(0.0, self.origin, 0.0,
                          TrajectoryConfig(trajectory_type='unknown'))
        with self.assertRaises(ValueError):
            TrajectoryConfig(yaw_mode='unknown').validate()
        with self.assertRaises(ValueError):
            TrajectoryConfig(takeoff_profile='unknown').validate()

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

    def test_spiral_has_smooth_endpoints_and_expected_radius(self):
        config = TrajectoryConfig(trajectory_type='spiral')
        start = spiral_offset(0.0, config)
        end = spiral_offset(config.flight_duration_s, config)
        self.assertEqual(start[0], (0.0, 0.0, 0.0))
        for derivatives in (start[1], start[2], end[1], end[2]):
            for value in derivatives:
                self.assertAlmostEqual(value, 0.0, places=9)
        self.assertAlmostEqual(math.hypot(end[0][0], end[0][1]),
                               config.spiral_radius_m)
        self.assertAlmostEqual(end[0][2], 0.0)

    def test_spiral_derivatives_match_position(self):
        config = TrajectoryConfig(
            trajectory_type='spiral', spiral_max_height_m=0.3)
        t = config.flight_duration_s * 0.37
        dt = 1e-4
        before = spiral_offset(t - dt, config)
        at = spiral_offset(t, config)
        after = spiral_offset(t + dt, config)
        for axis in range(3):
            numerical_velocity = (after[0][axis] - before[0][axis]) / (2*dt)
            numerical_acceleration = (after[1][axis] - before[1][axis]) / (2*dt)
            self.assertAlmostEqual(at[1][axis], numerical_velocity, places=5)
            self.assertAlmostEqual(at[2][axis], numerical_acceleration, places=4)

    def test_spiral_final_hold_stays_at_endpoint(self):
        config = TrajectoryConfig(
            trajectory_type='spiral', spiral_radius_m=0.7,
            spiral_turns=1.5, spiral_max_height_m=0.3)
        end = desired_state(config.duration_s, self.origin, 0.2, config)
        later = desired_state(config.duration_s + 10.0,
                              self.origin, 0.2, config)
        self.assertEqual(end, later)
        self.assertAlmostEqual(
            math.hypot(end.x - self.origin[0], end.y - self.origin[1]),
            config.spiral_radius_m)
        self.assertAlmostEqual(
            end.z, self.origin[2] - config.takeoff_height_m -
            config.spiral_max_height_m)

    def test_spiral_respects_max_height_without_dropping(self):
        config = TrajectoryConfig(
            trajectory_type='spiral', spiral_max_height_m=0.35)
        samples = [
            spiral_offset(config.flight_duration_s * index / 100.0, config)
            for index in range(101)]
        heights = [-sample[0][2] for sample in samples]
        self.assertAlmostEqual(heights[0], 0.0)
        self.assertAlmostEqual(heights[-1], config.spiral_max_height_m)
        self.assertTrue(all(0.0 <= height <= config.spiral_max_height_m
                            for height in heights))
        self.assertTrue(all(first <= second for first, second in
                            zip(heights, heights[1:])))
        for values in (samples[-1][1], samples[-1][2]):
            for value in values:
                self.assertAlmostEqual(value, 0.0, places=9)

    def test_double_spiral_returns_to_takeoff_hover(self):
        config = TrajectoryConfig(
            trajectory_type='double_spiral', spiral_radius_m=0.6,
            spiral_turns=1.0, spiral_max_height_m=0.35)
        start = double_spiral_offset(0.0, config)
        midpoint = double_spiral_offset(config.flight_duration_s / 2.0,
                                        config)
        end = double_spiral_offset(config.flight_duration_s, config)
        self.assertEqual(start[0], (0.0, 0.0, 0.0))
        self.assertAlmostEqual(
            math.hypot(midpoint[0][0], midpoint[0][1]),
            config.spiral_radius_m)
        self.assertAlmostEqual(midpoint[0][2],
                               -config.spiral_max_height_m)
        for values in (midpoint[1], midpoint[2]):
            for value in values:
                self.assertAlmostEqual(value, 0.0, places=9)
        for values in end:
            for value in values:
                self.assertAlmostEqual(value, 0.0, places=9)

        final = desired_state(config.duration_s, self.origin, 0.2, config)
        self.assertAlmostEqual(final.x, self.origin[0])
        self.assertAlmostEqual(final.y, self.origin[1])
        self.assertAlmostEqual(
            final.z, self.origin[2] - config.takeoff_height_m)

    def test_double_spiral_derivatives_and_joins_are_continuous(self):
        config = TrajectoryConfig(
            trajectory_type='double_spiral', spiral_max_height_m=0.3)
        dt = 1e-4
        for fraction in (0.31, 0.73):
            t = config.flight_duration_s * fraction
            before = double_spiral_offset(t - dt, config)
            at = double_spiral_offset(t, config)
            after = double_spiral_offset(t + dt, config)
            for axis in range(3):
                numerical_velocity = (
                    after[0][axis] - before[0][axis]) / (2*dt)
                numerical_acceleration = (
                    after[1][axis] - before[1][axis]) / (2*dt)
                self.assertAlmostEqual(
                    at[1][axis], numerical_velocity, places=5)
                self.assertAlmostEqual(
                    at[2][axis], numerical_acceleration, places=4)

        fields = ('x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az')
        for t in (config.takeoff_duration_s +
                  config.flight_duration_s / 2.0,
                  config.takeoff_duration_s + config.flight_duration_s):
            before = desired_state(t - 1e-5, self.origin, 0.4, config)
            at = desired_state(t, self.origin, 0.4, config)
            after = desired_state(t + 1e-5, self.origin, 0.4, config)
            for name in fields:
                self.assertAlmostEqual(
                    getattr(before, name), getattr(at, name), places=3)
                self.assertAlmostEqual(
                    getattr(after, name), getattr(at, name), places=3)

    def test_offset_spiral_fixed_edge_and_limits(self):
        config = TrajectoryConfig(
            trajectory_type='offset_spiral',
            flight_duration_s=12.0,
            offset_spiral_entry_duration_s=2.0,
            offset_spiral_initial_radius_m=0.2,
            offset_spiral_max_radius_m=0.8,
            offset_spiral_max_height_m=0.5,
            offset_spiral_turns=1.0,
            offset_spiral_x_align_m=-0.4,
            offset_spiral_y_center_m=0.15,
            offset_spiral_z0_m=-0.1)
        helix_duration = (config.flight_duration_s -
                          config.offset_spiral_entry_duration_s)

        # At half helix time, smoothstep(0.5)=0.5 and theta=pi: this is
        # exactly the tangent point on the aligned plane.
        left, _, _ = offset_spiral_offset(
            config.offset_spiral_entry_duration_s + helix_duration / 2.0,
            config)
        self.assertAlmostEqual(left[0], config.offset_spiral_x_align_m)
        self.assertAlmostEqual(left[1], config.offset_spiral_y_center_m)

        end, velocity, acceleration = offset_spiral_offset(
            config.flight_duration_s, config)
        self.assertAlmostEqual(
            end[0], config.offset_spiral_x_align_m +
            2.0 * config.offset_spiral_max_radius_m)
        self.assertAlmostEqual(end[1], config.offset_spiral_y_center_m)
        self.assertAlmostEqual(
            end[2], config.offset_spiral_z0_m -
            config.offset_spiral_max_height_m)
        for values in (velocity, acceleration):
            for value in values:
                self.assertAlmostEqual(value, 0.0, places=9)

    def test_offset_spiral_joins_are_continuous(self):
        config = TrajectoryConfig(
            trajectory_type='offset_spiral',
            offset_spiral_x_align_m=0.1,
            offset_spiral_y_center_m=-0.2,
            offset_spiral_z0_m=-0.1)
        joins = (
            config.takeoff_duration_s,
            config.takeoff_duration_s +
            config.offset_spiral_entry_duration_s,
            config.takeoff_duration_s + config.flight_duration_s,
        )
        fields = ('x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az')
        for t in joins:
            before = desired_state(t - 1e-5, self.origin, 0.4, config)
            at = desired_state(t, self.origin, 0.4, config)
            after = desired_state(t + 1e-5, self.origin, 0.4, config)
            for name in fields:
                self.assertAlmostEqual(
                    getattr(before, name), getattr(at, name), places=3)
                self.assertAlmostEqual(
                    getattr(after, name), getattr(at, name), places=3)

    def test_offset_spiral_y_aligned_edge_and_limits(self):
        config = TrajectoryConfig(
            trajectory_type='offset_spiral',
            flight_duration_s=12.0,
            offset_spiral_entry_duration_s=2.0,
            offset_spiral_initial_radius_m=0.2,
            offset_spiral_max_radius_m=0.8,
            offset_spiral_max_height_m=0.5,
            offset_spiral_turns=1.0,
            offset_spiral_alignment_axis='y',
            offset_spiral_y_align_m=-0.4,
            offset_spiral_x_center_m=0.15)
        helix_duration = (config.flight_duration_s -
                          config.offset_spiral_entry_duration_s)
        edge, _, _ = offset_spiral_offset(
            config.offset_spiral_entry_duration_s + helix_duration / 2.0,
            config)
        self.assertAlmostEqual(edge[0], config.offset_spiral_x_center_m)
        self.assertAlmostEqual(edge[1], config.offset_spiral_y_align_m)

        end, velocity, acceleration = offset_spiral_offset(
            config.flight_duration_s, config)
        self.assertAlmostEqual(end[0], config.offset_spiral_x_center_m)
        self.assertAlmostEqual(
            end[1], config.offset_spiral_y_align_m +
            2.0 * config.offset_spiral_max_radius_m)
        for values in (velocity, acceleration):
            for value in values:
                self.assertAlmostEqual(value, 0.0, places=9)

    def test_offset_spiral_rejects_inconsistent_limits(self):
        with self.assertRaises(ValueError):
            TrajectoryConfig(
                trajectory_type='offset_spiral',
                offset_spiral_initial_radius_m=0.7,
                offset_spiral_max_radius_m=0.6).validate()
        with self.assertRaises(ValueError):
            TrajectoryConfig(
                trajectory_type='offset_spiral', flight_duration_s=2.0,
                offset_spiral_entry_duration_s=2.0).validate()
        with self.assertRaises(ValueError):
            TrajectoryConfig(
                offset_spiral_alignment_axis='z').validate()
        with self.assertRaises(ValueError):
            TrajectoryConfig(spiral_max_height_m=-0.1).validate()

    def test_velocity_yaw_faces_horizontal_velocity(self):
        point = DesiredState(
            0.0, 0.0, 0.0, 1.0, 1.0, -0.2,
            -0.5, 0.5, 0.0, -1.0, 0.0)
        aligned = velocity_aligned_state(point, previous_yaw=-0.7)
        self.assertAlmostEqual(aligned.yaw, math.pi / 4.0)
        self.assertAlmostEqual(aligned.yawspeed, 0.5)

    def test_velocity_yaw_holds_last_heading_when_stopped(self):
        point = DesiredState(
            0.0, 0.0, 0.0, 0.0, 0.0, -0.2,
            0.0, 0.0, 0.0, 1.2, 0.4)
        aligned = velocity_aligned_state(point, previous_yaw=-0.7)
        self.assertAlmostEqual(aligned.yaw, -0.7)
        self.assertAlmostEqual(aligned.yawspeed, 0.0)


if __name__ == '__main__':
    unittest.main()
