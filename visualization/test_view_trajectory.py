import math
import unittest

from view_trajectory import drone_segments, frame_times
from trajectory_follower.trajectory import Setpoint


class VisualizationTest(unittest.TestCase):
    def test_frame_times_include_endpoint(self):
        self.assertEqual(frame_times(1.0, fps=2, speed=1.0), [0.0, 0.5, 1.0])
        self.assertEqual(frame_times(0.0, fps=30, speed=1.0), [0.0])

    def test_ned_z_is_drawn_as_altitude_and_yaw_rotates_nose(self):
        point = Setpoint(0.0, 2.0, 3.0, -1.0, 0, 0, 0, 0, 0, 0,
                         math.pi / 2, 0)
        _, _, nose = drone_segments(point)
        self.assertAlmostEqual(nose[0][2], 1.0)
        self.assertAlmostEqual(nose[1][0], 2.0)
        self.assertGreater(nose[1][1], nose[0][1])


if __name__ == '__main__':
    unittest.main()
