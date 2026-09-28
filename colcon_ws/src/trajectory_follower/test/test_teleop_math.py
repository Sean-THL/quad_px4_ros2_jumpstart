import math
import unittest

from trajectory_follower.teleop_math import command_for_key


class TeleopMathTest(unittest.TestCase):
    def test_forward_at_zero_heading_is_north(self):
        self.assertEqual(command_for_key('w', 0, 0.3, 0.2, 0.4),
                         (0.3, 0.0, 0.0, 0.0))

    def test_body_motion_rotates_with_heading(self):
        forward = command_for_key('w', math.pi / 2, 0.3, 0.2, 0.4)
        right = command_for_key('d', math.pi / 2, 0.3, 0.2, 0.4)
        self.assertAlmostEqual(forward[0], 0.0)
        self.assertAlmostEqual(forward[1], 0.3)
        self.assertAlmostEqual(right[0], -0.3)
        self.assertAlmostEqual(right[1], 0.0)

    def test_vertical_and_yaw_signs(self):
        self.assertEqual(command_for_key('r', 0, 0.3, 0.2, 0.4)[2], -0.2)
        self.assertEqual(command_for_key('f', 0, 0.3, 0.2, 0.4)[2], 0.2)
        self.assertEqual(command_for_key('q', 0, 0.3, 0.2, 0.4)[3], -0.4)
        self.assertEqual(command_for_key('e', 0, 0.3, 0.2, 0.4)[3], 0.4)

    def test_unknown_key(self):
        with self.assertRaises(ValueError):
            command_for_key(' ', 0, 0.3, 0.2, 0.4)


if __name__ == '__main__':
    unittest.main()
