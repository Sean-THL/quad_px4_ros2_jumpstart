import math
import unittest
from types import SimpleNamespace

from trajectory_follower.landing import settled_at_target
from trajectory_follower.trajectory import Setpoint


class LandingReadinessTest(unittest.TestCase):
    def setUp(self):
        self.target = Setpoint(2.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0)
        self.position = SimpleNamespace(x=0.1, y=0.0, z=-1.0,
                                        vx=0.0, vy=0.0, vz=0.0,
                                        xy_valid=True, z_valid=True,
                                        v_xy_valid=True, v_z_valid=True)

    def test_fresh_stationary_position_is_ready(self):
        self.assertTrue(settled_at_target(self.position, 9.9, self.target, 10.0))

    def test_stale_or_invalid_estimate_is_not_ready(self):
        self.assertFalse(settled_at_target(self.position, 9.0, self.target, 10.0))
        self.position.v_z_valid = False
        self.assertFalse(settled_at_target(self.position, 9.9, self.target, 10.0))
        self.position.v_z_valid = True
        self.position.x = math.nan
        self.assertFalse(settled_at_target(self.position, 9.9, self.target, 10.0))

    def test_moving_or_far_from_hover_is_not_ready(self):
        self.position.vx = 0.3
        self.assertFalse(settled_at_target(self.position, 9.9, self.target, 10.0))
        self.position.vx = 0.0
        self.position.x = 0.4
        self.assertFalse(settled_at_target(self.position, 9.9, self.target, 10.0))


if __name__ == '__main__':
    unittest.main()
