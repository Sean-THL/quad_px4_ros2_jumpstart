"""Small safety check before requesting PX4 Land mode."""

import math


LAND_POSITION_TOLERANCE_M = 0.25
LAND_SPEED_TOLERANCE_M_S = 0.2
LAND_SETTLE_TIME_S = 1.0


def settled_at_target(position, position_received_at, target, now):
    """Require a fresh, valid PX4 pose and low speed near the final hover."""
    if position is None or now - position_received_at > 0.5:
        return False
    if not (position.xy_valid and position.z_valid and
            position.v_xy_valid and position.v_z_valid):
        return False
    values = (position.x, position.y, position.z,
              position.vx, position.vy, position.vz)
    if not all(math.isfinite(value) for value in values):
        return False
    distance = math.dist(values[:3], (target.x, target.y, target.z))
    speed = math.sqrt(sum(value**2 for value in values[3:]))
    return (distance <= LAND_POSITION_TOLERANCE_M and
            speed <= LAND_SPEED_TOLERANCE_M_S)
