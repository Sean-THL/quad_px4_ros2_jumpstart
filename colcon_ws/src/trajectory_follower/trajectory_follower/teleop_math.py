"""Pure keyboard-command math; PX4 accepts local-NED velocity, not body velocity."""

import math


MOVE_KEYS = frozenset('wasdrfqe')


def command_for_key(key, heading, horizontal_speed, vertical_speed, yaw_rate):
    """Return (north, east, down, clockwise_yaw_rate) for one key.

    W/S are forward/back, A/D are left/right relative to current heading.
    R/F are up/down. Q/E rotate counterclockwise/clockwise viewed from above.
    In PX4 NED, down and clockwise yaw are positive.
    """
    if key not in MOVE_KEYS:
        raise ValueError(f'unsupported motion key: {key!r}')
    forward = horizontal_speed * (int(key == 'w') - int(key == 's'))
    right = horizontal_speed * (int(key == 'd') - int(key == 'a'))
    north = forward * math.cos(heading) - right * math.sin(heading)
    east = forward * math.sin(heading) + right * math.cos(heading)
    down = vertical_speed * (int(key == 'f') - int(key == 'r'))
    yaw = yaw_rate * (int(key == 'e') - int(key == 'q'))
    return north, east, down, yaw
