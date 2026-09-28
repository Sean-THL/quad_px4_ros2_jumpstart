"""Load and sample a timed trajectory in PX4's local NED frame."""

import csv
import math
from bisect import bisect_right
from dataclasses import dataclass


FIELDS = ('time_s', 'x', 'y', 'z', 'vx', 'vy', 'vz',
          'ax', 'ay', 'az', 'yaw', 'yawspeed')


@dataclass(frozen=True)
class Setpoint:
    time_s: float
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    ax: float
    ay: float
    az: float
    yaw: float
    yawspeed: float


def load_trajectory(path):
    """Return a list of setpoints with finite values and increasing times."""
    points = []
    with open(path, newline='', encoding='utf-8') as csv_file:
        reader = csv.DictReader(csv_file, skipinitialspace=True)
        if reader.fieldnames is None or tuple(reader.fieldnames) != FIELDS:
            raise ValueError('CSV header must be: ' + ','.join(FIELDS))
        for line_number, row in enumerate(reader, start=2):
            if None in row:
                raise ValueError(f'Extra CSV column on line {line_number}')
            try:
                values = [float(row[name]) for name in FIELDS]
            except (TypeError, ValueError, KeyError) as error:
                raise ValueError(f'Invalid number on CSV line {line_number}') from error
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f'Non-finite value on CSV line {line_number}')
            point = Setpoint(*values)
            if points and point.time_s <= points[-1].time_s:
                raise ValueError(f'Times must increase on CSV line {line_number}')
            points.append(point)
    if not points or points[0].time_s != 0.0:
        raise ValueError('CSV needs at least one row and must start at time_s=0')
    return points


def sample(points, times, elapsed_s):
    """Interpolate position/velocity/acceleration consistently between rows."""
    if elapsed_s <= 0.0:
        return points[0]
    if elapsed_s >= times[-1]:
        return points[-1]
    right = bisect_right(times, elapsed_s)
    before, after = points[right - 1], points[right]
    duration = after.time_s - before.time_s
    s = (elapsed_s - before.time_s) / duration

    def axis(position, velocity, acceleration):
        p0, p1 = getattr(before, position), getattr(after, position)
        v0, v1 = getattr(before, velocity), getattr(after, velocity)
        a0, a1 = getattr(before, acceleration), getattr(after, acceleration)
        scaled_v0 = v0 * duration
        scaled_a0 = a0 * duration * duration
        dp = p1 - p0 - scaled_v0 - 0.5 * scaled_a0
        dv = (v1 - v0) * duration - scaled_a0
        da = (a1 - a0) * duration * duration
        c3 = 10.0 * dp - 4.0 * dv + 0.5 * da
        c4 = -15.0 * dp + 7.0 * dv - da
        c5 = 6.0 * dp - 3.0 * dv + 0.5 * da
        p = p0 + scaled_v0 * s + 0.5 * scaled_a0 * s**2 + c3 * s**3 + c4 * s**4 + c5 * s**5
        v = (scaled_v0 + scaled_a0 * s + 3.0 * c3 * s**2 +
             4.0 * c4 * s**3 + 5.0 * c5 * s**4) / duration
        a = (scaled_a0 + 6.0 * c3 * s + 12.0 * c4 * s**2 +
             20.0 * c5 * s**3) / (duration * duration)
        return p, v, a

    x, vx, ax = axis('x', 'vx', 'ax')
    y, vy, ay = axis('y', 'vy', 'ay')
    z, vz, az = axis('z', 'vz', 'az')

    # Cubic Hermite interpolation keeps yaw and yaw rate continuous.
    yaw_delta = (after.yaw - before.yaw + math.pi) % (2.0 * math.pi) - math.pi
    scaled_yawspeed = before.yawspeed * duration
    dp = yaw_delta - scaled_yawspeed
    dv = (after.yawspeed - before.yawspeed) * duration
    c2 = 3.0 * dp - dv
    c3 = dv - 2.0 * dp
    yaw = before.yaw + scaled_yawspeed * s + c2 * s**2 + c3 * s**3
    yaw = (yaw + math.pi) % (2.0 * math.pi) - math.pi
    yawspeed = (scaled_yawspeed + 2.0 * c2 * s + 3.0 * c3 * s**2) / duration
    return Setpoint(elapsed_s, x, y, z, vx, vy, vz, ax, ay, az, yaw, yawspeed)
