"""Editable, time-dependent trajectory template for PX4 local NED.

Change ``position_at(t, config)`` to define x(t), y(t), z(t) in seconds.
Velocity and acceleration are computed automatically from those equations.
This only writes a preview CSV; it does not publish or arm.
"""

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

from time_derivatives import TimeValue, as_time_value, sin, cos


FIELDS = ('time_s', 'x', 'y', 'z', 'vx', 'vy', 'vz',
          'ax', 'ay', 'az', 'yaw', 'yawspeed')


@dataclass(frozen=True)
class Config:
    # All positions are in metres in the PX4 LOCAL NED frame (z is DOWN).
    start_north: float = 0.0
    start_east: float = 0.0
    start_down: float = 0.0
    start_yaw: float = 0.0  # radians
    cruise_height: float = 1.0  # metres ABOVE the start
    north_radius: float = 1.0
    east_radius: float = 0.5
    height_variation: float = 0.3

    # reach the cruise height and complete the flight segment
    takeoff_duration: float = 2.0
    # duration of the flying time
    flight_duration: float = 6.0
    hold_duration: float = 2.0

    def validate(self):
        for name, value in vars(self).items():
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        for name in ('cruise_height', 'north_radius', 'east_radius',
                     'takeoff_duration', 'flight_duration'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive')
        if self.height_variation < 0 or self.hold_duration < 0:
            raise ValueError('height_variation and hold_duration cannot be negative')


def ease(u):
    """Quintic transition and its first two derivatives with respect to u."""
    return (10*u**3 - 15*u**4 + 6*u**5,
            30*u**2 - 60*u**3 + 30*u**4,
            60*u - 180*u**2 + 120*u**3)


def position_at(t, config):
    """EDIT HERE: return x(t), y(t), z(t), relative to the cruise position.

    t is seconds since the flight segment began. Use sin/cos imported above,
    not math.sin/math.cos, so derivatives are computed automatically.
    Position offsets must be (0, 0, 0) at t=0 and t=flight_duration.
    """
    u = t / config.flight_duration
    q = 10*u**3 - 15*u**4 + 6*u**5  # smooth time scaling
    a, b, h = config.north_radius, config.east_radius, config.height_variation
    w = 2 * math.pi

    x = a * sin(math.pi * q)**2  # Out and back; zero at both ends.
    y = b * sin(math.pi * q)**2  # Out and back; zero at both ends.
    z = 0.0
    return x, y, z


def flight_at(t, config):
    """Calculate position, velocity, acceleration from position_at(t)."""
    if not math.isfinite(t) or not 0 <= t <= config.flight_duration:
        raise ValueError('flight time is outside the flight segment')
    values = tuple(as_time_value(value)
                   for value in position_at(TimeValue.time(t), config))
    if len(values) != 3:
        raise ValueError('position_at must return x, y, z')
    return (tuple(axis.value for axis in values),
            tuple(axis.velocity for axis in values),
            tuple(axis.acceleration for axis in values))


def validate_flight_joins(config):
    """Do not write a CSV that jumps between takeoff, flight, and hover."""
    for t in (0.0, config.flight_duration):
        position, velocity, acceleration = flight_at(t, config)
        if any(not math.isfinite(value) or abs(value) > 1e-6
               for value in (*position, *velocity, *acceleration)):
            raise ValueError(f'flight position/velocity/acceleration at t={t:g} '
                             'must be zero to join takeoff and hover; '
                             'edit position_at(t)')


def row_at(t, config):
    config.validate()
    flight_start = config.takeoff_duration
    flight_end = flight_start + config.flight_duration
    end = flight_end + config.hold_duration
    if not math.isfinite(t) or not 0 <= t <= end:
        raise ValueError('time is outside the trajectory')

    x0, y0, z0 = config.start_north, config.start_east, config.start_down

    # Calculate the cruise height relative to the starting down position.
    # the Px4 actual z position will be cruise_down= start_down - cruise_height
    cruise_down = z0 - config.cruise_height
    yaw = (config.start_yaw + math.pi) % (2 * math.pi) - math.pi
    if t <= flight_start:
        s, ds, dds = ease(t / config.takeoff_duration)
        return (t, x0, y0, z0 - config.cruise_height*s,
                0.0, 0.0, -config.cruise_height*ds/config.takeoff_duration,
                0.0, 0.0, -config.cruise_height*dds/config.takeoff_duration**2,
                yaw, 0.0)

    if t <= flight_end:
        position, velocity, acceleration = flight_at(t - flight_start, config)
        return (t, x0 + position[0], y0 + position[1],
                cruise_down + position[2], *velocity, *acceleration, yaw, 0.0)

    # End in a hover. The ROS follower does not land or disarm automatically.
    return (t, x0, y0, cruise_down, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, yaw, 0.0)


def sample_times(config, rate_hz):
    if not math.isfinite(rate_hz) or rate_hz < 10:
        raise ValueError('rate_hz must be finite and at least 10')
    boundaries = (0.0, config.takeoff_duration,
                  config.takeoff_duration + config.flight_duration,
                  config.takeoff_duration + config.flight_duration + config.hold_duration)
    times = [0.0]
    for start, end in zip(boundaries, boundaries[1:]):
        if end == start:
            continue
        steps = math.ceil((end - start) * rate_hz)
        times.extend(start + (end - start)*i/steps for i in range(1, steps + 1))
    return times


def write_csv(path, config, rate_hz=50.0):
    config.validate()
    validate_flight_joins(config)
    times = sample_times(config, rate_hz)
    with open(path, 'w', newline='', encoding='utf-8') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(FIELDS)
        writer.writerows([f'{value:.9f}' for value in row_at(t, config)]
                         for t in times)
    return len(times)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        default=Path(__file__).with_name('trajectory_preview.csv'))
    for name, field in Config.__dataclass_fields__.items():
        parser.add_argument('--' + name.replace('_', '-'), type=float,
                            default=field.default)
    parser.add_argument('--rate-hz', type=float, default=50.0)
    args = parser.parse_args()
    config = Config(**{name: getattr(args, name) for name in
                      Config.__dataclass_fields__})
    try:
        count = write_csv(args.output, config, args.rate_hz)
    except (OSError, ValueError) as error:
        parser.exit(1, f'Error: {error}\n')
    print(f'Wrote {count} timed setpoints to {args.output}')
    print('Preview only: verify PX4 start pose, flight volume, and limits before flight.')


if __name__ == '__main__':
    main()
