"""Generate a timed, three-dimensional figure-eight for PX4 local NED."""

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path


FIELDS = ('time_s', 'x', 'y', 'z', 'vx', 'vy', 'vz',
          'ax', 'ay', 'az', 'yaw', 'yawspeed')


@dataclass(frozen=True)
class FigureEight:
    start_north: float = 0.0
    start_east: float = 0.0
    start_down: float = 0.0
    start_yaw: float = 0.0
    north_radius: float = 1.0
    east_radius: float = 0.5
    cruise_height: float = 1.0
    height_variation: float = 0.3
    takeoff_duration: float = 4.0
    loop_duration: float = 12.0
    hold_duration: float = 2.0
    laps: int = 1
    yaw_mode: str = 'fixed'

    def validate(self):
        for name, value in vars(self).items():
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        for name in ('north_radius', 'east_radius', 'cruise_height',
                     'takeoff_duration', 'loop_duration'):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive')
        if self.height_variation < 0 or self.hold_duration < 0:
            raise ValueError('height_variation and hold_duration cannot be negative')
        if self.laps < 1:
            raise ValueError('laps must be at least 1')
        if self.yaw_mode not in ('fixed', 'tangent'):
            raise ValueError('yaw_mode must be fixed or tangent')


def ease(u):
    """Quintic 0-to-1 transition and its first two derivatives in u."""
    return (10*u**3 - 15*u**4 + 6*u**5,
            30*u**2 - 60*u**3 + 30*u**4,
            60*u - 180*u**2 + 120*u**3)


def wrap(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


def row_at(t, config):
    """Analytic position, velocity, acceleration, yaw, and yaw rate at time t."""
    config.validate()
    takeoff_end = config.takeoff_duration
    loop_end = takeoff_end + config.loop_duration
    if not 0 <= t <= loop_end + config.hold_duration:
        raise ValueError('time is outside the trajectory')

    x0, y0, z0 = config.start_north, config.start_east, config.start_down
    cruise_z = z0 - config.cruise_height
    tangent_start = math.atan2(2*config.east_radius, config.north_radius)

    if t <= takeoff_end:
        u = t / config.takeoff_duration
        s, ds, dds = ease(u)
        z = z0 - config.cruise_height*s
        vz = -config.cruise_height*ds/config.takeoff_duration
        az = -config.cruise_height*dds/config.takeoff_duration**2
        yaw = config.start_yaw
        yawspeed = 0.0
        if config.yaw_mode == 'tangent':
            delta = wrap(tangent_start - config.start_yaw)
            yaw = wrap(config.start_yaw + delta*s)
            yawspeed = delta*ds/config.takeoff_duration
        return (t, x0, y0, z, 0.0, 0.0, vz, 0.0, 0.0, az,
                wrap(yaw), yawspeed)

    if t <= loop_end:
        u = (t - takeoff_end) / config.loop_duration
        s, ds, dds = ease(u)
        phase_scale = 2*math.pi*config.laps
        theta = phase_scale*s
        theta_dot = phase_scale*ds/config.loop_duration
        theta_ddot = phase_scale*dds/config.loop_duration**2
        a, b, c = config.north_radius, config.east_radius, config.height_variation

        # North/East trace a figure-eight. The two centre crossings have
        # different heights, making it a genuinely three-dimensional curve.
        x = x0 + a*math.sin(theta)
        y = y0 + b*math.sin(2*theta)
        z = cruise_z - c*(1 - math.cos(theta))/2
        d1 = (a*math.cos(theta), 2*b*math.cos(2*theta),
              -c*math.sin(theta)/2)
        d2 = (-a*math.sin(theta), -4*b*math.sin(2*theta),
              -c*math.cos(theta)/2)
        velocity = [part*theta_dot for part in d1]
        acceleration = [d2[i]*theta_dot**2 + d1[i]*theta_ddot
                        for i in range(3)]

        yaw = config.start_yaw
        yawspeed = 0.0
        if config.yaw_mode == 'tangent':
            yaw = math.atan2(d1[1], d1[0])
            yawspeed = ((d1[0]*d2[1] - d1[1]*d2[0]) /
                        (d1[0]**2 + d1[1]**2) * theta_dot)
        return (t, x, y, z, *velocity, *acceleration, wrap(yaw), yawspeed)

    # The CSV ends at a hover; it does not command landing or disarming.
    yaw = tangent_start if config.yaw_mode == 'tangent' else config.start_yaw
    return (t, x0, y0, cruise_z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            wrap(yaw), 0.0)


def sample_times(config, rate_hz):
    """Include each segment boundary exactly once."""
    if not math.isfinite(rate_hz) or rate_hz < 10:
        raise ValueError('rate_hz must be finite and at least 10')
    boundaries = (0.0, config.takeoff_duration,
                  config.takeoff_duration + config.loop_duration,
                  config.takeoff_duration + config.loop_duration + config.hold_duration)
    times = [0.0]
    for start, end in zip(boundaries, boundaries[1:]):
        if end == start:
            continue
        steps = math.ceil((end - start)*rate_hz)
        times.extend(start + (end - start)*index/steps
                     for index in range(1, steps + 1))
    return times


def write_csv(path, config, rate_hz):
    config.validate()
    times = sample_times(config, rate_hz)
    with open(path, 'w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(FIELDS)
        writer.writerows([f'{value:.9f}' for value in row_at(t, config)]
                         for t in times)
    return len(times)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        default=Path(__file__).with_name('figure8_preview.csv'))
    parser.add_argument('--start-north', type=float, default=0.0)
    parser.add_argument('--start-east', type=float, default=0.0)
    parser.add_argument('--start-down', type=float, default=0.0)
    parser.add_argument('--start-yaw', type=float, default=0.0,
                        help='Radians in PX4 local NED')
    parser.add_argument('--north-radius', type=float, default=1.0)
    parser.add_argument('--east-radius', type=float, default=0.5)
    parser.add_argument('--cruise-height', type=float, default=1.0,
                        help='Metres above the starting position')
    parser.add_argument('--height-variation', type=float, default=0.3,
                        help='Extra height at the second centre crossing')
    parser.add_argument('--takeoff-duration', type=float, default=4.0)
    parser.add_argument('--loop-duration', type=float, default=12.0)
    parser.add_argument('--hold-duration', type=float, default=2.0)
    parser.add_argument('--laps', type=int, default=1)
    parser.add_argument('--yaw-mode', choices=('fixed', 'tangent'), default='fixed')
    parser.add_argument('--rate-hz', type=float, default=50.0)
    args = parser.parse_args()
    config = FigureEight(**{name: getattr(args, name) for name in
                            FigureEight.__dataclass_fields__})
    try:
        count = write_csv(args.output, config, args.rate_hz)
    except (OSError, ValueError) as error:
        parser.exit(1, f'Error: {error}\n')
    print(f'Wrote {count} timed setpoints to {args.output}')
    print('Preview only: match the real PX4 start pose and verify limits before flight.')


if __name__ == '__main__':
    main()
