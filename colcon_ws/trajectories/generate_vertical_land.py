"""Generate a vertical climb, optional return descent, and final hover.

With --descent-duration and --return-height, descend to a hover above the
takeoff pose. This is not a landing: PX4 Land mode handles touchdown and
disarming. The CSV never commands the takeoff/ground height.
"""

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path


FIELDS = ('time_s', 'x', 'y', 'z', 'vx', 'vy', 'vz',
          'ax', 'ay', 'az', 'yaw', 'yawspeed')


@dataclass(frozen=True)
class Config:
    # Read these four numbers from PX4's vehicle_local_position before a real
    # flight. They are in PX4 local NED, NOT raw Vicon coordinates.
    start_north: float = 0.0
    start_east: float = 0.0
    start_down: float = 0.0
    start_yaw: float = 0.0

    # Rise this far ABOVE the measured takeoff pose; bigger = higher first hover.
    height: float = 1.0
    # Longer durations make each climb/descent gentler for the same distance.
    climb_duration: float = 4.0
    # Time spent hovering at the top before returning.
    hover_duration: float = 2.0
    # Set > 0 to add a smooth descent; 0 keeps the original climb-only CSV.
    descent_duration: float = 0.1
    # Final hover ABOVE the measured takeoff pose. Smaller = lower.
    # Must stay strictly between 0 and height when descent is enabled.
    # This is NOT an absolute PX4 z coordinate or guaranteed floor clearance.
    return_height: float = -0.1
    # Time spent at the low hover. The ROS follower holds it after CSV ends.
    final_hover_duration: float = 2.0

    def validate(self):
        for name, value in vars(self).items():
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        if self.height <= 0 or self.climb_duration <= 0:
            raise ValueError('height and climb_duration must be positive')
        if self.hover_duration < 0 or self.final_hover_duration < 0:
            raise ValueError('hover durations cannot be negative')
        if self.descent_duration < 0:
            raise ValueError('descent_duration cannot be negative')
        if self.descent_duration:
            if not 0 < self.return_height < self.height:
                raise ValueError('return_height must be above zero and below height')
        elif self.return_height:
            raise ValueError('return_height requires a positive descent_duration')


def ease(u):
    """Smooth 0-to-1 motion with zero velocity/acceleration at each end."""
    return (10*u**3 - 15*u**4 + 6*u**5,
            30*u**2 - 60*u**3 + 30*u**4,
            60*u - 180*u**2 + 120*u**3)


def row_at(t, config):
    """Return a timed PX4-local-NED setpoint for climb, return, or hover."""
    config.validate()
    descent_start = config.climb_duration + config.hover_duration
    descent_end = descent_start + config.descent_duration
    end = descent_end + (config.final_hover_duration if config.descent_duration else 0.0)
    if not math.isfinite(t) or not 0 <= t <= end:
        raise ValueError('time is outside the trajectory')
    yaw = (config.start_yaw + math.pi) % (2*math.pi) - math.pi
    if t <= config.climb_duration:
        s, ds, dds = ease(t / config.climb_duration)
        z = config.start_down - config.height*s  # NED: negative is up
        vz = -config.height*ds/config.climb_duration
        az = -config.height*dds/config.climb_duration**2
    elif t <= descent_start or not config.descent_duration:
        z = config.start_down - config.height
        vz = az = 0.0
    elif t <= descent_end:
        s, ds, dds = ease((t - descent_start) / config.descent_duration)
        distance = config.height - config.return_height
        # PX4 NED z grows DOWN. Returning lower therefore increases z.
        # Example: start_down=0, height=1, return_height=0.3 -> final z=-0.3.
        z = config.start_down - config.height + distance*s
        vz = distance*ds/config.descent_duration
        az = distance*dds/config.descent_duration**2
    else:
        z = config.start_down - config.return_height
        vz = az = 0.0
    return (t, config.start_north, config.start_east, z,
            0.0, 0.0, vz, 0.0, 0.0, az, yaw, 0.0)


def sample_times(config, rate_hz):
    if not math.isfinite(rate_hz) or rate_hz < 10:
        raise ValueError('rate_hz must be finite and at least 10')
    boundaries = (0.0, config.climb_duration,
                  config.climb_duration + config.hover_duration)
    if config.descent_duration:
        boundaries += (boundaries[-1] + config.descent_duration,
                       boundaries[-1] + config.descent_duration + config.final_hover_duration)
    times = [0.0]
    for start, end in zip(boundaries, boundaries[1:]):
        if end == start:
            continue
        steps = math.ceil((end - start)*rate_hz)
        times.extend(start + (end - start)*i/steps for i in range(1, steps + 1))
    return times


def write_csv(path, config, rate_hz=50.0):
    config.validate()
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
                        default=Path(__file__).with_name('vertical_land_preview.csv'))
    help_text = {
        'start_north': 'Takeoff x from PX4 local position, metres (NED).',
        'start_east': 'Takeoff y from PX4 local position, metres (NED).',
        'start_down': 'Takeoff z from PX4 local position, metres (positive down).',
        'start_yaw': 'Takeoff heading from PX4 local position, radians.',
        'height': 'Initial hover height ABOVE takeoff pose, metres.',
        'climb_duration': 'Climb time in seconds; longer is gentler.',
        'hover_duration': 'Seconds to hover before optional descent.',
        'descent_duration': 'Seconds to return to low hover; 0 disables return.',
        'return_height': 'Final hover height ABOVE takeoff pose, metres; smaller is lower, but must be > 0 and < height.',
        'final_hover_duration': 'Seconds to hover after descent before CSV ends.',
    }
    for name, field in Config.__dataclass_fields__.items():
        parser.add_argument('--' + name.replace('_', '-'),
                            type=float, default=field.default,
                            help=help_text[name] + f' (default: {field.default})')
    parser.add_argument('--rate-hz', type=float, default=50.0)
    args = parser.parse_args()
    config = Config(**{name: getattr(args, name)
                      for name in Config.__dataclass_fields__})
    try:
        count = write_csv(args.output, config, args.rate_hz)
    except (OSError, ValueError) as error:
        parser.exit(1, f'Error: {error}\n')
    print(f'Wrote {count} climb/hover'
          f'{"/return/hover" if config.descent_duration else ""} setpoints to {args.output}')
    print('The CSV does NOT land or disarm; request PX4 Land mode separately.')


if __name__ == '__main__':
    main()
