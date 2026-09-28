"""Analytic trajectory shared by the live PX4 node and visualizer.

PX4 uses local NED coordinates: x=north, y=east, and positive z=down.
Edit :class:`TrajectoryConfig` or ``figure_eight_offset`` to change the path.
No CSV sampling is involved.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class DesiredState:
    """Position, velocity, acceleration, yaw, and yaw rate at one instant."""

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


@dataclass(frozen=True)
class TrajectoryConfig:
    """Parameters for a smooth takeoff, figure eight, and final hover."""

    takeoff_height_m: float = 0.5
    takeoff_duration_s: float = 3.0
    flight_duration_s: float = 12.0
    hold_duration_s: float = 2.0
    #north_radius_m: float = 1.0
    #east_radius_m: float = 0.6
    north_radius_m: float = 0.8
    east_radius_m: float = 0.3
    vertical_amplitude_m: float = 0.15

    def validate(self):
        for name, value in vars(self).items():
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        for name in ('takeoff_height_m', 'takeoff_duration_s',
                     'flight_duration_s', 'north_radius_m', 'east_radius_m'):
            if getattr(self, name) <= 0.0:
                raise ValueError(f'{name} must be positive')
        if self.hold_duration_s < 0.0 or self.vertical_amplitude_m < 0.0:
            raise ValueError(
                'hold_duration_s and vertical_amplitude_m cannot be negative')

    @property
    def duration_s(self):
        return (self.takeoff_duration_s + self.flight_duration_s +
                self.hold_duration_s)


def _smoothstep(u):
    """Quintic smoothstep and its first two derivatives with respect to u."""
    return (10*u**3 - 15*u**4 + 6*u**5,
            30*u**2 - 60*u**3 + 30*u**4,
            60*u - 180*u**2 + 120*u**3)


def figure_eight_offset(t, config):
    """Original 3D figure-eight: two vertical arches over a 1:2 Lissajous.

    This is the main function to edit when a different flight path is wanted.
    The time scaling makes position, velocity, and acceleration join the hover
    continuously at both ends.
    """
    u = min(max(t / config.flight_duration_s, 0.0), 1.0)
    q, dq_du, ddq_du2 = _smoothstep(u)
    qdot = dq_du / config.flight_duration_s
    qddot = ddq_du2 / config.flight_duration_s**2
    theta = 2.0 * math.pi * q
    # Horizontal figure eight plus a small arch, all returning to zero.
    a = config.north_radius_m
    b = config.east_radius_m
    h = config.vertical_amplitude_m
    position = (
        a * math.sin(theta),
        0.5 * b * math.sin(2.0 * theta),
        -0.5 * h * (1.0 - math.cos(2.0 * theta)),
    )
    derivative_q = (
        2.0 * math.pi * a * math.cos(theta),
        2.0 * math.pi * b * math.cos(2.0 * theta),
        -2.0 * math.pi * h * math.sin(2.0 * theta),
    )
    derivative_qq = (
        -(2.0 * math.pi)**2 * a * math.sin(theta),
        -8.0 * math.pi**2 * b * math.sin(2.0 * theta),
        -8.0 * math.pi**2 * h * math.cos(2.0 * theta),
    )
    velocity = tuple(value * qdot for value in derivative_q)
    acceleration = tuple(second * qdot**2 + first * qddot
                         for first, second in zip(derivative_q, derivative_qq))
    return position, velocity, acceleration


def lissajous_figure_eight_offset(t, config):
    """Alternative 3D Lissajous: cross the center at a different height.

    x = A sin(theta), y = (B/2) sin(2 theta),
    z = -(H/2) (1 - cos(theta)), with theta = 2 pi smoothstep(t / T).
    The derivatives below are analytic, including the time reparameterization.
    This starts and ends at the cruise hover pose with zero velocity and
    acceleration, just like the original path.
    """
    u = min(max(t / config.flight_duration_s, 0.0), 1.0)
    q, dq_du, ddq_du2 = _smoothstep(u)
    qdot = dq_du / config.flight_duration_s
    qddot = ddq_du2 / config.flight_duration_s**2
    theta = 2.0 * math.pi * q
    a = config.north_radius_m
    b = config.east_radius_m
    h = config.vertical_amplitude_m
    position = (
        a * math.sin(theta),
        0.5 * b * math.sin(2.0 * theta),
        -0.5 * h * (1.0 - math.cos(theta)),
    )
    derivative_q = (
        2.0 * math.pi * a * math.cos(theta),
        2.0 * math.pi * b * math.cos(2.0 * theta),
        -math.pi * h * math.sin(theta),
    )
    derivative_qq = (
        -(2.0 * math.pi)**2 * a * math.sin(theta),
        -8.0 * math.pi**2 * b * math.sin(2.0 * theta),
        -2.0 * math.pi**2 * h * math.cos(theta),
    )
    velocity = tuple(value * qdot for value in derivative_q)
    acceleration = tuple(second * qdot**2 + first * qddot
                         for first, second in zip(derivative_q, derivative_qq))
    return position, velocity, acceleration


def desired_state(t, origin, yaw, config=TrajectoryConfig()):
    """Evaluate the complete trajectory at ``t`` seconds from a NED origin."""
    config.validate()
    if not math.isfinite(t) or t < 0.0:
        raise ValueError('trajectory time must be finite and non-negative')
    if len(origin) != 3 or not all(math.isfinite(value) for value in origin):
        raise ValueError('origin must contain three finite NED coordinates')
    if not math.isfinite(yaw):
        raise ValueError('yaw must be finite')

    t = min(t, config.duration_s)
    if t <= config.takeoff_duration_s:
        u = t / config.takeoff_duration_s
        s, ds_du, dds_du2 = _smoothstep(u)
        height = config.takeoff_height_m
        duration = config.takeoff_duration_s
        return DesiredState(
            origin[0], origin[1], origin[2] - height*s,
            0.0, 0.0, -height*ds_du/duration,
            0.0, 0.0, -height*dds_du2/duration**2,
            yaw, 0.0)

    cruise = (origin[0], origin[1], origin[2] - config.takeoff_height_m)
    flight_t = t - config.takeoff_duration_s
    if flight_t <= config.flight_duration_s:
        # Choose ONE path. Keep the inactive option commented out.
        #path = figure_eight_offset
        path = lissajous_figure_eight_offset
        position, velocity, acceleration = path(flight_t, config)
        return DesiredState(
            *(cruise[i] + position[i] for i in range(3)),
            *velocity, *acceleration, yaw, 0.0)

    return DesiredState(*cruise, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, yaw, 0.0)
