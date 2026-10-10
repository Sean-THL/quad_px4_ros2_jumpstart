"""Analytic trajectories shared by the live PX4 node and visualizer.

PX4 uses local NED coordinates: x=north, y=east, and positive z=down.
Select a path with ``trajectory_type``. No CSV sampling is involved.
"""

import math
from dataclasses import dataclass, replace


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
    """Parameters for a smooth takeoff, selected path, and final hover."""

    trajectory_type: str = 'lissajous_figure_eight'
    yaw_mode: str = 'fixed'
    takeoff_profile: str = 'smooth'
    takeoff_height_m: float = 0.5
    # Absolute local-NED Z target used by the GUI-like prearmed_hover mode.
    # PX4 NED is negative upward, so -0.7 commands 0.7 m above local zero.
    takeoff_target_z_m: float = -0.7
    takeoff_duration_s: float = 3.0
    flight_duration_s: float = 12.0
    hold_duration_s: float = 2.0
    #north_radius_m: float = 1.0
    #east_radius_m: float = 0.6
    north_radius_m: float = 0.8
    east_radius_m: float = 0.3
    vertical_amplitude_m: float = 0.15
    spiral_radius_m: float = 0.6
    spiral_turns: float = 2.0
    spiral_max_height_m: float = 0.0

    # Offset-spiral geometry is expressed relative to the cruise hover in
    # local NED. The tangent boundary is the fixed plane x=x_align; y_center
    # locates the circle centers laterally, and positive max_height rises.
    offset_spiral_initial_radius_m: float = 0.15
    offset_spiral_max_radius_m: float = 0.6
    offset_spiral_max_height_m: float = 0.3
    offset_spiral_turns: float = 2.0
    offset_spiral_alignment_axis: str = 'x'
    offset_spiral_x_align_m: float = -0.3
    offset_spiral_y_center_m: float = 0.0
    offset_spiral_y_align_m: float = -0.3
    offset_spiral_x_center_m: float = 0.0
    offset_spiral_z0_m: float = 0.0
    offset_spiral_entry_duration_s: float = 2.0

    def validate(self):
        allowed = (
            'figure_eight', 'lissajous_figure_eight', 'spiral',
            'double_spiral',
            'offset_spiral')
        if self.trajectory_type not in allowed:
            raise ValueError(
                f'trajectory_type must be one of {allowed}, got '
                f'{self.trajectory_type!r}')
        if self.yaw_mode not in ('fixed', 'velocity'):
            raise ValueError(
                "yaw_mode must be 'fixed' or 'velocity', got "
                f'{self.yaw_mode!r}')
        if self.takeoff_profile not in ('smooth', 'prearmed_hover'):
            raise ValueError(
                "takeoff_profile must be 'smooth' or 'prearmed_hover'")
        if self.offset_spiral_alignment_axis not in ('x', 'y'):
            raise ValueError(
                "offset_spiral_alignment_axis must be 'x' or 'y', got "
                f'{self.offset_spiral_alignment_axis!r}')
        for name, value in vars(self).items():
            if name in ('trajectory_type', 'yaw_mode', 'takeoff_profile',
                        'offset_spiral_alignment_axis'):
                continue
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        for name in ('takeoff_height_m', 'takeoff_duration_s',
                     'flight_duration_s', 'north_radius_m', 'east_radius_m',
                     'spiral_radius_m', 'spiral_turns'):
            if getattr(self, name) <= 0.0:
                raise ValueError(f'{name} must be positive')
        if self.hold_duration_s < 0.0 or self.vertical_amplitude_m < 0.0:
            raise ValueError(
                'hold_duration_s and vertical_amplitude_m cannot be negative')
        if self.spiral_max_height_m < 0.0:
            raise ValueError('spiral_max_height_m cannot be negative')
        if self.trajectory_type == 'offset_spiral':
            for name in ('offset_spiral_initial_radius_m',
                         'offset_spiral_max_radius_m',
                         'offset_spiral_turns',
                         'offset_spiral_entry_duration_s'):
                if getattr(self, name) <= 0.0:
                    raise ValueError(f'{name} must be positive')
            if (self.offset_spiral_max_radius_m <
                    self.offset_spiral_initial_radius_m):
                raise ValueError(
                    'offset_spiral_max_radius_m must be at least '
                    'offset_spiral_initial_radius_m')
            if self.offset_spiral_max_height_m < 0.0:
                raise ValueError(
                    'offset_spiral_max_height_m cannot be negative')
            if self.offset_spiral_entry_duration_s >= self.flight_duration_s:
                raise ValueError(
                    'offset_spiral_entry_duration_s must be shorter than '
                    'flight_duration_s')

    @property
    def duration_s(self):
        return (self.takeoff_duration_s + self.flight_duration_s +
                self.hold_duration_s)


def _smoothstep(u):
    """Quintic smoothstep and its first two derivatives with respect to u."""
    return (10*u**3 - 15*u**4 + 6*u**5,
            30*u**2 - 60*u**3 + 30*u**4,
            60*u - 180*u**2 + 120*u**3)


def velocity_aligned_state(point, previous_yaw,
                           min_horizontal_speed_m_s=1e-3):
    """Face horizontal velocity, holding the prior yaw when nearly stopped."""
    speed_squared = point.vx**2 + point.vy**2
    if speed_squared < min_horizontal_speed_m_s**2:
        return replace(point, yaw=previous_yaw, yawspeed=0.0)
    yaw = math.atan2(point.vy, point.vx)
    # Time derivative of atan2(vy, vx), using horizontal acceleration.
    yawspeed = (point.vx * point.ay - point.vy * point.ax) / speed_squared
    return replace(point, yaw=yaw, yawspeed=yawspeed)


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


def spiral_offset(t, config):
    """Smooth, fixed-altitude outward spiral in the local NED XY plane.

    Radius grows from zero to ``spiral_radius_m`` over ``spiral_turns``.
    Quintic time scaling makes velocity and acceleration zero at both ends,
    allowing continuous joins from takeoff hover to the final endpoint hold.
    """
    u = min(max(t / config.flight_duration_s, 0.0), 1.0)
    q, dq_du, ddq_du2 = _smoothstep(u)
    qdot = dq_du / config.flight_duration_s
    qddot = ddq_du2 / config.flight_duration_s**2
    radius = config.spiral_radius_m
    angular_scale = 2.0 * math.pi * config.spiral_turns
    theta = angular_scale * q

    # Positive height means upward, which is negative z in PX4 local NED.
    position = (
        radius * q * math.cos(theta),
        radius * q * math.sin(theta),
        -config.spiral_max_height_m * q,
    )
    derivative_q = (
        radius * (math.cos(theta) - q * angular_scale * math.sin(theta)),
        radius * (math.sin(theta) + q * angular_scale * math.cos(theta)),
        -config.spiral_max_height_m,
    )
    derivative_qq = (
        radius * (-2.0 * angular_scale * math.sin(theta) -
                  q * angular_scale**2 * math.cos(theta)),
        radius * (2.0 * angular_scale * math.cos(theta) -
                  q * angular_scale**2 * math.sin(theta)),
        0.0,
    )
    velocity = tuple(value * qdot for value in derivative_q)
    acceleration = tuple(second * qdot**2 + first * qddot
                         for first, second in zip(derivative_q, derivative_qq))
    return position, velocity, acceleration


def double_spiral_offset(t, config):
    """Spiral outward/up, then inward/down to the takeoff hover.

    ``spiral_turns`` is the number of turns on each half. Both halves use
    quintic timing, so the turnaround and final hover join with zero velocity
    and acceleration. The path never descends below the takeoff-hover height.
    """
    half_duration = config.flight_duration_s / 2.0
    returning = t > half_duration
    phase_time = t - half_duration if returning else t
    u = min(max(phase_time / half_duration, 0.0), 1.0)
    q, dq_du, ddq_du2 = _smoothstep(u)
    qdot = dq_du / half_duration
    qddot = ddq_du2 / half_duration**2

    angular_scale = 2.0 * math.pi * config.spiral_turns
    if returning:
        radial_fraction = 1.0 - q
        radial_derivative = -1.0
        theta = angular_scale * (1.0 + q)
        height_fraction = 1.0 - q
        height_derivative = -1.0
    else:
        radial_fraction = q
        radial_derivative = 1.0
        theta = angular_scale * q
        height_fraction = q
        height_derivative = 1.0

    radius = config.spiral_radius_m * radial_fraction
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    position = (
        radius * cos_theta,
        radius * sin_theta,
        -config.spiral_max_height_m * height_fraction,
    )
    derivative_q = (
        config.spiral_radius_m * (
            radial_derivative * cos_theta -
            radial_fraction * angular_scale * sin_theta),
        config.spiral_radius_m * (
            radial_derivative * sin_theta +
            radial_fraction * angular_scale * cos_theta),
        -config.spiral_max_height_m * height_derivative,
    )
    derivative_q_q = (
        config.spiral_radius_m * (
            -2.0 * radial_derivative * angular_scale * sin_theta -
            radial_fraction * angular_scale**2 * cos_theta),
        config.spiral_radius_m * (
            2.0 * radial_derivative * angular_scale * cos_theta -
            radial_fraction * angular_scale**2 * sin_theta),
        0.0,
    )
    velocity = tuple(value * qdot for value in derivative_q)
    acceleration = tuple(second * qdot**2 + first * qddot
                         for first, second in zip(derivative_q,
                                                  derivative_q_q))
    return position, velocity, acceleration


def offset_spiral_offset(t, config):
    """Expanding 3D helix tangent to a fixed local-NED x plane.

    A smooth entry moves from cruise hover to the helix start. During the
    helix, its center is x_align+r, so the left edge is always x_align and
    the opposite edge is x_align+2r. Quintic clock scaling preserves this
    geometry while making velocity and acceleration zero at both endpoints.
    """
    entry_duration = config.offset_spiral_entry_duration_s
    helix_duration = config.flight_duration_s - entry_duration
    initial_radius = config.offset_spiral_initial_radius_m
    max_radius = config.offset_spiral_max_radius_m
    x_align = config.offset_spiral_x_align_m
    y_center = config.offset_spiral_y_center_m
    y_align = config.offset_spiral_y_align_m
    x_center = config.offset_spiral_x_center_m
    z0 = config.offset_spiral_z0_m

    if config.offset_spiral_alignment_axis == 'x':
        initial_position = (
            x_align + 2.0 * initial_radius, y_center, z0)
    else:
        initial_position = (
            x_center, y_align + 2.0 * initial_radius, z0)
    if t <= entry_duration:
        u = min(max(t / entry_duration, 0.0), 1.0)
        s, ds_du, dds_du2 = _smoothstep(u)
        position = tuple(value * s for value in initial_position)
        velocity = tuple(value * ds_du / entry_duration
                         for value in initial_position)
        acceleration = tuple(value * dds_du2 / entry_duration**2
                             for value in initial_position)
        return position, velocity, acceleration

    elapsed = min(max(t - entry_duration, 0.0), helix_duration)
    u = elapsed / helix_duration
    q, dq_du, ddq_du2 = _smoothstep(u)
    tau = helix_duration * q
    tau_dot = dq_du
    tau_ddot = ddq_du2 / helix_duration

    # Requested geometry. Rates are derived from the requested maxima so the
    # last sample reaches max_radius and max_height exactly.
    k = (max_radius - initial_radius) / helix_duration
    omega = 2.0 * math.pi * config.offset_spiral_turns / helix_duration
    h = -config.offset_spiral_max_height_m / helix_duration  # NED: up is -z.
    radius = initial_radius + k * tau
    theta = omega * tau
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)

    aligned_position = radius * (1.0 + cos_theta)
    lateral_position = radius * sin_theta
    aligned_derivative = (
        k * (1.0 + cos_theta) - radius * omega * sin_theta)
    lateral_derivative = k * sin_theta + radius * omega * cos_theta
    aligned_second_derivative = (
        -2.0 * k * omega * sin_theta - radius * omega**2 * cos_theta)
    lateral_second_derivative = (
        2.0 * k * omega * cos_theta - radius * omega**2 * sin_theta)
    if config.offset_spiral_alignment_axis == 'x':
        position = (x_align + aligned_position,
                    y_center + lateral_position, z0 + h * tau)
        derivative_tau = (aligned_derivative, lateral_derivative, h)
        derivative_tau_tau = (
            aligned_second_derivative, lateral_second_derivative, 0.0)
    else:
        position = (x_center + lateral_position,
                    y_align + aligned_position, z0 + h * tau)
        derivative_tau = (lateral_derivative, aligned_derivative, h)
        derivative_tau_tau = (
            lateral_second_derivative, aligned_second_derivative, 0.0)
    velocity = tuple(value * tau_dot for value in derivative_tau)
    acceleration = tuple(
        second * tau_dot**2 + first * tau_ddot
        for first, second in zip(derivative_tau, derivative_tau_tau))
    return position, velocity, acceleration


def trajectory_offset(t, config):
    """Evaluate the selected path relative to the cruise hover position."""
    paths = {
        'figure_eight': figure_eight_offset,
        'lissajous_figure_eight': lissajous_figure_eight_offset,
        'spiral': spiral_offset,
        'double_spiral': double_spiral_offset,
        'offset_spiral': offset_spiral_offset,
    }
    return paths[config.trajectory_type](t, config)


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
        if config.takeoff_profile == 'prearmed_hover':
            # Like the RViz GUI, command the hover height before arming and
            # keep it fixed while the vehicle takes off.
            return DesiredState(
                origin[0], origin[1], config.takeoff_target_z_m,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, yaw, 0.0)
        u = t / config.takeoff_duration_s
        s, ds_du, dds_du2 = _smoothstep(u)
        height = config.takeoff_height_m
        duration = config.takeoff_duration_s
        return DesiredState(
            origin[0], origin[1], origin[2] - height*s,
            0.0, 0.0, -height*ds_du/duration,
            0.0, 0.0, -height*dds_du2/duration**2,
            yaw, 0.0)

    cruise_z = (config.takeoff_target_z_m
                if config.takeoff_profile == 'prearmed_hover' else
                origin[2] - config.takeoff_height_m)
    cruise = (origin[0], origin[1], cruise_z)
    flight_t = t - config.takeoff_duration_s
    if flight_t <= config.flight_duration_s:
        position, velocity, acceleration = trajectory_offset(flight_t, config)
        return DesiredState(
            *(cruise[i] + position[i] for i in range(3)),
            *velocity, *acceleration, yaw, 0.0)

    endpoint, _, _ = trajectory_offset(config.flight_duration_s, config)
    final = tuple(cruise[i] + endpoint[i] for i in range(3))
    return DesiredState(*final, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, yaw, 0.0)
