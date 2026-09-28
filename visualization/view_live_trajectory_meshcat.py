"""Preview the same analytic trajectory as ``follow_live`` in MeshCat."""

import argparse
import math
import sys
from pathlib import Path

import numpy as np

PACKAGE_ROOT = (Path(__file__).resolve().parents[1] / 'colcon_ws' / 'src' /
                'trajectory_follower')
sys.path.insert(0, str(PACKAGE_ROOT))

from trajectory_follower.live_trajectory import (  # noqa: E402
    TrajectoryConfig, desired_state)


def ned_to_neu(point):
    return np.array([point.x, point.y, -point.z])


def sample_path(config, rate_hz=50.0):
    count = math.ceil(config.duration_s * rate_hz)
    times = [min(index / rate_hz, config.duration_s)
             for index in range(count + 1)]
    points = [desired_state(t, (0.0, 0.0, 0.0), 0.0, config)
              for t in times]
    return times, points


def dashed_vertices(vertices, dash_length=0.10, gap_length=0.06):
    """Convert one polyline into endpoint pairs for fixed-length dashes."""
    segments = []
    drawing = True
    remaining = dash_length
    for start, end in zip(vertices.T[:-1], vertices.T[1:]):
        delta = end - start
        length = np.linalg.norm(delta)
        if length <= 1e-12:
            continue
        direction = delta / length
        travelled = 0.0
        while travelled < length:
            step = min(remaining, length - travelled)
            if drawing and step > 1e-12:
                segments.extend((start + direction * travelled,
                                 start + direction * (travelled + step)))
            travelled += step
            remaining -= step
            if remaining <= 1e-12:
                drawing = not drawing
                remaining = dash_length if drawing else gap_length
    return np.column_stack(segments)


def add_drone(vis, geometry, transformations):
    dark = geometry.MeshLambertMaterial(color=0x222222)
    red = geometry.MeshLambertMaterial(color=0xE53935)
    vis['drone']['body'].set_object(geometry.Sphere(0.055), dark)
    vis['drone']['north_arm'].set_object(
        geometry.Box([0.44, 0.025, 0.018]), dark)
    vis['drone']['east_arm'].set_object(
        geometry.Box([0.025, 0.44, 0.018]), dark)
    vis['drone']['nose'].set_object(geometry.Box([0.16, 0.045, 0.025]), red)
    vis['drone']['nose'].set_transform(
        transformations.translation_matrix([0.16, 0.0, 0.0]))


def visualize(config, fps=30, speed=1.0):
    try:
        import meshcat
        import meshcat.animation as animation
        import meshcat.geometry as geometry
        import meshcat.transformations as transformations
    except ImportError as error:
        raise RuntimeError(
            'MeshCat is not installed. Run: python3 -m pip install -r '
            'visualization/requirements.txt') from error

    times, points = sample_path(config)
    vertices = np.column_stack([ned_to_neu(point) for point in points])
    vis = meshcat.Visualizer().open()
    vis['planned_path'].set_object(
        geometry.LineSegments(
            geometry.PointsGeometry(dashed_vertices(vertices)),
            geometry.LineBasicMaterial(color=0xE53935, linewidth=3)))
    add_drone(vis, geometry, transformations)

    frame_count = math.ceil(config.duration_s * fps / speed) + 1
    playback = animation.Animation(default_framerate=fps)
    for frame in range(frame_count):
        t = min(frame * speed / fps, config.duration_s)
        point = desired_state(t, (0.0, 0.0, 0.0), 0.0, config)
        transform = transformations.translation_matrix(ned_to_neu(point))
        transform = transform @ transformations.rotation_matrix(
            point.yaw, [0, 0, 1])
        with playback.at_frame(vis, frame) as frame_vis:
            frame_vis['drone'].set_transform(transform)
    vis.set_animation(playback)
    print(f'MeshCat URL: {vis.url()}')
    print('The red dashed line is the desired path. Use the browser controls '
          'to replay the '
          'animation.')
    try:
        input('Press Enter to close the visualizer...')
    except EOFError:
        # Allows a non-interactive smoke test while keeping normal use open.
        pass


def main():
    defaults = TrajectoryConfig()
    parser = argparse.ArgumentParser(description=__doc__)
    for name in TrajectoryConfig.__dataclass_fields__:
        parser.add_argument('--' + name.replace('_', '-'), type=float,
                            default=getattr(defaults, name))
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--speed', type=float, default=1.0)
    args = parser.parse_args()
    config = TrajectoryConfig(**{
        name: getattr(args, name)
        for name in TrajectoryConfig.__dataclass_fields__})
    try:
        config.validate()
        if not 1 <= args.fps <= 120:
            raise ValueError('fps must be between 1 and 120')
        if not math.isfinite(args.speed) or args.speed <= 0.0:
            raise ValueError('speed must be positive and finite')
        visualize(config, args.fps, args.speed)
    except (RuntimeError, ValueError) as error:
        parser.exit(1, f'Error: {error}\n')


if __name__ == '__main__':
    main()
