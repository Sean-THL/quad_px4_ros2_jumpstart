"""Preview the commanded CSV trajectory without ROS, PX4, or Gazebo."""

import argparse
import math
import sys
from pathlib import Path

# Reuse the same parser and interpolation as the ROS 2 setpoint publisher.
PACKAGE_ROOT = Path(__file__).resolve().parents[1] / 'colcon_ws' / 'src' / 'trajectory_follower'
sys.path.insert(0, str(PACKAGE_ROOT))

from trajectory_follower.trajectory import load_trajectory, sample  # noqa: E402


def frame_times(duration_s, fps, speed):
    """Return trajectory times for fixed-rate playback, including the last point."""
    if duration_s == 0:
        return [0.0]
    count = math.ceil(duration_s * fps / speed)
    return [min(index * speed / fps, duration_s) for index in range(count + 1)]


def drone_segments(point, arm_length=0.22):
    """Return north/east/up endpoints for the yaw-only quadrotor icon."""
    north, east, up = point.x, point.y, -point.z
    forward = (math.cos(point.yaw), math.sin(point.yaw))
    right = (-forward[1], forward[0])

    def ends(direction, scale):
        return ((north - scale * direction[0], east - scale * direction[1], up),
                (north + scale * direction[0], east + scale * direction[1], up))

    return ends(forward, arm_length), ends(right, arm_length), (
        (north, east, up),
        (north + 1.5 * arm_length * forward[0],
         east + 1.5 * arm_length * forward[1], up))


def set_line(line, segment):
    line.set_data([point[0] for point in segment],
                  [point[1] for point in segment])
    line.set_3d_properties([point[2] for point in segment])


def visualize(csv_path, fps=30, speed=1.0, save_path=None):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    points = load_trajectory(csv_path)
    times = [point.time_s for point in points]
    playback_times = frame_times(times[-1], fps, speed)
    path_points = [sample(points, times, t) for t in playback_times]
    north = [point.x for point in path_points]
    east = [point.y for point in path_points]
    up = [-point.z for point in path_points]

    figure = plt.figure(figsize=(9, 7))
    axes = figure.add_subplot(111, projection='3d')
    axes.plot(north, east, up, color='0.75', linewidth=1.5,
              label='Commanded path')
    axes.scatter([point.x for point in points], [point.y for point in points],
                 [-point.z for point in points], color='tab:orange', s=24,
                 label='CSV setpoints')
    travelled, = axes.plot([], [], [], color='tab:blue', linewidth=2.5,
                           label='Playback')
    forward_arm, = axes.plot([], [], [], color='black', linewidth=4)
    side_arm, = axes.plot([], [], [], color='black', linewidth=4)
    heading, = axes.plot([], [], [], color='tab:red', linewidth=3,
                         label='Nose / yaw')

    for setter, values in ((axes.set_xlim, north), (axes.set_ylim, east),
                           (axes.set_zlim, up)):
        minimum, maximum = min(values), max(values)
        centre = (minimum + maximum) / 2
        half_width = max((maximum - minimum) / 2 + 0.4, 0.7)
        setter(centre - half_width, centre + half_width)
    axes.set_xlabel('North (m)')
    axes.set_ylabel('East (m)')
    axes.set_zlabel('Altitude above local origin (m)')
    axes.set_title('Commanded trajectory preview — not simulated flight')
    axes.set_box_aspect((1, 1, 1))
    axes.legend(loc='upper left')
    time_label = axes.text2D(0.02, 0.02, '', transform=axes.transAxes)

    def update(index):
        point = path_points[index]
        travelled.set_data(north[:index + 1], east[:index + 1])
        travelled.set_3d_properties(up[:index + 1])
        for line, segment in zip((forward_arm, side_arm, heading),
                                 drone_segments(point)):
            set_line(line, segment)
        time_label.set_text(f't = {playback_times[index]:.2f} / {times[-1]:.2f} s')
        return travelled, forward_arm, side_arm, heading, time_label

    animation = FuncAnimation(figure, update, frames=len(path_points),
                              interval=1000 / fps, blit=False, repeat=False)
    if save_path is not None:
        animation.save(save_path, writer=PillowWriter(fps=fps))
        print(f'Saved animation to {save_path}')
    else:
        plt.show()
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='?', type=Path,
                        default=Path(__file__).with_name('example_trajectory.csv'),
                        help='Timed trajectory CSV; defaults to the example in this folder')
    parser.add_argument('--fps', type=int, default=30, help='Animation frames per second')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Playback speed multiplier')
    parser.add_argument('--save', type=Path, metavar='OUTPUT.gif',
                        help='Save a GIF instead of opening a window')
    args = parser.parse_args()
    if not 1 <= args.fps <= 120:
        parser.error('--fps must be between 1 and 120')
    if not math.isfinite(args.speed) or args.speed <= 0:
        parser.error('--speed must be a positive finite number')
    if args.save is not None and args.save.suffix.lower() != '.gif':
        parser.error('--save must end in .gif')
    try:
        visualize(args.csv, fps=args.fps, speed=args.speed, save_path=args.save)
    except (OSError, ValueError) as error:
        parser.exit(1, f'Error: {error}\n')


if __name__ == '__main__':
    main()
