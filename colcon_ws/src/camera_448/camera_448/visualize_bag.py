"""Render a synchronized RGB + flight-trajectory MP4 from a ROS 2 bag."""

import argparse
import bisect
import csv
import math
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from camera_448.flight_topics import RGB_COMPRESSED_TOPIC, RGB_RAW_TOPIC


VICON_TOPIC = '/vicon/px4_201/px4_201'
PX4_LOCAL_TOPIC = '/px4_201/fmu/out/vehicle_local_position'
PX4_ODOMETRY_TOPIC = '/px4_201/fmu/out/vehicle_odometry'
TRAJECTORY_TOPIC = '/px4_201/fmu/in/trajectory_setpoint'

# The first two are produced by camera_448. The last two keep the converter
# useful for older bags recorded directly from realsense2_camera.
RGB_TOPIC_PREFERENCE = (
    RGB_COMPRESSED_TOPIC,
    RGB_RAW_TOPIC,
    '/camera/camera/color/image_raw/compressed',
    '/camera/camera/color/image_raw',
)

COLORS = {
    'vicon': (40, 170, 40),
    'px4': (220, 120, 30),
    'command': (40, 40, 220),
}


def parse_args():
    parser = argparse.ArgumentParser(
        description='Create a split-screen RGB and XY trajectory video.')
    parser.add_argument('bag', help='ROS 2 bag directory')
    parser.add_argument('-o', '--output', help='Output MP4 path')
    parser.add_argument('--fps', type=float, default=20.0,
                        help='Output frame rate (default: 20)')
    parser.add_argument('--max-frames', type=int, default=0,
                        help='Stop after this many frames; 0 means all')
    parser.add_argument('--absolute', action='store_true',
                        help='Plot absolute coordinates instead of start-relative paths')
    return parser.parse_args()


def open_reader(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    return reader


def topic_types(reader):
    return {item.name: item.type for item in reader.get_all_topics_and_types()}


def deserialize(serialized, type_name):
    return deserialize_message(serialized, get_message(type_name))


def xyz_from_message(topic, msg):
    if topic == VICON_TOPIC:
        p = msg.transform.translation
        return float(p.x), float(p.y), float(p.z)
    if topic == PX4_LOCAL_TOPIC:
        return float(msg.x), float(msg.y), float(msg.z)
    if topic in (PX4_ODOMETRY_TOPIC, TRAJECTORY_TOPIC):
        p = msg.position
        return float(p[0]), float(p[1]), float(p[2])
    raise ValueError(f'Unsupported position topic: {topic}')


def finite_xyz(xyz):
    return all(math.isfinite(value) for value in xyz)


def scan_paths(bag_path):
    reader = open_reader(bag_path)
    types = topic_types(reader)
    available = set(types)
    rgb_topic = next((topic for topic in RGB_TOPIC_PREFERENCE
                      if topic in available), None)
    if rgb_topic is None:
        raise RuntimeError(
            'No supported RGB topic found. Available topics:\n  ' +
            '\n  '.join(sorted(available)))

    wanted = {
        VICON_TOPIC: 'vicon',
        PX4_LOCAL_TOPIC: 'px4',
        PX4_ODOMETRY_TOPIC: 'px4_odometry',
        TRAJECTORY_TOPIC: 'command',
    }
    samples = {name: [] for name in wanted.values()}
    first_timestamp = None
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        if first_timestamp is None:
            first_timestamp = timestamp
        if topic not in wanted:
            continue
        msg = deserialize(serialized, types[topic])
        xyz = xyz_from_message(topic, msg)
        if finite_xyz(xyz):
            samples[wanted[topic]].append((timestamp, *xyz))

    # Prefer VehicleLocalPosition; use VehicleOdometry only as its fallback.
    if not samples['px4']:
        samples['px4'] = samples['px4_odometry']
    del samples['px4_odometry']
    return types, rgb_topic, samples, first_timestamp or 0


def decode_rgb(msg):
    if hasattr(msg, 'format') and hasattr(msg, 'data'):
        encoded = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError('OpenCV could not decode a compressed RGB frame')
        return image

    height, width, step = int(msg.height), int(msg.width), int(msg.step)
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(height, step)
    encoding = msg.encoding.lower()
    if encoding in ('rgb8', 'bgr8'):
        image = rows[:, :width * 3].reshape(height, width, 3)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if encoding == 'rgb8' else image
    if encoding in ('rgba8', 'bgra8'):
        image = rows[:, :width * 4].reshape(height, width, 4)
        code = cv2.COLOR_RGBA2BGR if encoding == 'rgba8' else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(image, code)
    if encoding in ('mono8', '8uc1'):
        image = rows[:, :width]
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    raise RuntimeError(f'Unsupported raw RGB encoding: {msg.encoding}')


def transformed_xy(source, sample):
    _, x, y, _ = sample
    # Vicon is plotted as its workspace XY. PX4 local position and trajectory
    # are NED, so plot East=y horizontally and North=x vertically.
    return (x, y) if source == 'vicon' else (y, x)


def plot_series(samples, absolute):
    plotted = {}
    for source, values in samples.items():
        points = [transformed_xy(source, sample) for sample in values]
        plotted[source] = points

    if absolute:
        return plotted

    if plotted['vicon']:
        vx, vy = plotted['vicon'][0]
        plotted['vicon'] = [(x - vx, y - vy) for x, y in plotted['vicon']]

    # PX4 positions and command setpoints use the same local NED frame.
    px4_origin = plotted['px4'][0] if plotted['px4'] else (0.0, 0.0)
    for source in ('px4', 'command'):
        ox, oy = px4_origin
        plotted[source] = [(x - ox, y - oy) for x, y in plotted[source]]
    return plotted


def plot_bounds(plotted):
    points = [point for values in plotted.values() for point in values]
    if not points:
        return -1.0, 1.0, -1.0, 1.0
    xs, ys = zip(*points)
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    span = max(xmax - xmin, ymax - ymin, 2.0)
    xmid, ymid = (xmin + xmax) / 2.0, (ymin + ymax) / 2.0
    half = span * 0.6
    return xmid - half, xmid + half, ymid - half, ymid + half


def canvas_point(point, bounds, left=54, top=46, size=340):
    xmin, xmax, ymin, ymax = bounds
    x, y = point
    px = int(left + (x - xmin) / (xmax - xmin) * size)
    py = int(top + size - (y - ymin) / (ymax - ymin) * size)
    return px, py


def last_index_at(samples, timestamp):
    ceiling = (timestamp, float('inf'), float('inf'), float('inf'))
    return bisect.bisect_right(samples, ceiling) - 1


def draw_trajectory_panel(samples, plotted, bounds, timestamp, absolute):
    panel = np.full((448, 448, 3), 248, dtype=np.uint8)
    cv2.putText(panel, 'Workspace trajectory (top view)', (14, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.56, (20, 20, 20), 1,
                cv2.LINE_AA)
    cv2.rectangle(panel, (54, 46), (394, 386), (150, 150, 150), 1)

    xmin, xmax, ymin, ymax = bounds
    for fraction in (0.25, 0.5, 0.75):
        x = int(54 + 340 * fraction)
        y = int(46 + 340 * fraction)
        cv2.line(panel, (x, 46), (x, 386), (225, 225, 225), 1)
        cv2.line(panel, (54, y), (394, y), (225, 225, 225), 1)
    cv2.putText(panel, f'E / X [{xmin:.1f}, {xmax:.1f}] m', (55, 406),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (60, 60, 60), 1,
                cv2.LINE_AA)
    cv2.putText(panel, f'N / Y [{ymin:.1f}, {ymax:.1f}] m', (55, 424),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (60, 60, 60), 1,
                cv2.LINE_AA)

    any_visible = False
    labels = {'vicon': 'Vicon', 'px4': 'PX4 estimate', 'command': 'Command'}
    legend_y = 44
    for source in ('vicon', 'px4', 'command'):
        index = last_index_at(samples[source], timestamp)
        if index < 0:
            continue
        any_visible = True
        points = [canvas_point(point, bounds) for point in plotted[source][:index + 1]]
        color = COLORS[source]
        if len(points) > 1:
            cv2.polylines(panel, [np.asarray(points, dtype=np.int32)], False,
                          color, 2, cv2.LINE_AA)
        cv2.circle(panel, points[-1], 5, color, -1, cv2.LINE_AA)
        cv2.line(panel, (286, legend_y), (304, legend_y), color, 3)
        cv2.putText(panel, labels[source], (309, legend_y + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, color, 1, cv2.LINE_AA)
        legend_y += 17

    if not any_visible:
        cv2.putText(panel, 'No flight position samples in this bag', (73, 210),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (40, 40, 190), 1,
                    cv2.LINE_AA)
    mode = 'absolute' if absolute else 'start-relative'
    cv2.putText(panel, mode, (14, 442), cv2.FONT_HERSHEY_SIMPLEX, 0.36,
                (90, 90, 90), 1, cv2.LINE_AA)
    return panel


def write_csv(output_path, samples):
    csv_path = output_path.with_name(output_path.stem + '_paths.csv')
    with csv_path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(('source', 'timestamp_ns', 'x', 'y', 'z', 'frame'))
        for source, values in samples.items():
            frame = 'vicon_workspace' if source == 'vicon' else 'px4_ned'
            for timestamp, x, y, z in values:
                writer.writerow((source, timestamp, x, y, z, frame))
    return csv_path


def render(args):
    bag_path = Path(args.bag).resolve()
    if not bag_path.is_dir():
        raise RuntimeError(f'Bag directory does not exist: {bag_path}')
    output_path = Path(args.output).resolve() if args.output else (
        bag_path.parent / f'{bag_path.name}_visualization.mp4')
    output_path.parent.mkdir(parents=True, exist_ok=True)

    types, rgb_topic, samples, bag_start = scan_paths(bag_path)
    plotted = plot_series(samples, args.absolute)
    bounds = plot_bounds(plotted)
    counts = ', '.join(f'{name}={len(values)}'
                       for name, values in samples.items())
    print(f'RGB source: {rgb_topic}')
    print(f'Flight samples: {counts}')

    reader = open_reader(bag_path)
    writer = cv2.VideoWriter(
        str(output_path), cv2.VideoWriter_fourcc(*'mp4v'), args.fps,
        (896, 448))
    if not writer.isOpened():
        raise RuntimeError(f'Could not open MP4 writer: {output_path}')

    frame_count = 0
    last_frame_time = None
    minimum_period_ns = int(1e9 / args.fps)
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        if topic != rgb_topic:
            continue
        if last_frame_time is not None and timestamp - last_frame_time < minimum_period_ns:
            continue
        msg = deserialize(serialized, types[topic])
        rgb = cv2.resize(decode_rgb(msg), (448, 448), interpolation=cv2.INTER_AREA)
        panel = draw_trajectory_panel(
            samples, plotted, bounds, timestamp, args.absolute)
        elapsed = (timestamp - bag_start) / 1e9
        cv2.rectangle(rgb, (0, 0), (448, 28), (0, 0, 0), -1)
        cv2.putText(rgb, f'RealSense RGB   t={elapsed:.2f}s', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                    cv2.LINE_AA)
        writer.write(np.hstack((rgb, panel)))
        frame_count += 1
        last_frame_time = timestamp
        if args.max_frames and frame_count >= args.max_frames:
            break
    writer.release()

    if frame_count == 0:
        if output_path.exists():
            output_path.unlink()
        raise RuntimeError(f'No decodable frames found on {rgb_topic}')
    csv_path = write_csv(output_path, samples)
    print(f'Wrote {frame_count} frames: {output_path}')
    print(f'Wrote flight data: {csv_path}')


def main():
    args = parse_args()
    if args.fps <= 0:
        raise SystemExit('--fps must be greater than zero')
    try:
        render(args)
    except Exception as error:  # Present concise failures in ros2 run output.
        raise SystemExit(f'ERROR: {error}') from error


if __name__ == '__main__':
    main()
