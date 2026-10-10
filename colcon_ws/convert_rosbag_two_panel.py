"""Render a flight rosbag to RGB + 3-D trajectory video without ROS replay.

Run inside the ROS 2 container, where rosbag2_py and px4_msgs are installed.
The right panel compares PX4 local position (Vicon-backed) with the PX4
trajectory setpoint; both use the same local NED frame. The complete bag keeps
the raw Vicon transform, depth image, and TF for later analysis.
"""

import argparse
from bisect import bisect_right
import math
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage


RGB_TOPIC = '/camera_448/color/image_raw/compressed'
POSITION_TOPIC = '/px4_201/fmu/out/vehicle_local_position'
SETPOINT_TOPIC = '/px4_201/fmu/in/trajectory_setpoint'
WIDTH, HEIGHT = 1600, 900
LEFT = (40, 120, 720, 720)
RIGHT = (840, 120, 720, 720)
ACTUAL_COLOR = (200, 120, 35)  # BGR blue
TARGET_COLOR = (30, 145, 225)  # BGR orange


def open_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    found = {topic.name for topic in reader.get_all_topics_and_types()}
    needed = {RGB_TOPIC, POSITION_TOPIC, SETPOINT_TOPIC}
    if needed - found:
        raise ValueError(f'Bag is missing {sorted(needed - found)}')
    return reader


def collect_traces(path):
    """Sample position/setpoint traces at <=10 Hz; retain bag timestamps."""
    reader = open_bag(path)
    actual, target = [], []
    first_rgb_ns = last_rgb_ns = None
    last_sample = {POSITION_TOPIC: -10**18, SETPOINT_TOPIC: -10**18}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == RGB_TOPIC:
            if first_rgb_ns is None:
                first_rgb_ns = stamp
            last_rgb_ns = stamp
        elif topic in last_sample and stamp-last_sample[topic] >= 100_000_000:
            if topic == POSITION_TOPIC:
                msg = deserialize_message(data, VehicleLocalPosition)
                valid = msg.xy_valid and msg.z_valid
                xyz = (msg.x, msg.y, msg.z)
                output = actual
            else:
                msg = deserialize_message(data, TrajectorySetpoint)
                valid = True
                xyz = tuple(msg.position)
                output = target
            if valid and all(math.isfinite(v) for v in xyz):
                output.append((stamp, *xyz))
                last_sample[topic] = stamp
    if not first_rgb_ns or not actual or not target:
        raise ValueError('No RGB frames, valid local positions, or setpoints')
    print(f'Collected {len(actual)} actual positions, {len(target)} targets; '
          f'RGB duration {(last_rgb_ns-first_rgb_ns)/1e9:.1f}s', flush=True)
    return actual, target, first_rgb_ns, last_rgb_ns


class TrajectoryPanel:
    def __init__(self, actual, target):
        self.actual = actual
        self.target = target
        self.actual_times = [point[0] for point in actual]
        self.target_times = [point[0] for point in target]
        self.origin = actual[0][1:]
        raw = [self.raw_projection(point) for point in actual + target]
        us, vs = zip(*raw)
        self.mid_u, self.mid_v = (min(us)+max(us))/2, (min(vs)+max(vs))/2
        span_u = max(us)-min(us)
        span_v = max(vs)-min(vs)
        self.scale = min(640/max(span_u, 0.5), 640/max(span_v, 0.5))
        self.actual_pixels = np.array([self.pixel(point) for point in actual],
                                      dtype=np.int32)
        self.target_pixels = np.array([self.pixel(point) for point in target],
                                      dtype=np.int32)

    def raw_projection(self, point):
        _, x, y, z = point
        north = x-self.origin[0]
        east = y-self.origin[1]
        up = self.origin[2]-z  # PX4 NED: negative z is up.
        return 0.87*east-0.43*north, 0.25*east+0.25*north+0.95*up

    def pixel(self, point):
        u, v = self.raw_projection(point)
        x0, y0, side, _ = RIGHT
        return (int(x0+side/2+(u-self.mid_u)*self.scale),
                int(y0+side/2-(v-self.mid_v)*self.scale))

    def draw(self, canvas, stamp):
        x0, y0, side, _ = RIGHT
        cv2.rectangle(canvas, (x0, y0), (x0+side, y0+side),
                      (195, 195, 195), 2)
        # Faint full target shows the planned workspace path; saturated
        # segments show progress at this camera timestamp.
        cv2.polylines(canvas, [self.target_pixels], False,
                      (225, 215, 205), 2, cv2.LINE_AA)
        ai = bisect_right(self.actual_times, stamp)
        ti = bisect_right(self.target_times, stamp)
        if ti >= 2:
            cv2.polylines(canvas, [self.target_pixels[:ti]], False,
                          TARGET_COLOR, 3, cv2.LINE_AA)
        if ai >= 2:
            cv2.polylines(canvas, [self.actual_pixels[:ai]], False,
                          ACTUAL_COLOR, 3, cv2.LINE_AA)
        if ai:
            cv2.circle(canvas, tuple(self.actual_pixels[ai-1]), 8,
                       ACTUAL_COLOR, -1, cv2.LINE_AA)
        if ti:
            cv2.circle(canvas, tuple(self.target_pixels[ti-1]), 8,
                       TARGET_COLOR, -1, cv2.LINE_AA)
        cv2.putText(canvas, '3-D local workspace (isometric)', (x0+16, y0+32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (45, 45, 45), 2)
        cv2.putText(canvas, 'Blue: actual (Vicon-backed PX4)', (x0+10, 868),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.66, ACTUAL_COLOR, 2)
        cv2.putText(canvas, 'Orange: target', (x0+425, 868),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.66, TARGET_COLOR, 2)
        if ai and ti:
            actual_h = -self.actual[ai-1][3]
            target_h = -self.target[ti-1][3]
            cv2.putText(canvas,
                        f'Height actual {actual_h:.2f} m  target {target_h:.2f} m',
                        (x0+16, y0+side-16), cv2.FONT_HERSHEY_SIMPLEX,
                        0.64, (45, 45, 45), 2)


def draw_frame(rgb, panel, stamp, first_rgb_ns, name):
    canvas = np.full((HEIGHT, WIDTH, 3), 250, dtype=np.uint8)
    cv2.putText(canvas, name, (40, 51), cv2.FONT_HERSHEY_SIMPLEX,
                0.9, (30, 30, 30), 2)
    elapsed = (stamp-first_rgb_ns)/1e9
    cv2.putText(canvas, f't = {elapsed:.1f} s', (1310, 51),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (30, 30, 30), 2)
    x0, y0, width, height = LEFT
    ih, iw = rgb.shape[:2]
    scale = min(width/iw, height/ih)
    fitted = cv2.resize(rgb, (round(iw*scale), round(ih*scale)),
                        interpolation=cv2.INTER_AREA)
    px = x0+(width-fitted.shape[1])//2
    py = y0+(height-fitted.shape[0])//2
    canvas[py:py+fitted.shape[0], px:px+fitted.shape[1]] = fitted
    cv2.rectangle(canvas, (x0, y0), (x0+width, y0+height),
                  (195, 195, 195), 2)
    cv2.putText(canvas, 'RealSense RGB', (x0+10, 868),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (45, 45, 45), 2)
    panel.draw(canvas, stamp)
    return canvas


def convert(path, output, fps, max_seconds):
    actual, target, first_rgb_ns, last_rgb_ns = collect_traces(path)
    if max_seconds is not None:
        last_rgb_ns = min(last_rgb_ns,
                          first_rgb_ns+round(max_seconds*1e9))
    panel = TrajectoryPanel(actual, target)
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*'mp4v'),
                             fps, (WIDTH, HEIGHT))
    if not writer.isOpened():
        raise RuntimeError('Could not open MP4 writer')
    reader = open_bag(path)
    step_ns = round(1e9/fps)
    next_stamp = first_rgb_ns
    rgb = None
    frames = 0
    try:
        while reader.has_next():
            topic, data, stamp = reader.read_next()
            if topic != RGB_TOPIC:
                continue
            if stamp > last_rgb_ns:
                break
            if rgb is not None:
                while next_stamp < stamp and next_stamp <= last_rgb_ns:
                    writer.write(draw_frame(rgb, panel, next_stamp,
                                            first_rgb_ns, path.name))
                    next_stamp += step_ns
                    frames += 1
                    if frames % 300 == 0:
                        print(f'Rendered {frames} frames', flush=True)
            msg = deserialize_message(data, CompressedImage)
            decoded = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8),
                                   cv2.IMREAD_COLOR)
            if decoded is not None:
                rgb = decoded
        while rgb is not None and next_stamp <= last_rgb_ns:
            writer.write(draw_frame(rgb, panel, next_stamp,
                                    first_rgb_ns, path.name))
            next_stamp += step_ns
            frames += 1
    finally:
        writer.release()
    if not frames:
        raise ValueError('No RGB video frames were rendered')
    print(f'Wrote {output} ({frames} frames at {fps:g} fps)', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--fps', type=float, default=20.0)
    parser.add_argument('--max-seconds', type=float)
    args = parser.parse_args()
    if not 1 <= args.fps <= 60:
        parser.error('--fps must be between 1 and 60')
    convert(args.bag, args.output, args.fps, args.max_seconds)


if __name__ == '__main__':
    main()
