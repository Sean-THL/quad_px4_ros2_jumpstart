# camera_448

This package starts the Intel RealSense camera at a native, supported
`640x480x30` profile, aligns depth to color, then center-crops both streams to
`448x448`. It adjusts `CameraInfo` principal points for the crop.

Build and launch from the ROS container:

```bash
cd /home/colcon_ws
colcon build --symlink-install --packages-select camera_448
source install/setup.bash
ros2 launch camera_448 camera_448.launch.py
```

The 448x448 topics are:

```text
/camera_448/color/image_raw                 (raw RGB, optional)
/camera_448/color/image_raw/compressed      (JPEG RGB, quality 90)
/camera_448/color/camera_info
/camera_448/aligned_depth_to_color/image_raw (lossless 16UC1 depth)
/camera_448/aligned_depth_to_color/image_raw/compressedDepth
                                               (lossless 16-bit PNG depth)
/camera_448/aligned_depth_to_color/camera_info
```

Record compressed RGB and lossless depth instead of the native topics:

```bash
cd /home/colcon_ws/camera
ros2 bag record -o realsense_448_$(date +%Y%m%d_%H%M%S) \
  /camera_448/color/image_raw/compressed \
  /camera_448/color/camera_info \
  /camera_448/aligned_depth_to_color/image_raw/compressedDepth \
  /camera_448/aligned_depth_to_color/camera_info \
  /tf \
  /tf_static
```

The original 640x480 RealSense topics remain available under
`/camera/camera/...`; recording those would store 640x480 rather than 448x448.
The raw 448x448 RGB topic also remains available for visualization and tests,
but do not include it in the bag when storage size matters. JPEG is lossy;
change `jpeg_quality` in `camera_448.launch.py` if a different quality/size
tradeoff is needed.

The compressed depth topic is a ROS-compatible `compressedDepth` message. Its
16-bit PNG encoding is lossless, so decompression preserves every depth value.
Change `depth_png_level` from 0 (fastest/largest) to 9 (slowest/smallest); the
default level 3 is intended to avoid excessive CPU use at 30 FPS.

## Complete flight recording

The combined launch starts the PX4/Vicon bridge stack, the compressed 448x448
camera, and one timestamped rosbag recorder:

```bash
source /opt/ros/humble/setup.bash
source /home/colcon_ws/install/setup.bash
ros2 launch camera_448 flight_record.launch.py
```

Compressed-only recording is the default. To record both raw and compressed
RGB and depth in the same bag:

```bash
ros2 launch camera_448 flight_record.launch.py \
  record_raw_rgb:=true \
  record_raw_depth:=true
```

Each representation can be selected independently:

```text
record_raw_rgb:=false
record_compressed_rgb:=true
record_raw_depth:=false
record_compressed_depth:=true
```

At least one RGB and one depth representation must remain enabled. Recording
both preserves maximum flexibility but substantially increases bag size.

Start the trajectory only after the recorder reports that recording has begun.
Stop the launch once with `Ctrl-C` after landing so rosbag can write its
metadata cleanly.

The bag manifest in `camera_448/flight_topics.py` includes the selected RGB and
depth representations, both camera calibrations, raw Vicon ground truth,
PX4 local position/odometry, desired trajectory setpoints, `/tf`, `/tf_static`,
and small PX4 status/failsafe topics. The measured PX4 local position and the
trajectory setpoints share the NED control frame and are the preferred pair for
the trajectory panel; raw Vicon is retained independently as ground truth.

If `all_launch px4.launch.py` is already running in another terminal, avoid
starting a duplicate stack:

```bash
ros2 launch camera_448 flight_record.launch.py start_px4_stack:=false
```

Immediately validate the completed bag (replace the directory name):

```bash
ros2 run camera_448 validate_flight_bag \
  /home/colcon_ws/camera/flight_rgbd_YYYYMMDD_HHMMSS
```

The command fails visibly if compressed RGB, compressed depth, either camera
calibration, raw Vicon pose, PX4 local position, trajectory setpoints, or
`/tf_static` has zero messages. Do this before powering down or leaving the
flight area so a missing publisher can be corrected while the setup is intact.
# Visualize a completed flight bag

Create an MP4 with RealSense RGB on the left and the synchronized top-down
Vicon, PX4-estimate, and commanded paths on the right:

```bash
ros2 run camera_448 visualize_flight_bag BAG_DIRECTORY
```

The converter prefers compressed RGB when both representations were recorded.
It writes `*_visualization.mp4` and `*_visualization_paths.csv` beside the bag.
Use `-o OUTPUT.mp4`, `--fps N`, or `--absolute` to customize the result.

