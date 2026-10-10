# quad_px4_ros2_jumpstart

ROS 2 Humble workspace for a PX4 quadrotor using Vicon external-vision pose,
live Offboard trajectories, and synchronized RealSense RGB-D flight recording.
The `all_launch`, `trajectory_follower`, and `camera_448` code here was synced
from the Orin3 workspace. The standalone `PX4-Autopilot` checkout, flashed
flight-controller firmware, flight bags, and camera test media are **not** in
this repository. This ROS code does not flash PX4.

> **Flight safety:** The commands below can arm and move a real vehicle.
> Preview the path, verify frames and limits, clear the flight area, and keep
> a tested pilot/GUI takeover and landing procedure ready. The live follower
> does **not** land automatically. Run only one Offboard setpoint publisher.

## Repository map

| Path | Purpose |
| --- | --- |
| `colcon_ws/src/all_launch` | XRCE agent, MAVLink router, Vicon bridge, and Vicon-to-PX4 odometry launch/config |
| `colcon_ws/src/trajectory_follower` | Analytic live trajectories, CSV follower, and tests |
| `colcon_ws/src/camera_448` | RealSense D455 448×448 RGB-D crop, combined recorder, bag validator, and two-panel converter |
| `visualization/view_live_trajectory_meshcat.py` | Hardware-free preview using the live follower's analytic trajectory function |
| `visualization/` | Additional CSV and bag visualization tools |

Detailed options: [trajectory follower](colcon_ws/src/trajectory_follower/README.md),
[camera/recording](colcon_ws/src/camera_448/README.md), and
[visualization](visualization/README.md).

## Setup

The Orin3 workflow runs in a ROS 2 Humble container with the workspace at
`/home/colcon_ws`. On a fresh checkout, initialize the ROS submodules; some
use SSH URLs and require access to their repositories:

```bash
git submodule update --init --recursive \
  colcon_ws/src/px4_msgs \
  colcon_ws/src/ros2-vicon-bridge \
  colcon_ws/src/vicon_px4_bridge \
  colcon_ws/src/ros2-ping \
  colcon_ws/src/dasc_ros_common
```

The repository's `docker-compose.yaml` supplies an idle ROS 2 container with
`ROS_DOMAIN_ID=4`. The existing Orin3 container may already have camera
dependencies installed. The base `Dockerfile.python` does **not** install the
RealSense ROS driver or all `camera_448` runtime dependencies. For a fresh
container, install the dependencies in
[`camera_448/package.xml`](colcon_ws/src/camera_448/package.xml), including
`realsense2_camera`, `cv_bridge`, OpenCV, and rosbag2. Ensure the container
can see the camera and both PX4 serial devices.

Inside the prepared ROS container:

```bash
cd /home/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  all_launch trajectory_follower camera_448
source install/setup.bash
```

Before connecting hardware, check the site-specific settings:

- [`px4.launch.py`](colcon_ws/src/all_launch/launch/px4.launch.py) uses
  `px4_201` and XRCE serial `/dev/ttyUSB1` at 921600 baud. `px4_201` must
  match the Vicon subject/segment and PX4 namespace.
- [`config_vicon.yaml`](colcon_ws/src/all_launch/config/config_vicon.yaml)
  contains the Vicon host and segment settings.
- [`mavlink-router.conf`](colcon_ws/src/all_launch/config/mavlink-router.conf)
  reads `/dev/ttyUSB0` at 115200 baud and sends telemetry to
  `192.168.0.12:14550`; adjust this for the actual ground station.
- Verify the PX4 system ID independently. `allow_unknown_system_id:=true`
  only permits an unavailable `VehicleStatus.system_id=0`; it does **not**
  prove that the vehicle is 201.

## Preview a path before flight

From the repository root on a machine with a browser:

```bash
python3 -m venv visualization/.venv
visualization/.venv/bin/python -m pip install -r visualization/requirements.txt
visualization/.venv/bin/python visualization/view_live_trajectory_meshcat.py \
  --trajectory-type double_spiral --yaw-mode velocity \
  --takeoff-profile prearmed_hover --takeoff-target-z-m -0.7 \
  --flight-duration-s 30.0 --hold-duration-s 2.0 \
  --spiral-radius-m 0.30 --spiral-turns 1.0 \
  --spiral-max-height-m 0.30
```

The preview shows **commanded** local-NED setpoints, not dynamics, obstacle
clearance, or estimator behavior. Match flight parameters in the preview and
ROS launch. `double_spiral` returns to its takeoff **hover** and holds; it
does not land. `yaw_mode:=velocity` faces horizontal motion and holds the
previous heading when nearly stopped.

## Record a flight on Orin3

The combined launch starts the PX4/Vicon bridge stack, RealSense camera,
448×448 crop, and a timestamped rosbag. It records JPEG RGB and lossless PNG
`compressedDepth` by default:

```bash
source /opt/ros/humble/setup.bash
source /home/colcon_ws/install/setup.bash
ros2 launch camera_448 flight_record.launch.py
```

If `ros2 launch all_launch px4.launch.py` is already running, avoid duplicate
serial/Vicon processes:

```bash
ros2 launch camera_448 flight_record.launch.py start_px4_stack:=false
```

To record raw and compressed formats together, add
`record_raw_rgb:=true record_raw_depth:=true`. The four independent flags are
`record_raw_rgb`, `record_compressed_rgb`, `record_raw_depth`, and
`record_compressed_depth`; at least one RGB and one depth format must remain
enabled. The recorder also selects camera calibration, raw Vicon pose, PX4
local position/odometry, trajectory setpoints, TF, and status/failsafe topics.
The topic list is in
[`flight_topics.py`](colcon_ws/src/camera_448/camera_448/flight_topics.py).
Some selected topics can have zero messages if their publisher is absent;
validate the bag afterward.

Bags go to `/home/colcon_ws/camera/flight_rgbd_YYYYMMDD_HHMMSS` by default.
Stop recording with `Ctrl-C` **after landing** so rosbag writes its metadata.

## Start the live trajectory

Launch in a **second terminal** in the same ROS container after the
bridge/camera/recorder is running. This example uses the custom
`simple_commander` interface: `commander_set_state` **requires**
`takeoff_profile:=prearmed_hover`. Check that `takeoff_target_z_m` is an
appropriate absolute local-NED height; negative Z is upward.
`takeoff_height_m` applies to the separate `smooth` profile, not this one.

```bash
source /opt/ros/humble/setup.bash
source /home/colcon_ws/install/setup.bash
ros2 launch trajectory_follower follow_live.launch.py \
  px4_namespace:=/px4_201 target_system:=201 \
  allow_unknown_system_id:=true \
  auto_arm_offboard:=true auto_command_interface:=commander_set_state \
  takeoff_profile:=prearmed_hover takeoff_target_z_m:=-0.7 \
  trajectory_type:=double_spiral yaw_mode:=velocity \
  flight_duration_s:=30.0 hold_duration_s:=2.0 \
  spiral_radius_m:=0.30 spiral_turns:=1.0 spiral_max_height_m:=0.30
```

Confirm fresh Vicon-backed PX4 position and status, a safe pre-arm target,
and a clear workspace. Then, from a **third terminal**:

```bash
source /opt/ros/humble/setup.bash
source /home/colcon_ws/install/setup.bash
ros2 service call /start_live_trajectory std_srvs/srv/Trigger '{}'
```

The service response means activation was accepted, **not** that PX4 is
already armed or flying. Watch the follower log and vehicle status for
confirmed Arm, Offboard, hover settling, and path start. The node prestreams
a hold setpoint before requesting Arm/Offboard. It holds at the path endpoint;
the pilot/GUI must land and disarm. Do not run another setpoint publisher
alongside it. See the [follower guide](colcon_ws/src/trajectory_follower/README.md)
for standard `vehicle_command`, manual GUI mode, other paths, and parameters.

## Check and visualize the bag

After the recorder stops and the bag directory exists:

```bash
ros2 run camera_448 validate_flight_bag \
  /home/colcon_ws/camera/flight_rgbd_YYYYMMDD_HHMMSS
ros2 run camera_448 visualize_flight_bag \
  /home/colcon_ws/camera/flight_rgbd_YYYYMMDD_HHMMSS
```

The converter writes an MP4 with RealSense RGB on the left and
Vicon/PX4 actual versus commanded trajectory on the right, plus a path CSV.
It prefers compressed RGB if both formats were recorded. Depth and TF remain
in the bag for later analysis; they are not shown in the RGB panel.

For troubleshooting, check `ros2 topic list`, serial device names, Vicon
reachability, ROS domain ID, PX4 namespace/system ID, and the bag validator's
missing-topic report before attempting another flight.
