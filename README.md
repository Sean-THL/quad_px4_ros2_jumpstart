# quad_px4_ros2_jumpstart

ROS 2 workspace for running PX4 with Vicon-based external pose feedback.

This repository is organized as a `colcon` workspace plus launch/config glue to bring up:
- PX4 DDS bridge (`MicroXRCEAgent`)
- MAVLink routing (`mavlink-routerd`)
- Vicon data ingestion (`vicon_bridge` from `ros2-vicon-bridge`)
- Vicon to PX4 visual odometry conversion (`vicon_px4_bridge`)

## 1) Repository Layout

- `colcon_ws/src/all_launch`: Main launch files and config (`px4.launch.py`, `vicon.launch.py`)
- `colcon_ws/src/ros2-vicon-bridge`: ROS 2 package `vicon_bridge` (Vicon SDK client)
- `colcon_ws/src/vicon_px4_bridge`: ROS 2 package that converts Vicon transforms to PX4 `VehicleOdometry`
- `colcon_ws/src/px4_msgs`: PX4 ROS message definitions
- `colcon_ws/src/dasc_ros_common/dasc_msgs`: Custom DASC message definitions
- `colcon_ws/src/dasc_ros_common/dasc_ros_utils`: Utilities (setpoint publishing + helper scripts)
- `colcon_ws/src/ros2-ping`: Ping/pong latency test nodes
- `colcon_ws/src/julia_common_exp`: Julia experiment code (not a ROS package)

## 2) Packages and What They Do

### `all_launch`
Purpose: orchestration package for runtime launch and config.

Contains:
- `launch/px4.launch.py`: starts PX4-related processes + Vicon pipeline
- `launch/vicon.launch.py`: starts Vicon bridge + static frame transform
- `config/config_vicon.yaml`: parameters for `vicon_bridge`
- `config/mavlink-router.conf`: MAVLink router serial/UDP endpoint config

### `vicon_bridge` (folder: `ros2-vicon-bridge`, package name: `vicon_bridge`)
Purpose: connects to Vicon DataStream SDK and publishes Vicon poses to ROS 2.

What it publishes:
- Dynamic TF for subjects/segments
- Per-segment `geometry_msgs/msg/TransformStamped` on:
  - `vicon/<subject>/<segment>`
- Per-segment `geometry_msgs/msg/PoseStamped` on:
  - `vicon/<subject>/<segment>/pose`

Key behavior:
- Connects to Vicon host `host_name` (e.g. `192.168.1.115:801`)
- Supports stream mode `ServerPush` or `ClientPull`
- Converts Vicon translation mm -> m
- Applies latency correction to message timestamps
- Can publish all segments, or one target segment only

In this project, `all_launch/launch/vicon.launch.py` runs executable `vicon_bridge` with parameters from `config_vicon.yaml`.

### `vicon_px4_bridge`
Purpose: takes Vicon transform data and converts it into PX4 external vision odometry input.

What it subscribes to:
- `vicon/<vicon_name>/<vicon_name>` (`geometry_msgs/msg/TransformStamped`)

What it publishes:
- `<px4_name>/fmu/in/vehicle_visual_odometry` (`px4_msgs/msg/VehicleOdometry`)

Key behavior:
- Frame conversion for PX4 NED-style position:
  - `x = vicon_y`
  - `y = vicon_x`
  - `z = -vicon_z`
- Yaw conversion:
  - extracts yaw from Vicon quaternion
  - converts to NED-style yaw (`-yaw + 1.57`)
  - publishes yaw-only quaternion (roll/pitch zeroed)
- Downsamples incoming Vicon callbacks by factor 2
- Uses local ROS clock timestamp in microseconds (time-sync subscriber code exists but is currently commented)

### `px4_msgs`
Purpose: ROS 2 message interfaces mirroring PX4 uORB messages.

Used heavily by:
- `vicon_px4_bridge` (`VehicleOdometry`)
- `dasc_ros_utils` (`TrajectorySetpoint`, `VehicleLocalPosition`, etc.)

### `dasc_msgs`
Purpose: custom DASC interfaces for trajectories and mission/status messaging.

Includes messages like:
- `DITrajectory`, `DITrajectorySimple`, `DIState`, `DIAcc`
- `CandToSp`, `SpToCand`
- mission status messages

### `dasc_ros_utils`
Purpose: helper nodes/scripts for trajectory and setpoint workflows.

Main compiled node:
- `setpointPublisher`:
  - subscribes to `dasc_msgs/DITrajectory`
  - transforms trajectory into PX4 frame via TF2
  - publishes `px4_msgs/TrajectorySetpoint` to `<px4_robot_name>/fmu/in/trajectory_setpoint`

Installed script:
- `publish_lab_poly.py` publishes a lab boundary polygon on `lab_polygon`

Other scripts in repo (not installed by CMake, but useful during development):
- `joystick_goal.py`
- `joystick_traj.py`
- `debug_time.py`

### `ping` (folder: `ros2-ping`, package name: `ping`)
Purpose: measure ROS 2 round-trip timing.

Executables:
- `ping`
- `pong`
- `rt_listener`

Useful for quick network/ROS transport sanity checks.

## 3) Launch Files (Only the Two You Asked For)

### `all_launch/launch/vicon.launch.py`
Starts:
1. `vicon_bridge` node
- package: `vicon_bridge`
- executable: `vicon_bridge`
- params loaded from: `all_launch/config/config_vicon.yaml`

2. Static TF publisher (`tf2_ros/static_transform_publisher`)
- parent frame: `/vicon/world`
- child frame: `/vicon/world/NED`
- transform:
  - translation: `(0, 0, 0)`
  - roll: `pi`
  - pitch: `0`
  - yaw: `pi/2`

Why this matters:
- Vicon is typically ENU-like / lab-world oriented.
- PX4 expects NED conventions.
- This static transform creates a world-to-NED reference frame for downstream consumers.

### `all_launch/launch/px4.launch.py`
Starts:
1. `MicroXRCEAgent` process
- command: `MicroXRCEAgent serial --dev /dev/ttyUSB1 -b 921600`
- role: DDS-XRCE bridge for PX4 <-> ROS 2 topics

2. `mavlink-routerd` process
- command: `mavlink-routerd -c <all_launch/config/mavlink-router.conf>`
- default config routes serial `/dev/ttyUSB0` at `115200` to UDP `192.168.1.138:14550`

3. `vicon_px4_bridge` node
- package: `vicon_px4_bridge`
- executable: `bridge`
- params set in launch:
  - `px4_name = "px4_100"`
  - `vicon_name = "px4_100"`

4. Included launch: `all_launch/launch/vicon.launch.py`
- brings up Vicon input + static world/NED transform

Net effect of `px4.launch.py`:
- One command starts the full PX4 + Vicon external-vision ingestion pipeline.

## 4) What the Two Bridges Do (Direct Answer)

### What `ros2-vicon bridge` does (`vicon_bridge` package)
- Connects to Vicon Tracker/DataStream over network.
- Pulls subject/segment pose data from Vicon SDK.
- Publishes ROS2 transforms/topics (`TransformStamped`, `PoseStamped`, TF tree).
- Handles latency-corrected timestamps and configurable stream mode/rates.

Think of it as: **Vicon network data -> ROS 2 pose/TF data**.

### What `vicon-px4 bridge` does (`vicon_px4_bridge` package)
- Subscribes to a selected Vicon transform topic for one robot.
- Converts that pose into PX4-compatible `VehicleOdometry` conventions.
- Publishes to PX4 input topic `.../vehicle_visual_odometry`.

Think of it as: **ROS 2 Vicon transform -> PX4 external vision odometry message**.

## 5) How to Run (End-to-End)

### Prerequisites
- Ubuntu 22.04 + ROS 2 Humble environment (native), or Docker path below
- Serial access to PX4 autopilot device(s)
- Vicon network reachable from the runtime machine/container
- Git submodules initialized

Initialize submodules:

```bash
git submodule update --init --recursive
```

### Option A: Run with Docker (repo-provided flow)

Build and start container:

```bash
docker compose build
docker compose up -d
docker compose exec ros bash
```

Note: `docker-compose.yaml` sets `ROS_DOMAIN_ID=4`. Use the same domain on any external ROS 2 machine that needs to communicate with this stack.

Inside container:

```bash
cd /home/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run the full stack:

```bash
ros2 launch all_launch px4.launch.py
```

### Option B: Run natively on host

Install dependencies similar to Dockerfile:
- ROS 2 Humble base + dev tools
- `mavlink-routerd`
- `MicroXRCEAgent`
- `libboost-*`, `ros-humble-diagnostic-updater`

Then:

```bash
cd colcon_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch all_launch px4.launch.py
```

## 6) Configuration You Will Almost Certainly Need to Change

### Vicon host
Edit `colcon_ws/src/all_launch/config/config_vicon.yaml`:
- `host_name`: Vicon machine IP + `:801`
- `target_subject_name`/`target_segment_name` if using specific segment mode

### Robot naming consistency
`px4.launch.py` currently hardcodes `robot_name = "px4_100"` for both:
- Vicon topic selection in `vicon_px4_bridge`
- PX4 namespace prefix

This must match:
- Your Vicon subject/segment names
- The PX4 namespace exposed by your XRCE setup

### Serial devices and baud
Current defaults:
- `MicroXRCEAgent`: `/dev/ttyUSB1` @ `921600`
- `mavlink-router.conf`: `/dev/ttyUSB0` @ `115200`

Adjust if your device paths differ.

### MAVLink UDP destination
Edit `colcon_ws/src/all_launch/config/mavlink-router.conf`:
- `Address = 192.168.1.138`
- `Port = 14550`

Set to the ground station/consumer IP you use.

## 7) Basic Validation Checklist

After launching `px4.launch.py`, check:

```bash
ros2 topic list | rg -E 'vicon|vehicle_visual_odometry|timesync|fmu'
```

Verify Vicon topic exists (example):

```bash
ros2 topic echo /vicon/px4_100/px4_100 --once
```

Verify PX4 odometry injection topic exists:

```bash
ros2 topic echo /px4_100/fmu/in/vehicle_visual_odometry --once
```

Optional TF sanity:

```bash
ros2 run tf2_ros tf2_echo /vicon/world /vicon/world/NED
```

## 8) Common Failure Modes

- No Vicon data:
  - wrong `host_name`, network route, firewall, or Vicon stream not running
- Bridge running but no PX4 odometry consumed:
  - `px4_name` namespace mismatch
  - XRCE agent not connected to correct serial device/baud
- No serial device access in container:
  - check device presence and permissions (`/dev/ttyUSB*`)
- Wrong pose orientation in PX4:
  - verify Vicon frame assumptions and static transform (`world -> world/NED`)

## 9) Useful Commands

Run only Vicon side:

```bash
ros2 launch all_launch vicon.launch.py
```

Run only Vicon->PX4 bridge manually:

```bash
ros2 run vicon_px4_bridge bridge --ros-args -p px4_name:=px4_100 -p vicon_name:=px4_100
```

Run ROS ping/pong latency test:

```bash
ros2 run ping pong
# in another terminal
ros2 run ping ping
```
