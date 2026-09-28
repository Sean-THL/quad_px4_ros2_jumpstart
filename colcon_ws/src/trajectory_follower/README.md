# CSV trajectory follower

## Live analytic trajectory (no CSV)

`follow_live` evaluates the desired position, velocity, and acceleration on
every 50 Hz timer tick. The editable path is in
[`live_trajectory.py`](trajectory_follower/live_trajectory.py), especially
`figure_eight_offset()`. Its default sequence is:

1. Smooth takeoff from the current PX4 local pose.
2. One smooth 3D figure eight.
3. Final hover at the takeoff height.

The default horizontal path is already a 1:2 Lissajous figure-eight. An
alternative `lissajous_figure_eight_offset()` crosses the center at a higher
altitude instead of making two arches over the lobes. To choose either path,
comment out one `path = ...` line and uncomment the other in `desired_state()`.
The MeshCat viewer imports this same function, so preview the selected path
before building or flying it.
The launch file also reads its trajectory defaults from `TrajectoryConfig`;
explicit launch arguments still override them.

There is deliberately no landing logic. Use the established GUI/pilot landing
procedure. The node obtains its launch origin and actual state from
`VehicleLocalPosition`. In this repository that is PX4's Vicon-backed state,
after Vicon has passed through `vicon_px4_bridge` and the PX4 estimator. This
is preferable to subtracting the raw Vicon transform because the actual and
desired values then share exactly the local-NED origin used by PX4's position
controller.

The node publishes desired state to `TrajectorySetpoint`; PX4's existing
firmware computes feedback/control effort. It also publishes the monitor-only
error `desired - actual` as `Vector3Stamped` on
`/live_trajectory_follower/tracking_error` and logs its magnitude once per
second. Do not feed that error back into another position correction unless
the PX4 position loop is intentionally being replaced and retuned; doing both
would create two competing feedback loops.

Build and launch it from the ROS container:

```bash
cd /home/colcon_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to trajectory_follower
source install/setup.bash

# Terminal 1: existing PX4 and Vicon bridge
ros2 launch all_launch px4.launch.py

# Terminal 2: live desired-state publisher
ros2 launch trajectory_follower follow_live.launch.py \
  px4_namespace:=/px4_100 target_system:=100

# Terminal 3: explicit start (this requests Offboard and arms)
ros2 service call /start_live_trajectory std_srvs/srv/Trigger '{}'
```

Before the service call, confirm that the Vicon-backed position and vehicle
status are fresh. Only run one Offboard setpoint publisher at a time. Example
parameter changes, which affect both the path and its analytic derivatives:

```bash
ros2 launch trajectory_follower follow_live.launch.py \
  takeoff_height_m:=0.8 takeoff_duration_s:=4.0 \
  flight_duration_s:=16.0 north_radius_m:=1.2 east_radius_m:=0.5 \
  vertical_amplitude_m:=0.1
```

Positive PX4 `z` is down. The path is anchored to the valid pose captured by
`/start_live_trajectory`, so no absolute launch coordinates or CSV rewrite is
needed. The service refuses to start without fresh valid local pose/status,
from an already armed vehicle, or when the reported system ID differs from
`target_system`. Test new dimensions and durations in MeshCat and simulation
before flight; parameter validation is not a collision or dynamics check.

## Keyboard manual teleoperation (separate node)

[`teleop_keyboard.py`](trajectory_follower/teleop_keyboard.py) is an
**alternative** to CSV playback, not an extra publisher to run with it. It
publishes local-NED **velocity** setpoints at 20 Hz; W/A/S/D velocities are
rotated from the current PX4 heading. It uses PX4's velocity controller,
not Eware. Zero velocity after a key expires is **not a position hold** and
the drone may drift. This is experimental code; it has not been flown on
hardware. Keep a tested RC/QGroundControl takeover and landing procedure.

Keys (use **one motion key at a time** in the focused terminal):

| Key | Action |
| --- | --- |
| W / S | Forward / backward relative to drone nose |
| A / D | Strafe left / right relative to drone nose |
| R / F | Climb / descend |
| Q / E | Counterclockwise / clockwise yaw viewed from above |
| Space | Stop commanded motion; sends zero velocity |
| G | Start 1.2 s prestream, then request PX4 Offboard mode |
| L | Request PX4 Land mode (not Ctrl+C) |
| X | Exit, only after PX4 reports disarmed |

Each motion key is a short 0.3 s pulse. Holding the key relies on normal
terminal keyboard repeat. The terminal cannot reliably detect multiple
simultaneous keys or key release. If the terminal loses focus, commands
expire and zero velocity is sent; if the **whole ROS process** stops, PX4's
configured Offboard-loss failsafe takes over instead. Do not treat this
software timeout as a flight-safety mechanism.

The node must see the drone **disarmed at the takeoff spot first**. It saves
the PX4 local z there and blocks deliberate F/down commands within
`min_clearance_m` (default 0.4 m) of that z. This is only a soft setpoint
guard; estimator error, uneven floor, momentum, and vehicle geometry can
still cause a collision. It never arms or takes off: manually take off in a
tested PX4 mode and settle to a stable hover before pressing G. It requires
fresh valid PX4 position, velocity, heading, and status, a matching system
ID, and a takeoff altitude above the soft clearance. Land in PX4 Land mode;
do not use F to fly into the ground.

From the repository root on the Linux host, with PX4/Vicon/QGroundControl
connected and the existing Vicon-name and PX4-ID mismatch resolved:

```bash
docker compose up -d ros
docker compose exec ros bash
source /opt/ros/humble/setup.bash
cd /home/colcon_ws
colcon build --symlink-install --packages-up-to trajectory_follower
source /home/colcon_ws/install/setup.bash
ros2 launch all_launch px4.launch.py
```

Leave that hardware-bridge terminal running. In a **second interactive**
container terminal, while the drone is still disarmed at the launch spot:

```bash
docker compose exec ros bash
source /opt/ros/humble/setup.bash
source /home/colcon_ws/install/setup.bash
ros2 run trajectory_follower teleop_keyboard
```

Wait for the PX4 local-position/status topics to be healthy. Take off and
hover using your established RC/QGroundControl procedure. Press G in the
keyboard terminal and verify PX4 confirms Offboard before using motion keys.
At the end, press L or switch to Land mode in QGroundControl/RC; verify PX4
actually entered Land, then wait for touchdown and disarm before X. Do not
run `follow_csv`, Eware, meSch, or any other Offboard publisher simultaneously.
Change the slow test speeds with ROS parameters, for example:

```bash
ros2 run trajectory_follower teleop_keyboard --ros-args \
  -p horizontal_speed_m_s:=0.20 -p vertical_speed_m_s:=0.10 \
  -p yaw_rate_rad_s:=0.25 -p min_clearance_m:=0.5
```

These numbers are examples, not flight-approved limits. The PX4 local frame
must match the Vicon bridge, and the actual firmware must accept the ROS 2
Offboard topics. Validate in simulation or a controlled test with an
independent pilot before relying on keyboard control.

## CSV playback (the original follower)

The separate `follow_csv` node streams a precomputed multicopter trajectory
to PX4. It does not use Eware or Julia. Its launch file loads a CSV and waits
for an explicit `/start_trajectory` service call. That call starts a 1.2
second Offboard prestream, requests Offboard mode and arming, then starts the
trajectory only after PX4 reports both states. By default it holds the final
position after playback; it does not automatically land or disarm.

To request PX4 Land mode after the CSV ends, launch with `land_on_complete:=true`.
The CSV must end at a safe **hover above the landing spot**, not at ground
height. The follower waits until PX4 reports a fresh position within 0.25 m
of the final setpoint and speed below 0.2 m/s for 1 second. It also requires
fresh `/px4_100/fmu/out/vehicle_land_detected` telemetry: `landed=true` before
takeoff, then `landed=false` while hovering in flight. If that topic is
missing, starting with auto-land is rejected, or the drone remains hovering
if the topic becomes stale later. Then it sends one Land request. Once PX4
reports Land mode, the follower stops Offboard streaming and PX4 handles
descent and touchdown. The follower logs **touchdown reported** when PX4's
land detector sets `landed=true`, and **landed and disarmed** only when PX4
also reports the standby arming state. These are PX4 telemetry reports, not
independent proof that the landing area was safe or that no damage occurred.
If PX4 does not confirm Land
mode within 3 seconds, the follower keeps holding the final hover and logs an
error; land manually or change flight mode in the ground station. If position
or velocity estimates are invalid, it will wait at the final hover instead
of requesting landing. Automatic landing is **off by default** and has not
been tested on a physical drone.

## CSV format

The header must be exactly:

```csv
time_s,x,y,z,vx,vy,vz,ax,ay,az,yaw,yawspeed
```

Times are seconds from trajectory start and must begin at `0` and increase. Position is in metres, velocity in m/s, acceleration in m/s², yaw in radians, and yaw speed in rad/s. All values are in PX4's **local NED frame**: positive `z` is down, so a one metre altitude above the local origin is normally `z=-1`. Yaw is around the down axis. Use the same local origin as `/px4_100/fmu/out/vehicle_local_position`; this node does no frame conversion.

The node interpolates each position, velocity, and acceleration axis together with a quintic polynomial. Yaw and yaw speed use a shortest-angle cubic interpolation. Give the final row zero velocity, acceleration, and yaw speed if you want a smooth hover at the end. Adapt the first row to the actual vehicle position and heading before using it.

To create a timed 3D figure-eight analytically instead of hand-writing CSV rows, use [the figure-eight generator](../../trajectories/README.md). Its output uses this exact CSV format and can be previewed with the Python viewer before any ROS launch.

For a simple ascent/hover, or an ascent followed by a smooth return descent
to a lower hover above the takeoff pose, use
[`generate_vertical_land.py`](../../trajectories/generate_vertical_land.py).
The return descent still does not land or disarm; the final step requires
PX4 Land mode through QGroundControl/RC or `land_on_complete:=true`.

The first CSV position must be within `max_start_distance_m` (default 0.5 m) of PX4's current local position when `/start_trajectory` is called. The first yaw must be within `max_start_yaw_error_rad` (default 0.5 rad) of the current heading. The vehicle must initially be disarmed and have valid local position and heading. The node also checks that `target_system` matches PX4's reported MAVLink system ID.

## Build and run

The workspace dependencies, especially the `px4_msgs` Git submodule, must be present, and the message definitions must match the firmware. For the Python workflow, initialize the ROS submodules listed in the repository-root README; the optional Julia submodule is not needed. Then build from `colcon_ws`:

```bash
cd colcon_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Use Ubuntu's system Python for ROS Humble. If an Anaconda environment is active, deactivate it before building or launching; this machine's `python3` currently resolves to Anaconda Python 3.12, while ROS Humble is installed for system Python 3.10.

In terminal 1, with the flight controller and Vicon system connected:

```bash
ros2 launch all_launch px4.launch.py
```

That launch file starts the XRCE agent, MAVLink router, Vicon bridge, and Vicon-to-PX4 odometry bridge. It does not start the PX4 firmware itself.

In terminal 2, after sourcing the workspace, run your trajectory file (use an absolute path):

```bash
ros2 launch trajectory_follower follow_csv.launch.py \
  trajectory_file:=/absolute/path/to/your_trajectory.csv \
  px4_namespace:=/px4_100 target_system:=100 \
  land_on_complete:=false
```

Check the topics and PX4 state before asking the node to start:

```bash
ros2 topic echo /px4_100/fmu/out/vehicle_local_position --once
ros2 topic echo /px4_100/fmu/out/vehicle_status --once
ros2 topic echo /px4_100/fmu/out/vehicle_land_detected --once
ros2 topic info /px4_100/fmu/in/trajectory_setpoint
```

Then, in terminal 3 with the same ROS environment:

```bash
ros2 service call /start_trajectory std_srvs/srv/Trigger '{}'
```

The service is the point at which the node may arm and fly. Watch the node log and PX4/QGroundControl status. At the end, the node holds the final setpoint unless `land_on_complete:=true` is selected and the landing checks pass. Check that your **actual PX4 firmware** publishes `vehicle_land_detected`: adding its ROS message to this repository does not enable it in PX4 firmware. Do not launch `eware.launch.py`, `meSch.launch.py`, or any other setpoint publisher at the same time.

PX4 must be configured to accept the three inbound DDS topics: `offboard_control_mode`, `trajectory_setpoint`, and `vehicle_command`. The ROS 2 namespace must match the vehicle's DDS namespace. The repo's Vicon settings currently have a `px4_10` target while `px4.launch.py` uses `px4_100`; verify that the actual Vicon subject/segment and PX4 namespace match before flight.
