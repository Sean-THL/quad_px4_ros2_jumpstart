# Timed 3D figure-eight trajectory

## Vertical ascent, optional return descent, and PX4 landing

[`generate_vertical_land.py`](generate_vertical_land.py) writes a simple
vertical ascent and hover CSV. Add `--descent-duration` and `--return-height`
to descend to a second, lower hover at the takeoff x/y/yaw. Without those
options, the original ascent-and-hover output is unchanged. All z coordinates
are in PX4 local NED: the first hover is `start_down - height`; the optional
final hover is `start_down - return_height`. `return_height` is measured
relative to the **measured takeoff pose**, not an independently measured floor.
It must be positive and less than `height`. The CSV never commands ground
contact or disarming. Use PX4 Land mode for that final step.

From the repository root, make a **preview only** using placeholder zeros:

```bash
python3 colcon_ws/trajectories/generate_vertical_land.py \
  --height 1.0 --climb-duration 4 --hover-duration 2 \
  --descent-duration 5 --return-height 0.3 --final-hover-duration 2
visualization/.venv/bin/python visualization/view_trajectory.py \
  colcon_ws/trajectories/vertical_land_preview.csv
```

The numbers above are **only a geometry preview**, not a recommended flight
height or ground clearance for a real drone. For a real test, choose a final
hover with verified clearance for the vehicle and floor; do not treat
`return_height=0.1` as automatically safe. The follower will continue holding
the last hover indefinitely with `land_on_complete:=false`. Select PX4 Land
mode using QGroundControl/RC and wait until landed and disarmed before
stopping the ROS process. If `land_on_complete:=true`, the follower instead
requests PX4 Land mode after reaching the final hover and passing its checks.

To make the final hover lower, **decrease `--return-height`**: changing it
from `0.3` to `0.2` moves the final setpoint 0.1 m downward relative to the
takeoff pose. Do not change `--start-down` just to lower the endpoint: that
must remain the *measured* PX4 starting z. The formula is
`final_z = start_down - return_height`. A lower commanded hover is not a
landing and is not necessarily a known distance above the floor.

Before a real test, replace the start pose with the actual values from
`/px4_100/fmu/out/vehicle_local_position` using `--start-north`,
`--start-east`, `--start-down`, and `--start-yaw`; verify that PX4 has a valid
local pose and heading. Save the verified CSV to a new filename. With the
hardware ROS stack running, the follower launch argument is
`trajectory_file:=/home/colcon_ws/trajectories/<verified-file>.csv`.
`land_on_complete:=false` requires a separate manual Land-mode action;
`land_on_complete:=true` requests PX4 Land automatically after the final
hover. It will refuse automatic-landing startup if PX4
does not publish a fresh `vehicle_land_detected` message showing landed.
The follower reports touchdown and disarm separately; no code can guarantee
that the physical landing area is clear or that a flight cannot crash.

## Create your own trajectory

Use [generate_trajectory.py](generate_trajectory.py) as an editable starting
point. Edit only `position_at(t, config)`: `t` is seconds since the flight
segment began. Write x(t), y(t), z(t); velocity and acceleration are
calculated automatically using [time_derivatives.py](time_derivatives.py).
Use its imported `sin` and `cos`, not `math.sin`/`math.cos`, in position
equations. The template also handles takeoff and final hover. Keep position
offset and its first two derivatives at zero at `t=0` and `t=flight_duration`
so that these segments join smoothly; the generator checks this before
writing a CSV. Yaw is fixed at `start_yaw`; change `row_at` if you need
time-varying yaw/yaw rate.

The current equations produce a diagonal out-and-back flight at constant
cruise height:

```python
x = a * sin(math.pi * q)**2
y = b * sin(math.pi * q)**2
z = 0.0
```

Run the generator with `--north-radius 1.5` to move 1.5 m in PX4 local x;
it also moves in y according to `east_radius`. For x-only motion, edit the
equation to `y = 0.0` in `position_at`. The initial takeoff and final hover
are still at the configured PX4 local start position. To ask PX4 to land
after the drone has returned and settled at the final hover, launch the ROS
follower with `land_on_complete:=true`; this is off by default. The landing
request belongs to the ROS follower, not to `position_at`.

An earlier `trajectory_preview.csv` may still exist; regenerate it after
changing the equations or parameters.

From the repository root:

```bash
python3 colcon_ws/trajectories/generate_trajectory.py
visualization/.venv/bin/python visualization/view_trajectory.py \
  colcon_ws/trajectories/trajectory_preview.csv
```

For example, add `--start-north 0.2 --start-east -0.1 --start-yaw 0.5` and
`--flight-duration 20` to the generator command. It writes the same 12-column
CSV format read by `follow_csv.py`; no ROS or Docker is required for
generation/preview. Regenerate the CSV and restart the follower after an edit:
the follower reads the file once at launch. The template is a starting point,
not a verified flight plan. Do not use it on hardware without checking PX4
local NED alignment, actual start pose/yaw, speed/acceleration, safe flight
volume, estimator state, and how you will land or stop.

## Existing figure-eight generator

`generate_figure8.py` creates a CSV for the existing trajectory viewer and
ROS 2 follower. It needs only Python 3 and runs on the Linux host, **not** in
Docker. The output lives in `colcon_ws/trajectories`, which is also mounted in
the hardware Docker container at `/home/colcon_ws/trajectories`.

From `quad_px4_ros2_jumpstart`:

```bash
python3 colcon_ws/trajectories/generate_figure8.py
visualization/.venv/bin/python visualization/view_trajectory.py \
  colcon_ws/trajectories/figure8_preview.csv
```

The defaults are only for preview: 1 m north radius, 0.5 m east radius, a
smooth 4 s ascent to 1 m above the starting position, a 12 s figure-eight,
and a 2 s hover. The second centre crossing is 0.3 m higher than the first.
Yaw is fixed by default; `--yaw-mode tangent` rotates to follow the horizontal
path direction. Start position and yaw default to zero. Change parameters,
for example:

```bash
python3 colcon_ws/trajectories/generate_figure8.py \
  --output colcon_ws/trajectories/figure8_preview.csv \
  --start-north 0.4 --start-east -0.2 --start-down 0.1 --start-yaw 0.0 \
  --north-radius 0.8 --east-radius 0.5 \
  --cruise-height 0.7 --height-variation 0.2 \
  --takeoff-duration 5 --loop-duration 16 --yaw-mode tangent
```

The output contains time, position, velocity, acceleration, yaw, and yaw rate
at 50 Hz in **PX4 local NED**. It starts at the configured PX4 position,
ascends, flies a full spatial figure-eight, and ends at a hover. The follower
loads the full CSV and streams/interpolates it. This is not a physics
simulation, and the file is **not ready for real flight** until you verify the
actual PX4 local start pose/heading, coordinate frame, reachable flight
volume, speed and acceleration limits, estimator health, and landing plan.
The follower holds the last setpoint by default; `land_on_complete:=true`
requests PX4 Land mode after its final-hover checks pass.

To change the shape or derivatives, edit `generate_figure8.py` (`row_at`).
To change ROS publish behavior, edit
`colcon_ws/src/trajectory_follower/trajectory_follower/follow_csv.py`.
