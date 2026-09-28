# Python-only trajectory visualization

## MeshCat preview for the live trajectory

The no-CSV trajectory has a Python MeshCat viewer that imports the exact same
`desired_state()` function used by the ROS publisher:

```bash
visualization/.venv/bin/python visualization/view_live_trajectory_meshcat.py
```

It opens a browser-based 3D scene, draws the complete desired path as a red
dashed line, and animates a simple quadrotor. It converts local NED to
north/east/up for display only. Match any ROS launch overrides when
previewing, for example:

```bash
visualization/.venv/bin/python visualization/view_live_trajectory_meshcat.py \
  --takeoff-height-m 0.8 --takeoff-duration-s 4 \
  --flight-duration-s 16 --north-radius-m 1.2 --east-radius-m 0.5
```

The viewer is a commanded-path preview, not a dynamics simulation or safety
validation. MeshCat is installed by the requirements command below.

This previews the **commanded path**, not the drone's physical response. It
adapts the path-and-animated-vehicle idea in meSch's `Visualization.jl` to the
single-vehicle CSV format used by this project's `trajectory_follower`. It
does not require Julia, ROS, PX4, Gazebo, Docker, or Vicon. The quadrotor icon
shows **position and yaw only**; the CSV has no roll or pitch, and this viewer
does not calculate them. It uses the *same* CSV parser and quintic/yaw
interpolator as the ROS 2 flight publisher, so both read the same setpoints.

## Set up once

In a terminal, go to `quad_px4_ros2_jumpstart` and run:

```bash
python3 -m venv visualization/.venv
visualization/.venv/bin/python -m pip install -r visualization/requirements.txt
```

If `venv` is unavailable on Ubuntu, install `python3-venv` first. A virtual
environment keeps plotting packages separate from ROS and PX4 dependencies.

## View the example

```bash
visualization/.venv/bin/python visualization/view_trajectory.py
```

This opens a 3D Matplotlib window. The grey line is the whole trajectory, the
orange dots are CSV rows, the blue line is the already-played portion, and
the red line points toward the drone's commanded heading. Close the window
to exit.

To preview your own CSV (quote paths containing spaces):

```bash
visualization/.venv/bin/python visualization/view_trajectory.py "/absolute/path/to/your_trajectory.csv"
```

To save a GIF without opening a window:

```bash
visualization/.venv/bin/python visualization/view_trajectory.py \
  visualization/example_trajectory.csv --save visualization/preview.gif
```

Optional: `--fps 30` changes the animation frame rate; `--speed 2` plays
twice as fast. The viewer converts PX4 local NED to a human-friendly plot:
`x = North`, `y = East`, `height = -z`. It does **not** convert the CSV itself.
The CSV remains in PX4 local NED, with the same twelve-column header required
by `trajectory_follower`:

```csv
time_s,x,y,z,vx,vy,vz,ax,ay,az,yaw,yawspeed
```

The example takes off, flies a square, and lands *in the preview*. It is not
a ready-to-fly command: check the first pose, frame, timing, limits, and
landing procedure before using any CSV on a real vehicle.
