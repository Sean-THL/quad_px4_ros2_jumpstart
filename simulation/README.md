# Run the CSV follower against PX4 SITL with Docker

Everything specific to this simulation workflow lives in this folder. Its
`Dockerfile` uses ROS Humble on the host's architecture, without Julia or
physical Vicon/serial devices. The repository-root `Dockerfile` and
`docker-compose.yaml` belong to the original hardware workflow.

PX4 SITL and Gazebo run on the host. This container runs the XRCE-DDS agent
and the CSV follower. Both sides use DDS domain 4 and namespace `px4_100`.

## One-time setup

Install PX4 SITL and Gazebo on the host using the official Ubuntu setup guide:
https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu

From this repository's root, initialize its ROS dependencies (the submodule
URLs use SSH and require the corresponding repository access):

```bash
git submodule update --init --recursive
docker compose -f simulation/docker-compose.yaml build
docker compose -f simulation/docker-compose.yaml run --rm ros_sim \
  colcon build --symlink-install --packages-up-to trajectory_follower
```

The `px4_msgs` definitions in the workspace must match the PX4 version used
for SITL. This image uses Micro XRCE-DDS Agent v2.4.3, which builds in this
container. PX4's compatibility table lists v2.4.2 for ROS Humble, but the
v2.4.2 source build currently fails to fetch its Fast DDS 2.12.x dependency.
Check the DDS connection and the pinned `px4_msgs` fork against your PX4 SITL
version before attempting flight.

## Each simulation run

Terminal 1, from the host's PX4-Autopilot directory:

```bash
ROS_DOMAIN_ID=4 PX4_UXRCE_DDS_NS=px4_100 make px4_sitl gz_x500
```

Terminal 2, from this repository's root:

```bash
docker compose -f simulation/docker-compose.yaml up -d ros_sim
docker compose -f simulation/docker-compose.yaml logs -f ros_sim
```

Terminal 3, from this repository's root. The example CSV in this folder is
mounted at `/home/simulation/example_trajectory.csv` inside the container:

```bash
docker compose -f simulation/docker-compose.yaml exec ros_sim bash -lc \
  'source /opt/ros/humble/setup.bash && source /home/colcon_ws/install/setup.bash && ros2 launch trajectory_follower follow_csv.launch.py trajectory_file:=/home/simulation/example_trajectory.csv px4_namespace:=/px4_100 target_system:=1'
```

Terminal 4, from this repository's root, inspect the simulated state:

```bash
docker compose -f simulation/docker-compose.yaml exec ros_sim bash -lc \
  'source /opt/ros/humble/setup.bash && source /home/colcon_ws/install/setup.bash && ros2 topic echo /px4_100/fmu/out/vehicle_local_position --once'
docker compose -f simulation/docker-compose.yaml exec ros_sim bash -lc \
  'source /opt/ros/humble/setup.bash && source /home/colcon_ws/install/setup.bash && ros2 topic echo /px4_100/fmu/out/vehicle_status --once'
```

The CSV's first position and yaw must match the simulated PX4 local pose.
Calling this service requests Offboard mode and arming:

```bash
docker compose -f simulation/docker-compose.yaml exec ros_sim bash -lc \
  "source /opt/ros/humble/setup.bash && source /home/colcon_ws/install/setup.bash && ros2 service call /start_trajectory std_srvs/srv/Trigger '{}'"
```

The follower holds the final setpoint. Use QGroundControl to land or change
mode. Do not launch `all_launch px4.launch.py` or Eware for this simulation.

## Remove the simulation later

From the repository root, stop and remove the simulation container and its
locally built image before deleting this folder:

```bash
docker compose -f simulation/docker-compose.yaml down --rmi local
```

The reusable `colcon_ws/src/trajectory_follower` package remains available
for real flights. PX4 SITL/Gazebo installed on the host and any existing
`colcon_ws/build`, `install`, or `log` artifacts are separate from this folder.
