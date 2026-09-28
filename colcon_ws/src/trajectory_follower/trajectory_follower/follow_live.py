"""Generate desired states live and stream them to PX4 without a CSV file."""

import math
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger

from .live_trajectory import DesiredState, TrajectoryConfig, desired_state


PRESTREAM_SECONDS = 1.2
COMMAND_TIMEOUT_SECONDS = 10.0
TELEMETRY_MAX_AGE_SECONDS = 1.0


class LiveTrajectoryFollower(Node):
    """Stream an analytic desired state while PX4 closes the feedback loop."""

    def __init__(self):
        super().__init__('live_trajectory_follower')
        self._load_configuration()
        self._create_ros_interfaces()
        self._initialize_state()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(
            f'Live trajectory ready ({self.config.duration_s:.1f} s); '
            'waiting for /start_live_trajectory. Landing remains under '
            'GUI/pilot control.')

    def _load_configuration(self):
        self.declare_parameter('px4_namespace', '/px4_100')
        self.declare_parameter('target_system', 100)
        self.declare_parameter('publish_rate_hz', 50.0)
        defaults = TrajectoryConfig()
        for name, field in TrajectoryConfig.__dataclass_fields__.items():
            self.declare_parameter(name, getattr(defaults, name))
        self.config = TrajectoryConfig(**{
            name: float(self.get_parameter(name).value)
            for name in TrajectoryConfig.__dataclass_fields__})
        self.config.validate()
        self.target_system = int(self.get_parameter('target_system').value)
        self.rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if not 1 <= self.target_system <= 255:
            raise ValueError('target_system must be from 1 to 255')
        if not math.isfinite(self.rate_hz) or self.rate_hz < 10.0:
            raise ValueError(
                'publish_rate_hz must be finite and at least 10 Hz')

    def _create_ros_interfaces(self):
        namespace = self.get_parameter('px4_namespace').value.strip('/')
        if not namespace:
            raise ValueError('px4_namespace cannot be empty')
        prefix = f'/{namespace}/fmu/'
        self.mode_publisher = self.create_publisher(
            OffboardControlMode, prefix + 'in/offboard_control_mode', 10)
        self.setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, prefix + 'in/trajectory_setpoint', 10)
        self.command_publisher = self.create_publisher(
            VehicleCommand, prefix + 'in/vehicle_command', 10)
        self.error_publisher = self.create_publisher(
            Vector3Stamped, '~/tracking_error', 10)
        self.create_subscription(
            VehicleLocalPosition, prefix + 'out/vehicle_local_position',
            self.position_callback, qos_profile_sensor_data)
        self.create_subscription(
            VehicleStatus, prefix + 'out/vehicle_status',
            self.status_callback, qos_profile_sensor_data)
        self.create_service(
            Trigger, 'start_live_trajectory', self.start_callback)

    def _initialize_state(self):
        self.position = None
        self.position_received_at = 0.0
        self.status = None
        self.status_received_at = 0.0
        self.state = 'IDLE'
        self.origin = None
        self.start_yaw = 0.0
        self.warmup_started_at = 0.0
        self.commands_started_at = 0.0
        self.last_command_at = 0.0
        self.flight_started_at = 0.0
        self.last_log_at = 0.0

    def position_callback(self, msg):
        if msg.xy_valid and msg.z_valid and msg.heading_good_for_control:
            self.position = msg
            self.position_received_at = time.monotonic()

    def status_callback(self, msg):
        self.status = msg
        self.status_received_at = time.monotonic()

    def start_callback(self, request, response):
        del request
        now = time.monotonic()
        error = self._start_error(now)
        if error:
            response.message = error
            return response
        self.origin = (self.position.x, self.position.y, self.position.z)
        self.start_yaw = self.position.heading
        self.state = 'PRESTREAM'
        self.warmup_started_at = now
        response.success = True
        response.message = (
            'Holding current Vicon-backed pose; requesting armed Offboard '
            'in 1.2 s')
        self.get_logger().info(response.message)
        return response

    def _start_error(self, now):
        if self.state != 'IDLE':
            return f'Already started or stopped: {self.state}'
        if (self.position is None or
                now - self.position_received_at > TELEMETRY_MAX_AGE_SECONDS):
            return ('No recent valid Vicon-backed PX4 local position and '
                    'heading')
        if (self.status is None or
                now - self.status_received_at > TELEMETRY_MAX_AGE_SECONDS):
            return 'No recent PX4 vehicle status'
        if self.status.system_id != self.target_system:
            return (f'PX4 system ID is {self.status.system_id}; '
                    f'target_system is {self.target_system}')
        if self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            return ('Vehicle is already armed; start from the disarmed '
                    'launch pose')
        return None

    def timestamp_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def _hold_state(self):
        return DesiredState(*self.origin, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, self.start_yaw, 0.0)

    def publish_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_us()
        msg.position = True
        self.mode_publisher.publish(msg)

    def publish_setpoint(self, point):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_us()
        msg.position = [point.x, point.y, point.z]
        msg.velocity = [point.vx, point.vy, point.vz]
        msg.acceleration = [point.ax, point.ay, point.az]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = point.yaw
        msg.yawspeed = point.yawspeed
        self.setpoint_publisher.publish(msg)

    def publish_command(self, command, param1, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_publisher.publish(msg)

    def publish_tracking_error(self, point, now):
        if (self.position is None or
                now - self.position_received_at > TELEMETRY_MAX_AGE_SECONDS):
            return
        error = (point.x - self.position.x,
                 point.y - self.position.y,
                 point.z - self.position.z)
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'px4_local_ned'
        msg.vector.x, msg.vector.y, msg.vector.z = error
        self.error_publisher.publish(msg)
        if now - self.last_log_at >= 1.0:
            magnitude = math.sqrt(sum(value*value for value in error))
            self.get_logger().info(
                f'{self.state}: tracking error {magnitude:.3f} m '
                f'(NED {error[0]:+.3f}, {error[1]:+.3f}, {error[2]:+.3f})')
            self.last_log_at = now

    def tick(self):
        if self.state in ('IDLE', 'FAILED', 'INTERRUPTED'):
            return
        now = time.monotonic()
        if (self.state in ('ACTIVE', 'HOLD') and
                not self._offboard_is_healthy(now)):
            return

        if (self.state == 'PRESTREAM' and
                now - self.warmup_started_at >= PRESTREAM_SECONDS):
            self.state = 'WAIT_ACTIVE'
            self.commands_started_at = now
            self.last_command_at = 0.0
        elif self.state == 'WAIT_ACTIVE':
            if self._is_armed_offboard(now):
                self.state = 'ACTIVE'
                self.flight_started_at = now
                self.get_logger().info(
                    'PX4 armed in Offboard; live trajectory started')
            elif now - self.commands_started_at > COMMAND_TIMEOUT_SECONDS:
                self.state = 'FAILED'
                self.get_logger().error(
                    'PX4 did not confirm armed Offboard within 10 s')
                return

        point = self._hold_state()
        if self.state == 'ACTIVE':
            elapsed = now - self.flight_started_at
            point = desired_state(
                elapsed, self.origin, self.start_yaw, self.config)
            if elapsed >= self.config.duration_s:
                self.state = 'HOLD'
                self.get_logger().info(
                    'Trajectory finished; holding final hover. Land with the '
                    'existing GUI.')
        elif self.state == 'HOLD':
            point = desired_state(self.config.duration_s, self.origin,
                                  self.start_yaw, self.config)

        self.publish_mode()
        self.publish_setpoint(point)
        self.publish_tracking_error(point, now)
        self._maybe_send_start_commands(now)

    def _is_armed_offboard(self, now):
        return (self.status is not None and
                now - self.status_received_at <= TELEMETRY_MAX_AGE_SECONDS and
                self.status.nav_state ==
                VehicleStatus.NAVIGATION_STATE_OFFBOARD and
                self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def _offboard_is_healthy(self, now):
        if not self._is_armed_offboard(now):
            self.get_logger().warn(
                'PX4 left armed Offboard state; stopping setpoint stream')
            self.state = 'INTERRUPTED'
            return False
        if now - self.position_received_at > TELEMETRY_MAX_AGE_SECONDS:
            self.get_logger().error(
                'Vicon-backed local position became stale; stopping stream')
            self.state = 'INTERRUPTED'
            return False
        return True

    def _maybe_send_start_commands(self, now):
        if (self.state == 'WAIT_ACTIVE' and
                now - self.last_command_at >= 1.0 and
                self.status is not None and
                now - self.status_received_at <= TELEMETRY_MAX_AGE_SECONDS):
            self.publish_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publish_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.last_command_at = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LiveTrajectoryFollower()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
