"""Generate desired states live and stream them to PX4 without a CSV file."""

import math
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from px4_msgs.msg import (CommanderSetState, CommanderStatus,
                          OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger

from .live_trajectory import (DesiredState, TrajectoryConfig, desired_state,
                              velocity_aligned_state)


PRESTREAM_SECONDS = 1.2
COMMAND_TIMEOUT_SECONDS = 10.0
TELEMETRY_MAX_AGE_SECONDS = 1.0
TAKEOFF_SETTLE_SECONDS = 0.5
TAKEOFF_TIMEOUT_EXTRA_SECONDS = 10.0
TAKEOFF_POSITION_TOLERANCE_M = 0.12
TAKEOFF_SPEED_TOLERANCE_M_S = 0.20


class LiveTrajectoryFollower(Node):
    """Stream an analytic desired state while PX4 closes the feedback loop."""

    def __init__(self):
        super().__init__('live_trajectory_follower')
        self._load_configuration()
        self._create_ros_interfaces()
        self._initialize_state()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(
            f'Live {self.config.trajectory_type} trajectory ready '
            f'({self.config.duration_s:.1f} s); '
            'waiting for /start_live_trajectory. Landing remains under '
            'GUI/pilot control.')

    def _load_configuration(self):
        self.declare_parameter('px4_namespace', '/px4_100')
        self.declare_parameter('target_system', 100)
        self.declare_parameter('allow_unknown_system_id', False)
        self.declare_parameter('auto_arm_offboard', True)
        self.declare_parameter('auto_command_interface', 'vehicle_command')
        self.declare_parameter(
            'activation_timeout_s', COMMAND_TIMEOUT_SECONDS)
        self.declare_parameter('publish_rate_hz', 50.0)
        defaults = TrajectoryConfig()
        for name in TrajectoryConfig.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
        self.config = TrajectoryConfig(**{
            name: (str(self.get_parameter(name).value)
                   if name in ('trajectory_type', 'yaw_mode',
                               'takeoff_profile',
                               'offset_spiral_alignment_axis') else
                   float(self.get_parameter(name).value))
            for name in TrajectoryConfig.__dataclass_fields__})
        self.config.validate()
        self.target_system = int(self.get_parameter('target_system').value)
        self.allow_unknown_system_id = bool(
            self.get_parameter('allow_unknown_system_id').value)
        self.auto_arm_offboard = bool(
            self.get_parameter('auto_arm_offboard').value)
        self.auto_command_interface = str(
            self.get_parameter('auto_command_interface').value)
        self.activation_timeout_s = float(
            self.get_parameter('activation_timeout_s').value)
        self.rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if not 1 <= self.target_system <= 255:
            raise ValueError('target_system must be from 1 to 255')
        if self.auto_command_interface not in (
                'vehicle_command', 'commander_set_state'):
            raise ValueError('auto_command_interface must be vehicle_command '
                             'or commander_set_state')
        if (self.auto_command_interface == 'commander_set_state' and
                self.config.takeoff_profile != 'prearmed_hover'):
            raise ValueError('commander_set_state requires '
                             'takeoff_profile:=prearmed_hover')
        if not math.isfinite(self.rate_hz) or self.rate_hz < 10.0:
            raise ValueError(
                'publish_rate_hz must be finite and at least 10 Hz')
        if (not math.isfinite(self.activation_timeout_s) or
                self.activation_timeout_s < PRESTREAM_SECONDS):
            raise ValueError(
                'activation_timeout_s must be finite and at least 1.2 s')

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
        self.commander_set_state_publisher = self.create_publisher(
            CommanderSetState, prefix + 'in/commander_set_state', 10)
        self.error_publisher = self.create_publisher(
            Vector3Stamped, '~/tracking_error', 10)
        self.create_subscription(
            VehicleLocalPosition, prefix + 'out/vehicle_local_position',
            self.position_callback, qos_profile_sensor_data)
        self.create_subscription(
            VehicleStatus, prefix + 'out/vehicle_status',
            self.status_callback, qos_profile_sensor_data)
        self.create_subscription(
            CommanderStatus, prefix + 'out/commander_status',
            self.commander_status_callback, qos_profile_sensor_data)
        self.create_service(
            Trigger, 'start_live_trajectory', self.start_callback)

    def _initialize_state(self):
        self.position = None
        self.position_received_at = 0.0
        self.status = None
        self.status_received_at = 0.0
        self.commander_status = None
        self.commander_status_received_at = 0.0
        self.state = 'IDLE'
        self.origin = None
        self.start_yaw = 0.0
        self.commanded_yaw = 0.0
        self.warmup_started_at = 0.0
        self.commands_started_at = 0.0
        self.last_command_at = 0.0
        self.flight_started_at = 0.0
        self.takeoff_started_at = 0.0
        self.takeoff_ready_at = 0.0
        self.last_log_at = 0.0

    def position_callback(self, msg):
        if msg.xy_valid and msg.z_valid and msg.heading_good_for_control:
            self.position = msg
            self.position_received_at = time.monotonic()

    def status_callback(self, msg):
        self.status = msg
        self.status_received_at = time.monotonic()

    def commander_status_callback(self, msg):
        self.commander_status = msg
        self.commander_status_received_at = time.monotonic()

    def start_callback(self, request, response):
        del request
        now = time.monotonic()
        error = self._start_error(now)
        if error:
            response.message = error
            return response
        if self.status.system_id == 0:
            self.get_logger().warn(
                'VehicleStatus.system_id is unavailable (0); using the '
                f'explicitly configured target_system={self.target_system}')
        self.origin = (self.position.x, self.position.y, self.position.z)
        if (self.config.takeoff_profile == 'prearmed_hover' and
                self.config.takeoff_target_z_m >= self.origin[2]):
            response.message = (
                f'Pre-arm target Z {self.config.takeoff_target_z_m:.3f} m '
                f'is not above current local Z {self.origin[2]:.3f} m '
                '(PX4 NED: negative Z is up)')
            return response
        self.start_yaw = self.position.heading
        self.commanded_yaw = self.start_yaw
        self.state = 'PRESTREAM'
        self.warmup_started_at = now
        response.success = True
        target = ('hover-height target' if
                  self.config.takeoff_profile == 'prearmed_hover' else
                  'current Vicon-backed pose')
        action = ('requesting Arm then Offboard' if self.auto_arm_offboard else
                  'waiting for GUI Arm and Offboard')
        response.message = f'Prestreaming {target}; {action} in 1.2 s'
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
        if (self.commander_status is None or
                now - self.commander_status_received_at >
                TELEMETRY_MAX_AGE_SECONDS):
            return 'No recent PX4 commander status'
        if (self.status.system_id == 0 and
                not self.allow_unknown_system_id):
            return ('PX4 system ID is unavailable (0); verify MAV_SYS_ID, '
                    'then relaunch with allow_unknown_system_id:=true')
        if (self.status.system_id != 0 and
                self.status.system_id != self.target_system):
            return (f'PX4 system ID is {self.status.system_id}; '
                    f'target_system is {self.target_system}')
        if self.commander_status.state != CommanderStatus.STATE_DISARMED:
            return ('Commander reports state '
                    f'{self.commander_status.state}; start from DISARMED')
        return None

    def timestamp_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def _hold_state(self):
        z = self.origin[2]
        if self.config.takeoff_profile == 'prearmed_hover':
            z = self.config.takeoff_target_z_m
        return DesiredState(self.origin[0], self.origin[1], z,
                            0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, self.start_yaw, 0.0)

    def publish_mode(self):
        if self.auto_command_interface == 'commander_set_state':
            return  # The custom commander uses the GUI's setpoint stream.
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_us()
        msg.position = True
        self.mode_publisher.publish(msg)

    def publish_setpoint(self, point):
        if self.config.yaw_mode == 'velocity':
            point = velocity_aligned_state(point, self.commanded_yaw)
            self.commanded_yaw = point.yaw
        msg = TrajectorySetpoint()
        gui_format = self.auto_command_interface == 'commander_set_state'
        msg.timestamp = 0 if gui_format else self.timestamp_us()
        msg.position = [point.x, point.y, point.z]
        # The custom simple_commander receives the same position-only fields
        # as the RViz GUI that has flown successfully on this airframe.
        msg.velocity = ([0.0] * 3 if gui_format else
                        [point.vx, point.vy, point.vz])
        msg.acceleration = ([0.0] * 3 if gui_format else
                            [point.ax, point.ay, point.az])
        msg.jerk = ([0.0] * 3 if gui_format else [math.nan] * 3)
        msg.yaw = point.yaw
        msg.yawspeed = 0.0 if gui_format else point.yawspeed
        msg.raw_mode = False
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

    def publish_commander_state(self, state):
        msg = CommanderSetState()
        msg.timestamp = (0 if self.auto_command_interface ==
                         'commander_set_state' else self.timestamp_us())
        msg.new_state = state
        self.commander_set_state_publisher.publish(msg)

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
        if (self.state in ('TAKEOFF_HOLD', 'TAKEOFF_FAILED',
                           'ACTIVE', 'HOLD') and
                not self._offboard_is_healthy(now)):
            return

        if (self.state == 'PRESTREAM' and
                now - self.warmup_started_at >= PRESTREAM_SECONDS):
            self.state = 'WAIT_ACTIVE'
            self.commands_started_at = now
            self.last_command_at = 0.0
        elif self.state == 'WAIT_ACTIVE':
            if self._is_armed_offboard(now):
                if self.config.takeoff_profile == 'prearmed_hover':
                    self.state = 'TAKEOFF_HOLD'
                    self.takeoff_started_at = now
                    self.takeoff_ready_at = 0.0
                    self.get_logger().info(
                        'PX4 armed in Offboard; holding the prearmed hover '
                        'target until stable before starting the trajectory')
                else:
                    self.state = 'ACTIVE'
                    self.flight_started_at = now
                    self.get_logger().info(
                        'PX4 armed in Offboard; live trajectory started')
            elif (now - self.commands_started_at >
                  self.activation_timeout_s):
                self.state = 'FAILED'
                source = 'automatic commands' if self.auto_arm_offboard else 'GUI'
                self.get_logger().error(
                    'PX4 did not confirm armed Offboard from '
                    f'{source} within {self.activation_timeout_s:g} s; '
                    f'commander state={self.commander_status.state if self.commander_status else "unknown"}')
                return

        if self.state == 'TAKEOFF_HOLD':
            self._update_takeoff_hold(now)

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

    def _update_takeoff_hold(self, now):
        elapsed = now - self.takeoff_started_at
        if elapsed > (self.config.takeoff_duration_s +
                      TAKEOFF_TIMEOUT_EXTRA_SECONDS):
            self.state = 'TAKEOFF_FAILED'
            self.get_logger().error(
                'Takeoff hover was not reached and settled; holding target. '
                'Land using the GUI or RC.')
            return
        if elapsed < self.config.takeoff_duration_s:
            return
        target = self._hold_state()
        position_error = math.dist(
            (self.position.x, self.position.y, self.position.z),
            (target.x, target.y, target.z))
        speed = math.sqrt(
            self.position.vx**2 + self.position.vy**2 + self.position.vz**2)
        ready = (self.position.v_xy_valid and self.position.v_z_valid and
                 position_error <= TAKEOFF_POSITION_TOLERANCE_M and
                 speed <= TAKEOFF_SPEED_TOLERANCE_M_S)
        if not ready:
            self.takeoff_ready_at = 0.0
        elif self.takeoff_ready_at == 0.0:
            self.takeoff_ready_at = now
        elif now - self.takeoff_ready_at >= TAKEOFF_SETTLE_SECONDS:
            self.state = 'ACTIVE'
            self.flight_started_at = now - self.config.takeoff_duration_s
            self.get_logger().info(
                'Takeoff hover stable; live trajectory started')

    def _is_armed_offboard(self, now):
        # This custom firmware reports the GUI's authoritative state here.
        return (self.commander_status is not None and
                now - self.commander_status_received_at <=
                TELEMETRY_MAX_AGE_SECONDS and
                self.commander_status.state ==
                CommanderStatus.STATE_OFFBOARD)

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
        if (self.auto_arm_offboard and
                self.state == 'WAIT_ACTIVE' and
                now - self.last_command_at >= 1.0 and
                self.commander_status is not None and
                now - self.commander_status_received_at <=
                TELEMETRY_MAX_AGE_SECONDS):
            state = self.commander_status.state
            if state == CommanderStatus.STATE_DISARMED:
                if self.auto_command_interface == 'commander_set_state':
                    self.publish_commander_state(CommanderSetState.STATE_ARMED)
                else:
                    self.publish_command(
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            elif state == CommanderStatus.STATE_ARMED:
                if self.auto_command_interface == 'commander_set_state':
                    self.publish_commander_state(
                        CommanderSetState.STATE_OFFBOARD)
                else:
                    self.publish_command(
                        VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            else:
                return
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
