"""Stream a precomputed CSV trajectory to PX4 in Offboard position mode."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLandDetected,
                          VehicleLocalPosition, VehicleStatus)
from std_srvs.srv import Trigger

from .landing import LAND_SETTLE_TIME_S, settled_at_target
from .trajectory import Setpoint, load_trajectory, sample


PRESTREAM_SECONDS = 1.2
ARM_TIMEOUT_SECONDS = 10.0
LAND_CONFIRM_TIMEOUT_SECONDS = 3.0
LAND_MONITOR_WARNING_SECONDS = 15.0
TELEMETRY_MAX_AGE_SECONDS = 1.0


class CsvTrajectoryFollower(Node):
    """Read one CSV, stream setpoints, and monitor PX4 mode/landing state.

    IDLE -> PRESTREAM -> WAIT_ACTIVE -> ACTIVE -> HOLD; optional auto-landing
    continues via LAND_REQUESTED -> LANDING -> LANDED. Failure states stop
    trajectory playback or keep the final hover while the pilot takes over.
    """

    def __init__(self):
        super().__init__('trajectory_follower')
        self._load_configuration()
        self._create_ros_interfaces()
        self._initialize_flight_state()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(
            f'Loaded {len(self.points)} points ({self.times[-1]:.2f} s). '
            f'Automatic landing {"enabled" if self.land_on_complete else "disabled"}. '
            'Waiting for /start_trajectory service call.')

    def _load_configuration(self):
        """Declare and validate launch parameters; load the CSV once."""
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('px4_namespace', '/px4_100')
        self.declare_parameter('target_system', 100)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('max_start_distance_m', 0.5)
        self.declare_parameter('max_start_yaw_error_rad', 0.5)
        self.declare_parameter('land_on_complete', False)

        path = self.get_parameter('trajectory_file').value
        if not path:
            raise ValueError('Set the trajectory_file parameter to an absolute CSV path')
        self.points = load_trajectory(path)
        self.times = [point.time_s for point in self.points]
        self.target_system = int(self.get_parameter('target_system').value)
        self.rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.max_start_distance = float(self.get_parameter('max_start_distance_m').value)
        self.max_start_yaw_error = float(self.get_parameter('max_start_yaw_error_rad').value)
        self.land_on_complete = self.get_parameter('land_on_complete').value
        if not isinstance(self.land_on_complete, bool):
            raise ValueError('land_on_complete must be true or false')
        if not 1 <= self.target_system <= 255:
            raise ValueError('target_system must be a MAVLink system ID from 1 to 255')
        if self.rate_hz < 10.0 or not math.isfinite(self.rate_hz):
            raise ValueError('publish_rate_hz must be at least 10 Hz')
        if self.max_start_distance <= 0.0 or not math.isfinite(self.max_start_distance):
            raise ValueError('max_start_distance_m must be positive')
        if self.max_start_yaw_error <= 0.0 or not math.isfinite(self.max_start_yaw_error):
            raise ValueError('max_start_yaw_error_rad must be positive')

    def _create_ros_interfaces(self):
        """Connect the three PX4 command topics and three feedback topics."""
        namespace = self.get_parameter('px4_namespace').value.strip('/')
        if not namespace:
            raise ValueError('px4_namespace cannot be empty')
        prefix = '/' + namespace + '/fmu/'

        self.mode_publisher = self.create_publisher(
            OffboardControlMode, prefix + 'in/offboard_control_mode', 10)
        self.setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, prefix + 'in/trajectory_setpoint', 10)
        self.command_publisher = self.create_publisher(
            VehicleCommand, prefix + 'in/vehicle_command', 10)
        self.create_subscription(VehicleLocalPosition, prefix + 'out/vehicle_local_position',
                                 self.position_callback, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, prefix + 'out/vehicle_status',
                                 self.status_callback, qos_profile_sensor_data)
        self.create_subscription(VehicleLandDetected, prefix + 'out/vehicle_land_detected',
                                 self.land_detected_callback, qos_profile_sensor_data)
        self.create_service(Trigger, 'start_trajectory', self.start_callback)

    def _initialize_flight_state(self):
        """Nothing is streamed until the start service leaves IDLE."""
        self.position = None
        self.position_received_at = 0.0
        self.status = None
        self.status_received_at = 0.0
        self.land_detected = None
        self.land_detected_received_at = 0.0
        self.state = 'IDLE'
        self.hold_point = None
        self.warmup_started_at = 0.0
        self.commands_started_at = 0.0
        self.last_command_at = 0.0
        self.flight_started_at = 0.0
        self.last_point = None
        self.land_settled_at = None
        self.land_requested_at = 0.0
        self.land_detector_warning_logged = False
        self.touchdown_logged = False
        self.land_monitor_warning_logged = False

    # PX4 telemetry: cache valid messages and their local receipt times.
    def position_callback(self, msg):
        if msg.xy_valid and msg.z_valid and msg.heading_good_for_control:
            self.position = msg
            self.position_received_at = time.monotonic()

    def status_callback(self, msg):
        self.status = msg
        self.status_received_at = time.monotonic()

    def land_detected_callback(self, msg):
        self.land_detected = msg
        self.land_detected_received_at = time.monotonic()

    def land_detector_is_fresh(self, now):
        return (self.land_detected is not None and
                now - self.land_detected_received_at <= TELEMETRY_MAX_AGE_SECONDS)

    # /start_trajectory: this is the only normal entry into the flight sequence.
    def start_callback(self, request, response):
        del request
        now = time.monotonic()
        error = self._start_error(now)
        if error:
            response.message = error
            return response

        self.hold_point = Setpoint(
            0.0, self.position.x, self.position.y, self.position.z,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.position.heading, 0.0)
        self.state = 'PRESTREAM'
        self.warmup_started_at = now
        response.success = True
        response.message = 'Streaming hold setpoint; Offboard/arm requested after 1.2 s'
        self.get_logger().info(response.message)
        return response

    def _start_error(self, now):
        """Return a refusal reason, or None when it is safe to prestream."""
        if self.state != 'IDLE':
            return f'Already started or stopped: {self.state}'
        if self.position is None or now - self.position_received_at > TELEMETRY_MAX_AGE_SECONDS:
            return 'No recent valid PX4 local position and heading'
        if self.status is None or now - self.status_received_at > TELEMETRY_MAX_AGE_SECONDS:
            return 'No recent PX4 vehicle status'
        if self.status.system_id != self.target_system:
            return (f'PX4 system ID is {self.status.system_id}; '
                    f'target_system is {self.target_system}')
        if self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            return 'Vehicle is already armed; start from disarmed state'
        if (self.land_on_complete and
                (not self.land_detector_is_fresh(now) or not self.land_detected.landed)):
            return ('Automatic landing requires fresh PX4 '
                    'vehicle_land_detected.landed=true before takeoff')

        first = self.points[0]
        distance = math.dist((first.x, first.y, first.z),
                             (self.position.x, self.position.y, self.position.z))
        yaw_error = abs((first.yaw - self.position.heading + math.pi) %
                        (2.0 * math.pi) - math.pi)
        if distance > self.max_start_distance:
            return (f'First CSV point is {distance:.2f} m from current pose; '
                    f'limit is {self.max_start_distance:.2f} m')
        if yaw_error > self.max_start_yaw_error:
            return (f'First CSV yaw is {yaw_error:.2f} rad from current heading; '
                    f'limit is {self.max_start_yaw_error:.2f} rad')
        return None

    # ROS message construction: setpoints are continuous; commands are events.
    def timestamp_us(self):
        return self.get_clock().now().nanoseconds // 1000

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
        msg.jerk = [0.0, 0.0, 0.0]
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

    def request_land(self):
        """Ask PX4 to land at its current position (no global coordinates)."""
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        msg.param1 = math.nan
        msg.param2 = math.nan
        msg.param3 = math.nan
        msg.param4 = math.nan
        msg.param5 = math.nan
        msg.param6 = math.nan
        msg.param7 = math.nan
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_publisher.publish(msg)

    # Flight state machine: one timer tick chooses and publishes one setpoint.
    def tick(self):
        if self.state in ('IDLE', 'INTERRUPTED', 'LANDED'):
            return
        now = time.monotonic()
        if self.state == 'LANDING':
            self._monitor_landing(now)
            return
        if not self._check_offboard_state(now):
            return

        self._advance_startup(now)
        point = self._point_for_state(now)

        self.publish_mode()
        self.publish_setpoint(point)
        self._maybe_request_land(point, now)
        self._check_land_request_timeout(now)
        self._maybe_send_start_commands(now)

    def _monitor_landing(self, now):
        """PX4 owns the descent; report touchdown and later disarming."""
        if self.land_detector_is_fresh(now) and self.land_detected.landed:
            if not self.touchdown_logged:
                self.get_logger().info(
                    'PX4 land detector reports touchdown; waiting for disarm')
                self.touchdown_logged = True
            if (self.status is not None and
                    now - self.status_received_at <= TELEMETRY_MAX_AGE_SECONDS and
                    self.status_received_at >= self.land_requested_at and
                    self.status.arming_state == VehicleStatus.ARMING_STATE_STANDBY):
                self.state = 'LANDED'
                self.get_logger().info('PX4 reports landed and disarmed')
                return
        if (now - self.land_requested_at > LAND_MONITOR_WARNING_SECONDS and
                not self.land_monitor_warning_logged):
            self.get_logger().warn(
                'Landing not yet confirmed by landed + disarmed telemetry; '
                'inspect PX4/QGroundControl and be ready to take manual control')
            self.land_monitor_warning_logged = True

    def _check_offboard_state(self, now):
        """Stop playback if PX4 status is stale or PX4 leaves armed Offboard."""
        if self.state not in ('ACTIVE', 'HOLD', 'LAND_REQUESTED', 'LAND_FAILED'):
            return True
        if self.status is None or now - self.status_received_at > TELEMETRY_MAX_AGE_SECONDS:
            self.get_logger().error('PX4 status became stale; stopping trajectory playback')
            self.state = 'INTERRUPTED'
            return False
        if (self.state == 'LAND_REQUESTED' and
                self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND):
            self.state = 'LANDING'
            self.get_logger().info('PX4 entered Land mode; Offboard streaming stopped')
            return False
        if (self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD or
                self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED):
            self.get_logger().warn('PX4 left armed Offboard state; stopping playback')
            self.state = 'INTERRUPTED'
            return False
        return True

    def _advance_startup(self, now):
        """Prestream before requesting Offboard; start timing only when armed."""
        if self.state == 'PRESTREAM':
            if now - self.warmup_started_at >= PRESTREAM_SECONDS:
                self.state = 'WAIT_ACTIVE'
                self.commands_started_at = now
                self.last_command_at = 0.0
        elif self.state == 'WAIT_ACTIVE':
            if (self.status is not None and
                    now - self.status_received_at <= TELEMETRY_MAX_AGE_SECONDS and
                    self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
                    self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED):
                self.state = 'ACTIVE'
                self.flight_started_at = now
                self.get_logger().info('PX4 armed in Offboard; starting CSV trajectory')
            elif now - self.commands_started_at > ARM_TIMEOUT_SECONDS:
                self.state = 'FAILED'
                self.get_logger().error(
                    'PX4 did not confirm armed Offboard within 10 s; holding start pose. '
                    'Check PX4/QGroundControl status and restart the node to retry.')

    def _point_for_state(self, now):
        """Use the timed CSV while active; otherwise hold start or final pose."""
        if self.state == 'ACTIVE':
            elapsed = now - self.flight_started_at
            point = sample(self.points, self.times, elapsed)
            self.last_point = point
            if elapsed >= self.times[-1]:
                self.state = 'HOLD'
                self.get_logger().info('Trajectory finished; holding final position')
            return point
        if self.state in ('HOLD', 'LAND_REQUESTED', 'LAND_FAILED') and self.last_point is not None:
            end = self.last_point
            return Setpoint(end.time_s, end.x, end.y, end.z,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, end.yaw, 0.0)
        return self.hold_point

    def _maybe_request_land(self, point, now):
        """Ask PX4 to land only after a stable final hover and valid telemetry."""
        if self.state != 'HOLD' or not self.land_on_complete:
            return
        if not self.land_detector_is_fresh(now):
            self.land_settled_at = None
            if not self.land_detector_warning_logged:
                self.get_logger().warn(
                    'No fresh PX4 land-detector topic; holding final hover '
                    'instead of requesting automatic landing')
                self.land_detector_warning_logged = True
        elif self.land_detected.landed or self.land_detected.freefall:
            self.land_settled_at = None
            if not self.land_detector_warning_logged:
                self.get_logger().warn(
                    'PX4 land detector reports landed/freefall at final hover; '
                    'holding instead of requesting automatic landing')
                self.land_detector_warning_logged = True
        elif settled_at_target(self.position, self.position_received_at, point, now):
            if self.land_settled_at is None:
                self.land_settled_at = now
            elif now - self.land_settled_at >= LAND_SETTLE_TIME_S:
                self.request_land()
                self.land_requested_at = now
                self.state = 'LAND_REQUESTED'
                self.get_logger().info('Final hover reached; requesting PX4 Land mode')
        else:
            self.land_settled_at = None

    def _check_land_request_timeout(self, now):
        if (self.state == 'LAND_REQUESTED' and
                now - self.land_requested_at > LAND_CONFIRM_TIMEOUT_SECONDS):
            self.state = 'LAND_FAILED'
            self.get_logger().error(
                'PX4 did not confirm Land mode; holding final position. '
                'Land manually or change mode using the ground station.')

    def _maybe_send_start_commands(self, now):
        if (self.state == 'WAIT_ACTIVE' and now - self.last_command_at >= 1.0 and
                self.status is not None and
                now - self.status_received_at <= TELEMETRY_MAX_AGE_SECONDS):
            self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.last_command_at = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CsvTrajectoryFollower()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
