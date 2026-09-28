"""Conservative one-key-at-a-time PX4 Offboard velocity teleoperation.

Run in an interactive TTY with `ros2 run trajectory_follower teleop_keyboard`.
This never arms or takes off. Take off first with a tested PX4 manual mode.
"""

import math
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

from .teleop_math import MOVE_KEYS, command_for_key


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('px4_namespace', '/px4_100')
        self.declare_parameter('target_system', 100)
        self.declare_parameter('horizontal_speed_m_s', 0.25)
        self.declare_parameter('vertical_speed_m_s', 0.15)
        self.declare_parameter('yaw_rate_rad_s', 0.35)
        self.declare_parameter('min_clearance_m', 0.4)
        self.declare_parameter('key_timeout_s', 0.3)

        self.target_system = int(self.get_parameter('target_system').value)
        self.horizontal_speed = float(self.get_parameter('horizontal_speed_m_s').value)
        self.vertical_speed = float(self.get_parameter('vertical_speed_m_s').value)
        self.yaw_rate = float(self.get_parameter('yaw_rate_rad_s').value)
        self.min_clearance = float(self.get_parameter('min_clearance_m').value)
        self.key_timeout = float(self.get_parameter('key_timeout_s').value)
        if not 1 <= self.target_system <= 255:
            raise ValueError('target_system must be 1..255')
        for name, value, maximum in (
            ('horizontal_speed_m_s', self.horizontal_speed, 1.0),
            ('vertical_speed_m_s', self.vertical_speed, 0.5),
            ('yaw_rate_rad_s', self.yaw_rate, 1.0),
            ('min_clearance_m', self.min_clearance, 10.0),
            ('key_timeout_s', self.key_timeout, 0.5),
        ):
            if not math.isfinite(value) or not 0 < value <= maximum:
                raise ValueError(f'{name} must be > 0 and <= {maximum}')

        namespace = str(self.get_parameter('px4_namespace').value).strip('/')
        if not namespace:
            raise ValueError('px4_namespace cannot be empty')
        prefix = '/' + namespace + '/fmu/'
        self.mode_pub = self.create_publisher(
            OffboardControlMode, prefix + 'in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, prefix + 'in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(
            VehicleCommand, prefix + 'in/vehicle_command', 10)
        self.create_subscription(VehicleLocalPosition, prefix + 'out/vehicle_local_position',
                                 self.on_position, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, prefix + 'out/vehicle_status',
                                 self.on_status, qos_profile_sensor_data)

        self.position = None
        self.position_at = 0.0
        self.status = None
        self.status_at = 0.0
        self.ground_down = None
        self.ground_reset_counter = None
        self.ground_reference_valid = False
        self.state = 'IDLE'
        self.state_since = 0.0
        self.last_command_at = 0.0
        self.key = None
        self.key_at = 0.0
        self.yaw_hold = None
        self.warned_about_state = False
        self.offboard_timeout_warned = False
        self.create_timer(0.05, self.tick)  # 20 Hz Offboard proof-of-life.
        self.get_logger().info(
            'W/A/S/D move, R/F up/down, Q/E yaw, SPACE stop, G enter Offboard, '
            'L request PX4 Land, X exit when disarmed. No automatic arm/takeoff.')

    def on_position(self, msg):
        now = time.monotonic()
        if (msg.xy_valid and msg.z_valid and msg.v_xy_valid and msg.v_z_valid and
                msg.heading_good_for_control and
                all(math.isfinite(v) for v in
                    (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.heading))):
            self.position = msg
            self.position_at = now
            # Capture the PX4 z at the launch spot while the vehicle is disarmed.
            # This is a soft descent guard, not a range-sensor measurement.
            if (self.state == 'IDLE' and self.status is not None and
                    now - self.status_at <= 1.0 and
                    self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED):
                self.ground_down = msg.z
                self.ground_reset_counter = msg.z_reset_counter
                self.ground_reference_valid = True
            elif (self.ground_reference_valid and
                  msg.z_reset_counter != self.ground_reset_counter):
                self.ground_reference_valid = False
                self.key = None
                self.get_logger().error(
                    'PX4 local z origin reset; keyboard motion stopped. '
                    'Take over with RC/QGroundControl and land.')

    def on_status(self, msg):
        self.status = msg
        self.status_at = time.monotonic()

    def fresh(self, now):
        return (self.position is not None and now - self.position_at <= 1.0 and
                self.status is not None and now - self.status_at <= 1.0)

    def timestamp_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def publish_command(self, command, param1=math.nan, param2=math.nan):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
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
        self.command_pub.publish(msg)

    def publish_zero_or_key(self, now):
        mode = OffboardControlMode()
        mode.timestamp = self.timestamp_us()
        mode.velocity = True
        self.mode_pub.publish(mode)

        north = east = down = yaw_rate = 0.0
        if (self.state == 'ACTIVE' and self.key is not None and
                now - self.key_at <= self.key_timeout and self.fresh(now) and
                self.ground_reference_valid):
            north, east, down, yaw_rate = command_for_key(
                self.key, self.position.heading, self.horizontal_speed,
                self.vertical_speed, self.yaw_rate)
            # Soft limit: do not intentionally descend into the captured
            # takeoff plane. This cannot account for inertia or uneven floors.
            if down > 0 and self.position.z >= self.ground_down - self.min_clearance:
                down = 0.0

        point = TrajectorySetpoint()
        point.timestamp = self.timestamp_us()
        point.position = [math.nan, math.nan, math.nan]  # Velocity control only.
        point.velocity = [north, east, down]           # PX4 local NED.
        point.acceleration = [math.nan, math.nan, math.nan]
        point.jerk = [math.nan, math.nan, math.nan]
        # During Q/E use yaw rate; after release capture and hold that heading.
        if yaw_rate:
            self.yaw_hold = None
        elif self.yaw_hold is None and self.fresh(now):
            self.yaw_hold = self.position.heading
        point.yaw = math.nan if yaw_rate or self.yaw_hold is None else self.yaw_hold
        point.yawspeed = yaw_rate
        self.setpoint_pub.publish(point)

    def on_key(self, key):
        now = time.monotonic()
        key = key.lower()
        if key == 'g':
            if self.state != 'IDLE':
                self.get_logger().warn(f'Cannot enter Offboard from {self.state}')
            elif not self.fresh(now):
                self.get_logger().warn('Need fresh valid PX4 position, velocity, heading and status')
            elif not self.ground_reference_valid:
                self.get_logger().warn('Start node while disarmed on takeoff spot to record ground z')
            elif self.status.system_id != self.target_system:
                self.get_logger().warn('PX4 system ID does not match target_system')
            elif self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().warn('Arm and take off in a tested PX4 manual mode first')
            elif self.position.z > self.ground_down - self.min_clearance - 0.1:
                self.get_logger().warn('Take off and settle above the soft minimum clearance first')
            elif self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().warn('PX4 is already in Offboard; another publisher may be active')
            elif self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                self.get_logger().warn('PX4 is landing; cannot enter keyboard Offboard')
            elif math.sqrt(self.position.vx**2 + self.position.vy**2 +
                           self.position.vz**2) > 0.3:
                self.get_logger().warn('Settle to a near-stationary hover before entering Offboard')
            else:
                self.state = 'PRESTREAM'
                self.state_since = now
                self.key = None
                self.yaw_hold = self.position.heading
                self.offboard_timeout_warned = False
                self.get_logger().info('Prestreaming zero velocity; requesting Offboard after 1.2 s')
        elif key in MOVE_KEYS:
            if self.state == 'ACTIVE':
                self.key = key
                self.key_at = now
        elif key == ' ':
            self.key = None
            self.yaw_hold = None
        elif key == 'l':
            if self.state == 'ACTIVE':
                self.key = None
                self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.state = 'LAND_REQUESTED'
                self.state_since = now
                self.get_logger().warn('PX4 Land requested; confirm Land mode in QGroundControl')
        elif key == 'x':
            if (self.fresh(now) and
                    self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED):
                return True
            self.get_logger().warn('Exit only after PX4 reports disarmed; use RC/QGroundControl for takeover')
        return False

    def tick(self):
        now = time.monotonic()
        if self.state in ('IDLE', 'LANDING'):
            return

        if self.state in ('ACTIVE', 'LAND_REQUESTED', 'LAND_FAILED') and self.fresh(now):
            if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                self.state = 'LANDING'
                self.get_logger().info('PX4 confirmed Land mode; Offboard streaming stopped')
                return
            if (self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD or
                    self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED):
                self.state = 'IDLE'
                self.key = None
                self.get_logger().warn('PX4 left armed Offboard; keyboard control stopped')
                return
        if self.state == 'PRESTREAM':
            if not self.fresh(now) or self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.state = 'IDLE'
                self.get_logger().error('Lost valid PX4 state or arming during prestream')
                return
            if now - self.state_since >= 1.2:
                self.state = 'WAIT_ACTIVE'
                self.state_since = now
                self.last_command_at = 0.0
        elif self.state == 'WAIT_ACTIVE':
            if self.fresh(now) and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.state = 'ACTIVE'
                self.get_logger().info('PX4 confirmed Offboard; keyboard keys are active')
            elif now - self.state_since > 8.0 and not self.offboard_timeout_warned:
                # Keep streaming zero velocity: PX4 might have entered
                # Offboard even if its status telemetry stopped arriving.
                self.offboard_timeout_warned = True
                self.get_logger().error(
                    'PX4 did not confirm Offboard; streaming zero velocity. '
                    'Check QGroundControl and take over with RC if needed.')
            elif self.fresh(now) and now - self.last_command_at >= 1.0:
                self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.last_command_at = now
        elif self.state == 'LAND_REQUESTED' and now - self.state_since > 3.0:
            self.state = 'LAND_FAILED'
            self.get_logger().error(
                'PX4 did not confirm Land; streaming zero velocity. '
                'Use QGroundControl/RC to take over and land.')

        if self.state == 'ACTIVE' and (not self.fresh(now) or not self.ground_reference_valid):
            self.key = None
            if not self.warned_about_state:
                self.get_logger().error('PX4 state/ground reference invalid; commanding zero velocity')
                self.warned_about_state = True
        else:
            self.warned_about_state = False
        self.publish_zero_or_key(now)


def main(args=None):
    if not sys.stdin.isatty():
        raise SystemExit('Keyboard teleop needs an interactive terminal (TTY).')
    rclpy.init(args=args)
    node = None
    fd = sys.stdin.fileno()
    original_terminal = termios.tcgetattr(fd)
    try:
        node = KeyboardTeleop()
        print('Keyboard: W/A/S/D move | R/F up/down | Q/E yaw | SPACE stop | '
              'G Offboard | L Land | X exit after disarm', flush=True)
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if select.select([sys.stdin], [], [], 0)[0]:
                if node.on_key(sys.stdin.read(1)):
                    break
    except KeyboardInterrupt:
        print('\nInterrupted: if PX4 is airborne in Offboard, its configured '
              'Offboard-loss failsafe applies. Take over using RC/QGroundControl.',
              flush=True)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, original_terminal)
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
