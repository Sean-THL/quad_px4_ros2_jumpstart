"""Exercise landing state transitions without creating a ROS node or publisher."""

import math
import unittest
from unittest.mock import Mock, patch

from px4_msgs.msg import (VehicleCommand, VehicleLandDetected,
                          VehicleLocalPosition, VehicleStatus)
from std_srvs.srv import Trigger

from trajectory_follower.follow_csv import CsvTrajectoryFollower
from trajectory_follower.trajectory import Setpoint


class FollowerLandingTest(unittest.TestCase):
    def setUp(self):
        self.node = object.__new__(CsvTrajectoryFollower)
        self.node.state = 'HOLD'
        self.node.land_on_complete = True
        self.node.last_point = Setpoint(2.0, 0.0, 0.0, -1.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.node.position = VehicleLocalPosition()
        self.node.position.xy_valid = True
        self.node.position.z_valid = True
        self.node.position.v_xy_valid = True
        self.node.position.v_z_valid = True
        self.node.position.x = 0.0
        self.node.position.y = 0.0
        self.node.position.z = -1.0
        self.node.position_received_at = 10.0
        self.node.status = VehicleStatus()
        self.node.status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node.status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        self.node.status_received_at = 10.0
        self.node.land_detected = VehicleLandDetected()
        self.node.land_detected_received_at = 10.0
        self.node.land_settled_at = None
        self.node.land_requested_at = 0.0
        self.node.land_detector_warning_logged = False
        self.node.touchdown_logged = False
        self.node.land_monitor_warning_logged = False
        self.node.publish_mode = Mock()
        self.node.publish_setpoint = Mock()
        self.node.request_land = Mock()
        self.node.get_logger = Mock(return_value=Mock())

    def tick_at(self, seconds):
        self.node.status_received_at = seconds
        self.node.position_received_at = seconds
        self.node.land_detected_received_at = seconds
        with patch('trajectory_follower.follow_csv.time.monotonic',
                   return_value=seconds):
            self.node.tick()

    def test_waits_for_settle_then_yields_to_px4_land_mode(self):
        self.tick_at(10.0)
        self.assertEqual(self.node.state, 'HOLD')
        self.tick_at(10.5)
        self.node.request_land.assert_not_called()
        self.tick_at(11.1)
        self.node.request_land.assert_called_once_with()
        self.assertEqual(self.node.state, 'LAND_REQUESTED')

        published_before = self.node.publish_setpoint.call_count
        self.node.status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
        self.tick_at(11.2)
        self.assertEqual(self.node.state, 'LANDING')
        self.assertEqual(self.node.publish_setpoint.call_count, published_before)
        self.node.land_detected.landed = True
        self.node.status.arming_state = VehicleStatus.ARMING_STATE_STANDBY
        self.tick_at(12.0)
        self.assertEqual(self.node.state, 'LANDED')

    def test_no_land_if_pose_invalid_or_request_times_out(self):
        self.node.position.v_z_valid = False
        self.tick_at(10.0)
        self.tick_at(12.0)
        self.node.request_land.assert_not_called()
        self.assertEqual(self.node.state, 'HOLD')

        self.node.position.v_z_valid = True
        self.tick_at(13.0)
        self.tick_at(14.1)
        self.assertEqual(self.node.state, 'LAND_REQUESTED')
        self.tick_at(17.2)
        self.assertEqual(self.node.state, 'LAND_FAILED')
        self.node.request_land.assert_called_once_with()

    def test_no_land_request_without_detector_telemetry(self):
        self.node.land_detected = None
        self.tick_at(10.0)
        self.tick_at(12.0)
        self.assertEqual(self.node.state, 'HOLD')
        self.node.request_land.assert_not_called()

    def test_auto_land_refuses_start_without_detector(self):
        self.node.state = 'IDLE'
        self.node.land_detected = None
        self.node.target_system = 100
        self.node.status.system_id = 100
        self.node.status.arming_state = VehicleStatus.ARMING_STATE_STANDBY
        self.node.max_start_distance = 0.5
        self.node.max_start_yaw_error = 0.5
        self.node.points = [self.node.last_point]
        with patch('trajectory_follower.follow_csv.time.monotonic', return_value=10.0):
            response = self.node.start_callback(Trigger.Request(), Trigger.Response())
        self.assertFalse(response.success)
        self.assertIn('vehicle_land_detected', response.message)

    def test_matching_start_pose_enters_prestream_without_auto_land(self):
        self.node.state = 'IDLE'
        self.node.land_on_complete = False
        self.node.target_system = 100
        self.node.status.system_id = 100
        self.node.status.arming_state = VehicleStatus.ARMING_STATE_STANDBY
        self.node.max_start_distance = 0.5
        self.node.max_start_yaw_error = 0.5
        self.node.points = [self.node.last_point]
        with patch('trajectory_follower.follow_csv.time.monotonic', return_value=10.0):
            response = self.node.start_callback(Trigger.Request(), Trigger.Response())
        self.assertTrue(response.success)
        self.assertEqual(self.node.state, 'PRESTREAM')
        self.assertAlmostEqual(self.node.hold_point.z, -1.0)

    def test_land_request_has_no_global_coordinates(self):
        self.node.target_system = 100
        self.node.timestamp_us = Mock(return_value=123)
        self.node.command_publisher = Mock()
        CsvTrajectoryFollower.request_land(self.node)
        command = self.node.command_publisher.publish.call_args.args[0]
        self.assertEqual(command.command, VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.assertEqual(command.target_system, 100)
        self.assertTrue(all(math.isnan(getattr(command, f'param{i}'))
                            for i in range(1, 8)))

    def test_prestream_arms_then_plays_and_holds_csv(self):
        self.node.state = 'PRESTREAM'
        self.node.land_on_complete = False
        self.node.hold_point = self.node.last_point
        self.node.warmup_started_at = 10.0
        self.node.points = [Setpoint(0.0, 0.0, 0.0, -1.0,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                            Setpoint(1.0, 0.0, 0.0, -2.0,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]
        self.node.times = [0.0, 1.0]
        self.node.publish_command = Mock()
        self.node.status.arming_state = VehicleStatus.ARMING_STATE_STANDBY

        self.tick_at(11.21)
        self.assertEqual(self.node.state, 'WAIT_ACTIVE')
        self.assertEqual(self.node.publish_command.call_count, 2)

        self.node.status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        self.tick_at(11.3)
        self.assertEqual(self.node.state, 'ACTIVE')
        self.tick_at(12.3)
        self.assertEqual(self.node.state, 'HOLD')
        self.assertAlmostEqual(self.node.last_point.z, -2.0)
        self.tick_at(12.4)
        final_hold = self.node.publish_setpoint.call_args.args[0]
        self.assertAlmostEqual(final_hold.z, -2.0)
        self.assertEqual(final_hold.vz, 0.0)
        self.node.request_land.assert_not_called()

    def test_stale_status_interrupts_without_publishing_more_setpoints(self):
        with patch('trajectory_follower.follow_csv.time.monotonic', return_value=12.0):
            self.node.tick()
        self.assertEqual(self.node.state, 'INTERRUPTED')
        self.node.publish_setpoint.assert_not_called()


if __name__ == '__main__':
    unittest.main()
