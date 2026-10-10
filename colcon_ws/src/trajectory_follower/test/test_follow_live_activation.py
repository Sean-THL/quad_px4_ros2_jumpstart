"""Check the custom commander state used for live flight activation."""

import unittest
from unittest.mock import Mock

from px4_msgs.msg import (CommanderSetState, CommanderStatus,
                          VehicleLocalPosition, VehicleStatus)

from trajectory_follower.follow_live import LiveTrajectoryFollower
from trajectory_follower.live_trajectory import TrajectoryConfig


class LiveActivationTest(unittest.TestCase):
    def setUp(self):
        self.node = object.__new__(LiveTrajectoryFollower)
        self.node.state = 'IDLE'
        self.node.position = VehicleLocalPosition()
        self.node.position.x = 1.0
        self.node.position.y = 2.0
        self.node.position.z = -0.1
        self.node.position.v_xy_valid = True
        self.node.position.v_z_valid = True
        self.node.position_received_at = 10.0
        self.node.status = VehicleStatus()
        self.node.status.system_id = 0
        self.node.status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.node.status_received_at = 10.0
        self.node.commander_status = CommanderStatus()
        self.node.commander_status.state = CommanderStatus.STATE_DISARMED
        self.node.commander_status_received_at = 10.0
        self.node.target_system = 201
        self.node.allow_unknown_system_id = True
        self.node.auto_arm_offboard = True
        self.node.auto_command_interface = 'commander_set_state'
        self.node.publish_commander_state = Mock()
        self.node.publish_command = Mock()
        self.node.last_command_at = 0.0
        self.node.origin = (1.0, 2.0, -0.1)
        self.node.start_yaw = 0.3
        self.node.commanded_yaw = 0.3
        self.node.config = TrajectoryConfig(
            takeoff_profile='prearmed_hover', takeoff_target_z_m=-0.7,
            takeoff_duration_s=3.0)
        self.node.get_logger = Mock(return_value=Mock())

    def test_disarmed_commander_overrides_false_vehicle_offboard(self):
        self.assertIsNone(self.node._start_error(10.0))
        self.assertFalse(self.node._is_armed_offboard(10.0))

    def test_custom_auto_commands_arm_then_offboard(self):
        self.node.state = 'WAIT_ACTIVE'
        self.node._maybe_send_start_commands(10.0)
        self.node.publish_commander_state.assert_called_once_with(
            CommanderSetState.STATE_ARMED)
        self.node.publish_command.assert_not_called()

        self.node.commander_status.state = CommanderStatus.STATE_ARMED
        self.node.commander_status_received_at = 11.0
        self.node._maybe_send_start_commands(11.0)
        self.assertEqual(self.node.publish_commander_state.call_args.args,
                         (CommanderSetState.STATE_OFFBOARD,))

        self.node.commander_status.state = CommanderStatus.STATE_OFFBOARD
        self.node.commander_status_received_at = 11.1
        self.assertTrue(self.node._is_armed_offboard(11.1))

    def test_manual_mode_never_sends_arm_or_offboard(self):
        self.node.state = 'WAIT_ACTIVE'
        self.node.auto_arm_offboard = False
        self.node._maybe_send_start_commands(10.0)
        self.node.publish_commander_state.assert_not_called()
        self.node.publish_command.assert_not_called()

    def test_stale_commander_cannot_confirm_activation(self):
        self.node.commander_status.state = CommanderStatus.STATE_OFFBOARD
        self.assertFalse(self.node._is_armed_offboard(12.0))

    def test_prearmed_hover_is_published_like_gui(self):
        self.node.setpoint_publisher = Mock()
        self.node.mode_publisher = Mock()
        self.node.publish_mode()
        self.node.publish_setpoint(self.node._hold_state())
        msg = self.node.setpoint_publisher.publish.call_args.args[0]
        for actual, expected in zip(msg.position, (1.0, 2.0, -0.7)):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertEqual(tuple(msg.velocity), (0.0, 0.0, 0.0))
        self.assertEqual(tuple(msg.acceleration), (0.0, 0.0, 0.0))
        self.assertEqual(tuple(msg.jerk), (0.0, 0.0, 0.0))
        self.assertEqual(msg.timestamp, 0)
        self.assertFalse(msg.raw_mode)
        self.node.mode_publisher.publish.assert_not_called()

    def test_prearmed_target_must_be_above_current_local_z(self):
        self.node.position.z = -0.8
        self.node.position_received_at = 10.0
        self.node.status_received_at = 10.0
        self.node.commander_status_received_at = 10.0
        with unittest.mock.patch(
                'trajectory_follower.follow_live.time.monotonic',
                return_value=10.0):
            response = self.node.start_callback(None, Mock(success=False))
        self.assertFalse(response.success)
        self.assertIn('not above current local Z', response.message)
        self.assertEqual(self.node.state, 'IDLE')

    def test_path_waits_for_stable_takeoff_hover(self):
        self.node.state = 'TAKEOFF_HOLD'
        self.node.takeoff_started_at = 10.0
        self.node.takeoff_ready_at = 0.0
        self.node._update_takeoff_hold(13.0)
        self.assertEqual(self.node.state, 'TAKEOFF_HOLD')

        self.node.position.z = -0.7
        self.node._update_takeoff_hold(13.1)
        self.assertEqual(self.node.state, 'TAKEOFF_HOLD')
        self.node._update_takeoff_hold(13.61)
        self.assertEqual(self.node.state, 'ACTIVE')
        self.assertAlmostEqual(self.node.flight_started_at, 10.61)

    def test_takeoff_timeout_holds_instead_of_starting_path(self):
        self.node.state = 'TAKEOFF_HOLD'
        self.node.takeoff_started_at = 10.0
        self.node.takeoff_ready_at = 0.0
        self.node._update_takeoff_hold(23.01)
        self.assertEqual(self.node.state, 'TAKEOFF_FAILED')


if __name__ == '__main__':
    unittest.main()
