import time 

import rclpy 
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Pose, Twist, Accel, PointStamped, PoseArray
from dasc_msgs.msg import DITrajectory, EwareMissionStatus
from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint
from nav_msgs.msg import Path
import builtin_interfaces.msg 
from visualization_msgs.msg import Marker


class Eware(Node):

    def __init__(self):
        super().__init__("eware")

        # Declare all parameters
        self.get_logger().info("Initializing params")
        self.mode = "Stopped"
        self.CandTraj_update_rate = 1.0 # Hz
        self.SetpointPub_update_rate = 20.0 # Hz

        # Declare all variables
        self.CandTraj_Timer_ = None
        self.SetpointPub_Timer_ = None

        # Pubs

        # Subs
        self.sub_mission_status_ = self.create_subscription(
            EwareMissionStatus,
            'px4_100/gs/eware_mission_status', # Right now the name is fixed; TODO: Give as an arg
            self.mission_status,
            10
        )

    def mission_status(self, mission_status_msg):
        self.get_logger().info("Checking msg")
        if (mission_status_msg.mission_start == True) and (mission_status_msg.mission_quit == False):
            self.start_timers()
            self.mode = "Mission"
            self.get_logger().info("Starting Mission")
        elif (mission_status_msg.mission_start == False) and (mission_status_msg.mission_quit == True):
            self.mode = "Stopped"
            self.stop_timers()
            self.get_logger().info("Mission Stopped; Landing")


    def start_timers(self):
        # If the timers aren't already running, start them
        if self.CandTraj_Timer_ is None:
            self.CandTraj_Timer_ = self.create_timer(1 / self.CandTraj_update_rate, self.CandTraj_callback)
        if self.SetpointPub_Timer_ is None:
            self.SetpointPub_Timer_ = self.create_timer(1 / self.SetpointPub_update_rate, self.Setpoint_callback)
        self.get_logger().info('Timers Started')


    def stop_timers(self):
        # if the timers aren't already stopped, stop them
        if self.CandTraj_Timer_ is not None:
            self.CandTraj_Timer_.destroy()
            self.CandTraj_Timer_ = None
        if self.SetpointPub_Timer_ is not None:
            self.SetpointPub_Timer_.destroy()
            self.SetpointPub_Timer_ = None
        self.get_logger().info('Timers stopped')



    def CandTraj_callback(self):
        self.get_logger().info('Generating new Candidate Trajectory')

    def Setpoint_callback(self):
        # TODO: Remove this print statement; excessive printing
        self.get_logger().info('Publishing new setpoint')


def main(args = None):
    rclpy.init(args=args)
    node = Eware()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
