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
        self.mode = "Grounded"
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
            # If the timers aren't already running, start them
            if self.CandTraj_Timer_ is None and self.SetpointPub_Timer_ is None:
                self.start_timers()
                self.mode = "Mission"
                self.get_logger().info("Starting Mission")
        # elif (mission_status_msg.mission_start == True) and (mission_status_msg.mission_quit == True):
        #     ## Stop the timer and and move the quadrotor to the landing mode
        #     self.mode = "Stopped"
        elif (mission_status_msg.mission_start == False) and (mission_status_msg.mission_quit == True):
            self.mode = "Stop"
            ## TODO: Stop the timer and move to land mode
        else:
            self.get_logger().info("Already at Ground")

    def start_timers(self):
        self.CandTraj_Timer_ = self.create_timer(1 / self.CandTraj_update_rate, self.CandTraj_callback)
        self.SetpointPub_Timer_ = self.create_timer(1 / self.SetpointPub_update_rate, self.Setpoint_callback)
        self.get_logger().info('Timers Started')


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
