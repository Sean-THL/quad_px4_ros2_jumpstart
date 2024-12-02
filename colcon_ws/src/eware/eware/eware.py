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


## Julia call
from juliacall import Main as jl


'''
Flight Preview Current Version (V1):
- The flight preview button currently verifies the ros and px4 communication stack.
- Ensures that ros communication works fine
- Expected behaviour: The quadrotor will keep on starting candidate trajectory from the same initial state

'''

class Eware(Node):

    def __init__(self):
        super().__init__("eware")

        # Declare all parameters
        self.get_logger().info("Initializing params")
        self.mode = "Stopped"
        self.CandTraj_update_rate = 1.0 # Hz
        self.SetpointPub_update_rate = 20.0 # Hz
        self.discharge_rate = -0.677

        # Declare all variables
        self.CandTraj_Timer_ = None
        self.SetpointPub_Timer_ = None
        self.curr_battery_SoC_ = 60.0
        self.curr_SoC_ = self.curr_battery_SoC_

        # Pubs

        # Subs
        self.sub_mission_status_ = self.create_subscription(
            EwareMissionStatus,
            'px4_100/gs/eware_mission_status', # Right now the name is fixed; TODO: Give as an arg
            self.mission_status,
            10
        )

    def mission_status(self, mission_status_msg):
        # self.get_logger().info("Checking msg")
        if (mission_status_msg.mission_start == True) and (mission_status_msg.mission_quit == False):
            # If the timers aren't already running, start them
            if self.CandTraj_Timer_ is None and self.SetpointPub_Timer_ is None:
                self.start_timers()
                self.mode = "Mission"
        elif (mission_status_msg.mission_start == False) and (mission_status_msg.mission_quit == True):
            if self.CandTraj_Timer_ is not None and self.SetpointPub_Timer_ is not None:
                self.mode = "Stopped"
                self.stop_timers()


    def start_timers(self):
        self.CandTraj_Timer_ = self.create_timer(1 / self.CandTraj_update_rate, self.CandTraj_callback)
        self.SetpointPub_Timer_ = self.create_timer(1 / self.SetpointPub_update_rate, self.Setpoint_callback)
        self.get_logger().info('Timers Started')
        self.get_logger().info("Starting Mission")

    def stop_timers(self):
        self.CandTraj_Timer_.destroy()
        self.CandTraj_Timer_ = None
        self.SetpointPub_Timer_.destroy()
        self.SetpointPub_Timer_ = None
        self.get_logger().info('Timers stopped')
        self.get_logger().info("Mission Stopped; Landing")

    def CandTraj_callback(self):
        self.get_logger().info('Generating new Candidate Trajectory')

        '''
        Forward integration of the trajectory
        1. Get the quadrotor current state (p, v, a, yaw)
        2. Convert that into quadrotor states for forward integration
        '''

        ## TODO:
        # Send
        # 1. Current time since the timer CandTraj_Timer started
        # 2. Current SoC
        # 2. Current state of the quadrotor 
        # 3. Discharge rate
        # 4. 
        # Receive
        # 1. The Cand Trajectory
        # 2. The SoC at the end of the candidate trajectory

        ## Validate the candidate trajectory
        # 1. Send the cand_SoC_end for validation at the nominal trajectory
        # 2. If the service returns true -> commit the trajectory
        # 
        # valid = False
        # if (valid == True):
        #     self.get_logger().info('New Candidate Trajectory Committed')
        # else:
        #     self.get_logger().info('NOT COMMITTED! QUAD RETURNING FOR CHARGING')


    def Setpoint_callback(self):
        '''
        Get the current DI reference state and control input 
        1. If the quadrotor is in the landing mode check if it has reached charging station
            - self.at_charging = true if reached
        2. If the quadrotor has landed 
            - self.charge_start_time = current_time
        3. if self.at_charging == true
            - self.landed = 1
        4. Get the DI trajectory setpoint (px, py, pz, vx, vy, vz, ax, ay, az)
        5. if self.landed == 1
            - set (ax, ay, az) == (0, 0, 0)
            - Ensures that quad stays there
        6. Publish the setpoint
        7. RK4 on the battery_dynamics to update self.curr_SoC_ // Make sure to match the frequency
        '''
        self.get_logger().info('Publishing the new setpoint')



def main(args = None):
    rclpy.init(args=args)
    node = Eware()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
