import time 

import rclpy 
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Pose, Twist, Accel, PointStamped, PoseArray
from dasc_msgs.msg import DIState, DIAcc, DITrajectorySimple, EwareMissionStatus, CandSetpointStatus
from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint
from nav_msgs.msg import Path
import builtin_interfaces.msg 
from visualization_msgs.msg import Marker

## python imports
import numpy as np

## Julia call
from juliacall import Main as jl

'''
Flight Preview Current Version (V1):
- The flight preview button currently verifies the ros and px4 communication stack.
- Ensures that ros communication works fine
- Expected behaviour: The quadrotor will keep on starting candidate trajectory from the same initial state

Things to test:
- Mission turn on and off tested
- Timers truned on and off tested
- Timers testing (ONG)
- Julia code to generate the nominal trajectory
'''

class EwareSetpointPubNode(Node):

    def __init__(self):
        super().__init__("eware")

        # Declare all parameters
        self.get_logger().info("Initializing params")
        self.mode = "Stopped"
        self.get_logger().info(self.mode)
        self.CandTraj_update_rate = 1.0 # Hz
        self.SetpointPub_update_rate = 50.0 # Hz
        self.discharge_rate = 0.677
        self.charge_duration = 5.0 # seconds

        

        # Declare all variables
        self.CandTraj_Timer_ = None
        self.SetpointPub_Timer_ = None
        self.CanTrajTimer_started = False
        self.SetpointPubTimer_started = False
        self.cand_precomp_done = False # Only init here; later replaced by the sub

        self.battery_SoC_cap_ = 12.0
        self.curr_SoC_ = self.battery_SoC_cap_
        self.start_time_ = None 
        self.goal_yaw = 0.0 # in the ENU frame
        self.visits = 0
        self.charge_start_time = 0.0


        # Julia
        # Creating variable storage function
        self.jlstore = jl.seval("(k, v) -> (@eval $(Symbol(k)) = $v; return)")

        jl.seval("using DifferentialEquations, LinearAlgebra, BlockDiagonals, ForwardDiff, ControlSystems, ComponentArrays, Parameters, StaticArrays, Convex, ECOS")

        # Import modules
        self.eware = jl.include("src/julia_common_exp/julia_src/Eware.jl")
        # self.dynamics = jl.include("src/julia_common_exp/julia_src/Eware.jl")
        
        # Pubs
        self.pub_px4_ = self.create_publisher(TrajectorySetpoint, "/px4_100/fmu/in/trajectory_setpoint", 10)
        self.cand_sp_ = self.create_publisher(CandSetpointStatus, "/px4_100/cand_setpoint_status", 10)

        # Subs
        
        # Mission Status sub
        self.sub_mission_status_ = self.create_subscription(
            EwareMissionStatus,
            'px4_100/gs/eware_mission_status', # Right now the name is fixed; TODO: Give as an arg
            self.mission_status_cb,
            10
        )

        # Mission Status sub
        self.sub_cand_setpoint_status_ = self.create_subscription(
            CandSetpointStatus,
            'px4_100/cand_setpoint_status', # Right now the name is fixed; TODO: Give as an arg
            self.cand_setpoint_cb,
            10
        )

        # Candidate trajectory Sub
        self.sub_cand_traj = self.create_subscription(
            DITrajectorySimple,
            'px4_100/DI_committed_traj', # Right now the name is fixed; TODO: Give as an arg
            self.cand_traj_cb,
            10
        )
        # julia params
        self.charge_pos = jl.seval("charge_pos = [0.0; 0.0; 0.3]")

        # Julia varaible storage function
        self.jlstore = jl.seval("(k, v) -> (@eval $(Symbol(k)) = $v; return)")

        self.get_logger().info("Done initializing")   

    def mission_status_cb(self, mission_status_msg):     
        if (mission_status_msg.mission_start == True) and (mission_status_msg.mission_quit == False) and (self.visits < 2):
            # Create a local variale which tracks if this code triggered CandTraj_Timer_ to start
            # Start timer for the candidate trajectory generation [Pub]
            if self.CanTrajTimer_started == False and self.SetpointPubTimer_started == False and self.cand_precomp_done == True:
                #### Start timer for the candidate trajectory generation [Pub]

                ## Start timer for the setpoint pub [local]
                self.start_timers() 
                self.CandTrajTimer_started = True
                self.SetpointPubTimer_started = True

        elif ((mission_status_msg.mission_start == False) and (mission_status_msg.mission_quit == True)) or (self.visits == 2) :
            if self.CanTrajTimer_started == True and self.SetpointPubTimer_started == True:
                self.mode = "Stopped"
                self.stop_timers()
                self.CanTrajTimer_started = False
                self.SetpointPubTimer_started == False

    def cand_setpoint_cb(self, cand_sp_msg):
        '''
        Callback to create a link between the two nodes
        '''
        self.cand_precomp_done = cand_sp_msg.cand_precomp_done

    def cand_traj_cb(self, cand_traj_msg):
        '''
        Reads the candidate trajectory from cand_traj_msg so it can be used by the Setpoint_callback()
        '''

    def start_timers(self):
       
        # First precompile the julia
        # Incase preompiling missed
        
        # Start mission and record start time
        self.mode = "Mission"
        cand_sp_msg = CandSetpointStatus()
        cand_sp_msg.cand_timer_state = True
        cand_sp_msg.mission_mode = self.mode
        
        # Store ther start time
        self.start_time_ = time.time()
        
        # Signal to EwareCandTraj Node to start the candidate trajectory node
        self.cand_sp_.publish(cand_sp_msg)
        
        # Start the trajectory setpoint timer
        self.SetpointPub_Timer_ = self.create_timer(1 / self.SetpointPub_update_rate, self.Setpoint_callback)

        self.get_logger().info('[SpNode:] Timers Started')
        self.get_logger().info("[SpNode:] Starting Mission")

    def stop_timers(self):

        # Signal to stop the candidate trajectory timer in the other node
        cand_sp_msg = CandSetpointStatus()
        cand_sp_msg.cand_timer_state = False
        self.cand_sp_.publish(cand_sp_msg)

        self.SetpointPub_Timer_.destroy()
        self.SetpointPub_Timer_ = None
        self.get_logger().info('[SpNode:] Timers stopped')
        self.get_logger().info("[SpNode:] Mission Stopped; Landing")


    def Setpoint_callback(self):
        '''
        Read the Candidate trajectory to publish the new setpoint at the current time
        '''
        self.get_logger().info('Publishing the new setpoint')
        
        # if (self.visits == 2):
        #     self.mode = "Stopped"
        #     self.stop_timers()
        #     return

        # self.t_k = self.jlstore("t_k", time.time() - self.start_time_)

        # ## Update the soc
        # if self.mode != "Charging":
        #     self.update_battery_soc()

        
        
        # self.get_logger().info(f"self.curr_SoC_: {self.curr_SoC_}")
        # # Get the new setpoint
        # # Store the trajectory internally
        # # self.committed_tks = self.jlstore("committed_tks", self.committed_tks)
        # # self.committed_xks = self.jlstore("committed_xks", self.committed_xks)
        # # self.committed_uks = self.jlstore("committed_uks", self.committed_uks)
        
        # # self.get_logger().info(f"committed_xks: {committed_xks_[1]}")

        # # Interpolate to get the setpoint
        # # Print the end time of the candidate trajectory (being printing in Julia)
        # self.get_logger().info(f"Current_time: {time.time() - self.start_time_}")
        # # self.get_logger().info("Going to interpolate")
        # di_state_sp_k, di_acc_sp_k = jl.seval("di_state_sp_k, di_acc_sp_k = Eware.DI_interpolate_traj(t_k, committed_tks_, committed_xks_, committed_uks_)")
        # # self.get_logger().info("Back from interpolate")

        # # Check if the quadrotor is landing
        # if self.mode == "Landing":
        #     ## check distance to charge_pos
        #     # self.get_logger().info("Going at_charging")
        #     at_charging = jl.seval("at_charging = Eware.reached_charging_station(state_, charge_pos)")
        #     # self.get_logger().info("Back at_charging")
        #     if (at_charging == 1):
        #         self.mode = "Charging"
        #         self.get_logger().info("Mission Stopped; Landing")
        #         self.charge_start_time = time.time()
        #         self.get_logger().info(f"Charge_start_time: {self.charge_start_time}")
        #         self.curr_SoC_ = self.battery_SoC_cap_ # Restore battery level          

        # # Publish the setpoint]
        # px4_msg = TrajectorySetpoint()
        # px4_msg.raw_mode = False;

        # # Tajectorty Setpoint in in NED frame
        # # Set pos
        # px4_msg.position[0] = di_state_sp_k[1]
        # px4_msg.position[1] = di_state_sp_k[0]
        # px4_msg.position[2] = - di_state_sp_k[2]

        # # set vel
        # px4_msg.velocity[0] = di_state_sp_k[4]
        # px4_msg.velocity[1] = di_state_sp_k[3]
        # px4_msg.velocity[2] = - di_state_sp_k[5]

        # # set acc
        # self.get_logger().info(self.mode)
        # if self.mode == "Charging":
        #     self.get_logger().info('New acc == 0')
        #     px4_msg.acceleration[1] = 0.0
        #     px4_msg.acceleration[0] = 0.0
        #     px4_msg.acceleration[2] = 0.0
        # else:
        #     self.get_logger().info('New acc == from cand')
        #     px4_msg.acceleration[1] = di_acc_sp_k[0]
        #     px4_msg.acceleration[0] = di_acc_sp_k[1]
        #     px4_msg.acceleration[2] = -di_acc_sp_k[2]
        # px4_msg.jerk[0] = 0
        # px4_msg.jerk[1] = 0
        # px4_msg.jerk[2] = 0

        # px4_msg.yaw = np.pi/2 - self.goal_yaw
        # px4_msg.yawspeed = 0.0

        # self.pub_px4_.publish(px4_msg)

        # # self.get_logger().info('Out Publishing the new setpoint')
        # # self.get_logger().info(f"Setpoint publisher callback triggered. Current time: {t_k:.2f} seconds")

    def update_battery_soc(self):
        self.curr_SoC_ = self.curr_SoC_ - (1 / self.SetpointPub_update_rate) * self.discharge_rate


def main(args = None):
    rclpy.init(args=args)
    node = EwareSetpointPubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
