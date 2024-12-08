import time 

import rclpy 
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Pose, Twist, Accel, PointStamped, PoseArray
from dasc_msgs.msg import DIState, DIAcc, DITrajectorySimple, EwareMissionStatus, CandToSp, SpToCand
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

class EwareCandTrajNode(Node):

    def __init__(self):
        super().__init__("eware_cand_node")

        # Declare all parameters
        self.get_logger().info("Initializing Candidate Trajectory Node AGain")
        self.mode = "Stopped"
        self.CandTraj_update_rate = 1.0 # Hz
        self.discharge_rate = 0.677
        self.charge_duration = 5.0 # seconds

        self.cand_timer_state = None        
        self.cand_precomp_done = False
        self.just_charged = False
        # Declare all variables
        self.CandTraj_Timer_ = None
        self.battery_SoC_cap_ = 15.0
        self.curr_soc_state = self.battery_SoC_cap_
        self.get_logger().info(f"Curr_soc_state: {self.curr_soc_state}")

        # self.curr_SoC_ = self.battery_SoC_cap_
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
        self.pub_traj_ = self.create_publisher(DITrajectorySimple, "/px4_100/DI_committed_traj", 10)
        self.pub_CandToSp = self.create_publisher(CandToSp, "/px4_100/CandToSp", 10)
        # Subs

        
        # Vehicle Position sub
        self.sub_state_ = self.create_subscription( 
                VehicleLocalPosition,
                'px4_100/fmu/out/vehicle_local_position',
                self.state_callback,
                qos_profile_sensor_data)

        # Mission Status sub
        self.sub_mission_status_ = self.create_subscription(
            EwareMissionStatus,
            '/px4_100/gs/eware_mission_status', # Right now the name is fixed; TODO: Give as an arg
            self.mission_status_cb,
            10
        )

        # Message from Sp Node sub
        self.sub_cand_setpoint_status_ = self.create_subscription(
            SpToCand,
            '/px4_100/SpToCand', # Right now the name is fixed; TODO: Give as an arg
            self.SpToCand_cb,
            10
        )

        # Create a message instance and reuse it 
        self.CandToSp  = CandToSp()
        self.DITraj_msg = DITrajectorySimple()

        # julia params
        self.charge_pos = jl.seval("charge_pos = [2.525; 2.992; 0.880]")

        # Julia varaible storage function
        self.jlstore = jl.seval("(k, v) -> (@eval $(Symbol(k)) = $v; return)")

        self.get_logger().info("Done initializing")

    def state_callback(self, state_msg):

        if not (state_msg.xy_valid  and state_msg.z_valid  and state_msg.heading_good_for_control):
            return
        ## Vehicle local position provides 
        # self.state = np.array([
        #     state_msg.y,
        #     state_msg.x,
        #     -state_msg.z,
        #     state_msg.vy,
        #     state_msg.vx,
        #     -state_msg.vz,
        #     state_msg.ay,
        #     state_msg.ax,
        #     -state_msg.az])

        # self.state_yaw = np.pi/2 - state_msg.heading # in ENU coords
        # self.get_logger().info("Getting new state")
        self.state_ = self.jlstore("state_", [
                                    state_msg.y, 
                                    state_msg.x, 
                                    -state_msg.z,
                                    state_msg.vy,
                                    state_msg.vx,
                                    -state_msg.vz,
                                    state_msg.ay,
                                    state_msg.ax,
                                    -state_msg.az])

        self.state_yaw_ = self.jlstore("state_yaw", np.pi/2 - state_msg.heading)


    def mission_status_cb(self, mission_status_msg):
        # self.get_logger().info("Checking msg")
        if (self.cand_precomp_done == False):
            # First precompile the julia
            self.get_logger().info("Starting Precompiling")
            self.CandTraj_callback("Precompile")
            self.get_logger().info("Finished Precompiling")
            self.cand_precomp_done = True

            # Signal tp EwareSpPub Node for finishing precompilation
            self.CandToSp.cand_precomp_done = self.cand_precomp_done
            self.pub_CandToSp.publish(self.CandToSp)

        # If the precompilation is done and signal is received from the setpoint node to start the timer, start it
        elif (mission_status_msg.mission_start == True) and (mission_status_msg.mission_quit == False) and (self.cand_timer_state == True):
            # If the timers aren't already running, start them
            if self.CandTraj_Timer_ is None:
                self.start_timers()
        elif ((mission_status_msg.mission_start == False) and (mission_status_msg.mission_quit == True)) or (self.cand_timer_state == False):
            if self.CandTraj_Timer_ is not None:
                self.stop_timers()

    def SpToCand_cb(self, cand_sp_msg):
        self.cand_timer_state = cand_sp_msg.cand_timer_state
        self.sp_mode = cand_sp_msg.sp_mode
        self.curr_soc_state = cand_sp_msg.battery_soc_state

    def start_timers(self):
       
        # Start mission and record start time
        self.start_time_ = time.time()
        self.CandTraj_Timer_ = self.create_timer(1 / self.CandTraj_update_rate, self.CandTraj_callback)
        self.mode = "Mission"
        self.CandToSp.cand_mode = self.mode
        self.pub_CandToSp.publish(self.CandToSp)
        # Immediately call the callbacks for time = 0
        # self.CandTraj_callback()

    def stop_timers(self):
        self.CandTraj_Timer_.destroy()
        self.CandTraj_Timer_ = None
        self.get_logger().info('CandGen Timer stopped')


    def CandTraj_callback(self, stage = "Mission"):
        # self.get_logger().info('Generating new Candidate Trajectory')
        # self.get_logger().info(f"self.di_xs: {self.di_xs}")
        self.get_logger().info('Generating new Candidate Trajectory')
        '''
        Forward integration of the trajectory
        1. Get the quadrotor current state (p, v, a, yaw)
        2. Convert that into quadrotor states for forward integration
        '''
        
        if stage == "Precompile":
            # Precompile and commit the first trajectory
            self.get_logger().info('First Trajectory Committed') 
            self.get_logger().info(f"Curr_soc_state: {self.curr_soc_state}")
            pre_t_k = self.jlstore("pre_t_k", 0.0)
            pre_flat_states_k = jl.seval("pre_flat_states_k = [state_; state_yaw]")
            pre_battery_soc_k = self.jlstore("pre_battery_soc_k", self.curr_soc_state)
            pre_discharge_rate = self.jlstore("pre_discharge_rate", self.discharge_rate)
            pre_di_ts, pre_di_xs, pre_di_us, pre_can_soc_end = jl.seval("pre_di_ts, pre_di_xs, pre_di_us, pre_can_soc_end = Eware.generate_candidate_traj(pre_t_k, pre_flat_states_k, pre_battery_soc_k, pre_discharge_rate, charge_pos)")
            
            # Publish the first candidate trajectory
            # msg.header.stamp = rclpy.Time.now()
            self.DITraj_msg.header.frame_id = "vicon/world"
            self.DITraj_msg.dt = 1.0 / self.CandTraj_update_rate

            self.DITraj_msg.times = pre_di_ts

            for i in range(len(pre_di_xs)):
                di_state = DIState()
                di_state.state = pre_di_xs[i]
                self.DITraj_msg.states.append(di_state)

            for i in range(len(pre_di_us)):
                di_acc = DIAcc()
                di_acc.acceleration = pre_di_us[i]
                self.DITraj_msg.accelerations.append(di_acc) 

            self.pub_traj_.publish(self.DITraj_msg)
            self.get_logger().info('First Trajectory Committed')            
            
            return 
        
        # If the quad has charged and sp has moved into mission mode start publishing trajectories
        if self.mode != "Mission" and self.sp_mode == "Charged":
            self.mode = "Mission"
            self.just_charged = True
        
        # self.get_logger().info(f"self.mode: {self.mode}")
        if self.mode != "Mission":
            return


        # self.get_logger().info('Generating new Candidate Trajectory')
        # Candidate Trajectory Replanning

        # Current time
        # self.get_logger().info(f"Curr_soc_state: {self.curr_soc_state}")
        self.t_k = self.jlstore("t_k", time.time() - self.start_time_)
        self.flat_states_k = jl.seval("flat_states_k = [state_; state_yaw]")
        self.battery_soc_k = self.jlstore("battery_soc_k", self.curr_soc_state)
        self.discharge_rate_jl = self.jlstore("discharge_rate_jl", self.discharge_rate)
        self.di_ts, self.di_xs, self.di_us, self.can_soc_end = jl.seval("di_ts, di_xs, di_us, can_soc_end = Eware.generate_candidate_traj(t_k, flat_states_k, battery_soc_k, discharge_rate_jl, charge_pos)")
        

        ## Keep the copy of the committed trajectory internally and also publish it for possible plotting
        # self.get_logger().info("Updating soc")
        self.get_logger().info(f"can_soc_end: {self.can_soc_end}")
        if self.can_soc_end > 0.0:

            # For now validate the candidate trajectory right now
            DITraj_msg = DITrajectorySimple()
            # msg.header.stamp = rclpy.Time.now()
            DITraj_msg.header.frame_id = "vicon/world"
            DITraj_msg.dt = 1.0 / self.CandTraj_update_rate

            DITraj_msg.times = self.di_ts

            for i in range(len(self.di_xs)):
                di_state = DIState()
                di_state.state = self.di_xs[i]
                DITraj_msg.states.append(di_state)

            for i in range(len(self.di_us)):
                di_acc = DIAcc()
                di_acc.acceleration = self.di_us[i]
                DITraj_msg.accelerations.append(di_acc) 

            self.pub_traj_.publish(DITraj_msg)
            # self.get_logger().info('Trajectory Committed')

            if self.just_charged == True:
                self.CandToSp.cand_mode = self.mode
                self.pub_CandToSp.publish(self.CandToSp)
                self.just_charged = False
                
        else:
            # self.get_logger().info('Not committed; Quad returning')
            self.mode = "Landing"
            self.CandToSp.cand_mode = self.mode
            self.pub_CandToSp.publish(self.CandToSp)
            
        
        # self.get_logger().info('Out Generating new Candidate Trajectory')



def main(args = None):
    rclpy.init(args=args)
    node = EwareCandTrajNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
