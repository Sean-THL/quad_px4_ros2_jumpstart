import time 

import rclpy 
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Pose, Twist, Accel, PointStamped, PoseArray
from dasc_msgs.msg import DIState, DIAcc, DITrajectorySimple, EwareMissionStatus
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

class EwareNode(Node):

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
        self.start_time_ = None 
        self.goal_yaw = 0.0 # in the ENU frame



        # Committed trajectory init
        # self.committed_tks = None
        # self.committed_xks = None
        # self.committed_uks = None

        # Julia
        # Creating variable storage function
        self.jlstore = jl.seval("(k, v) -> (@eval $(Symbol(k)) = $v; return)")

        jl.seval("using DifferentialEquations, LinearAlgebra, BlockDiagonals, ForwardDiff, ControlSystems, ComponentArrays, Parameters, StaticArrays, Convex, ECOS")

        # Import modules
        self.eware = jl.include("src/julia_common_exp/julia_src/Eware.jl")
        # self.dynamics = jl.include("src/julia_common_exp/julia_src/Eware.jl")
        
        # Pubs
        self.pub_traj_ = self.create_publisher(DITrajectorySimple, "px4_100/DI_committed_traj", 10)
        self.pub_px4_ = self.create_publisher(TrajectorySetpoint, "/px4_100/fmu/in/trajectory_setpoint", 10)


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
            'px4_100/gs/eware_mission_status', # Right now the name is fixed; TODO: Give as an arg
            self.mission_status_cb,
            10
        )

        # self

        # julia imports and imports

        # Julia varaible storage function
        self.jlstore = jl.seval("(k, v) -> (@eval $(Symbol(k)) = $v; return)")

        self.get_logger().info("Done initializing")   

    def mission_status_cb(self, mission_status_msg):
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
       
        # First precompile the julia
        self.get_logger().info("Starting Precompiling")
        self.CandTraj_callback("Precompile")
        self.get_logger().info("Finished Precompiling")
        
        # Start mission and record start time
        self.start_time_ = time.time()
        self.CandTraj_Timer_ = self.create_timer(1 / self.CandTraj_update_rate, self.CandTraj_callback)
        self.SetpointPub_Timer_ = self.create_timer(1 / self.SetpointPub_update_rate, self.Setpoint_callback)
        self.get_logger().info('Timers Started')
                # Immediately call the callbacks for time = 0
        self.CandTraj_callback()
        self.Setpoint_callback()
        self.get_logger().info("Starting Mission")

    def stop_timers(self):
        self.CandTraj_Timer_.destroy()
        self.CandTraj_Timer_ = None
        self.SetpointPub_Timer_.destroy()
        self.SetpointPub_Timer_ = None
        self.get_logger().info('Timers stopped')
        self.get_logger().info("Mission Stopped; Landing")


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

        # self.get_logger().info(f"Current elevation: {self.state[6]:.2f} m")

        # self.state_initialized = True
        
        # if self.use_sim_time:
        #     unix_time_ns = time.time_ns()
        #     lag_ns = unix_time_ns - (state_msg.timestamp_sample * 1000)
        # else:
        #     time_ns = self.get_clock().now().nanoseconds # when using gazebo, the publish rate is attrocious
        #     lag_ns = time_ns - (state_msg.timestamp_sample*1000)

        # now_ns  = self.get_clock().now().nanoseconds

        # extra_lag_ms = 0*1000.0/self.traj_rate_hz # i.e. a skip_idx = 1

        # msg_ns = now_ns - lag_ns - extra_lag_ms * 10**6

        # shifted_s = int(msg_ns // 10**9)
        # shifted_ns = int(msg_ns % 10**9)

        # self.state_stamp = builtin_interfaces.msg.Time(sec = shifted_s, nanosec=shifted_ns)




    def CandTraj_callback(self, mode = "Mission"):
        # self.get_logger().info('Generating new Candidate Trajectory')

        '''
        Forward integration of the trajectory
        1. Get the quadrotor current state (p, v, a, yaw)
        2. Convert that into quadrotor states for forward integration
        '''
        
        if mode == "Precompile":
            pre_t_k = self.jlstore("pre_t_k", 0.0)
            pre_flat_states_k = jl.seval("pre_flat_states_k = [state_; state_yaw]")
            pre_battery_soc_k = self.jlstore("pre_battery_soc_k", 60.0)
            pre_discharge_rate = self.jlstore("pre_discharge_rate", 0.6677)
            pre_charge_pos = jl.seval("pre_charge_pos = [0.; 0; 0]")
            pre_di_ts, pre_di_xs, pre_di_us, pre_can_soc_end = jl.seval("pre_di_ts, pre_di_xs, pre_di_us, pre_can_soc_end = Eware.generate_candidate_traj(pre_t_k, pre_flat_states_k, pre_battery_soc_k, pre_discharge_rate, pre_charge_pos)")
        else:
            # Current time
            self.t_k = self.jlstore("t_k", time.time() - self.start_time_)
            self.flat_states_k = jl.seval("flat_states_k = [state_; state_yaw]")
            self.battery_soc_k = self.jlstore("battery_soc_k", 60.0)
            self.discharge_rate = self.jlstore("discharge_rate", 0.6677)
            self.charge_pos = jl.seval("charge_pos = [0.; 0; 0]")
            self.di_ts, self.di_xs, self.di_us, self.can_soc_end = jl.seval("di_ts, di_xs, di_us, can_soc_end = Eware.generate_candidate_traj(t_k, flat_states_k, battery_soc_k, discharge_rate, charge_pos)")
            
            # self.get_logger().info(f"self.di_xs: {self.di_xs}")
            # # self.get_logger().info(f"di_xs: {self.di_xs[1]}")
            # self.get_logger().info(f"len_di_ts: {len(self.di_ts)}")
            # self.get_logger().info(f"len_di_xs: {len(self.di_xs)}")
            # self.get_logger().info(f"len_di_xs[1]: {len(self.di_xs[1])}")
            # self.get_logger().info(f"len_di_us: {len(self.di_us)}")
            # self.get_logger().info(f"len_di_us[1]: {len(self.di_us[1])}")
            # self.get_logger().info(f"can_soc_end: {self.can_soc_end}")

            # length_di_ts: 199
            # length_di_xs: 199
            # length_di_us: 198
            # length_di_xs[1]: 6
            # length_di_us[1]: 3


            ## Keep the copy of the committed trajectory internally and also publish it for possible plotting
            if self.can_soc_end > 0.0:
                

                # # Store the trajectory internally
                # self.committed_tks = self.di_ts
                # self.committed_xks = self.di_xs
                # self.committed_uks = self.di_us
                self.committed_tks_ = self.jlstore("committed_tks_", self.di_ts)
                self.committed_xks_ = self.jlstore("committed_xks_", self.di_xs)
                self.committed_uks_ = self.jlstore("committed_uks_", self.di_us)

                # self.get_logger().info(f"committed_xks_cand: {self.committed_xks_}")

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
                self.get_logger().info('Trajectory Committed')
            else:
                self.get_logger().info('Not committed; Quad returning')
        


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
        # self.get_logger().info('Publishing the new setpoint')
        self.t_k = self.jlstore("t_k", time.time() - self.start_time_)

        # Get the new setpoint
        # Store the trajectory internally
        # self.committed_tks = self.jlstore("committed_tks", self.committed_tks)
        # self.committed_xks = self.jlstore("committed_xks", self.committed_xks)
        # self.committed_uks = self.jlstore("committed_uks", self.committed_uks)
        
        # self.get_logger().info(f"committed_xks: {committed_xks_[1]}")

        # Interpolate to get the setpoint
        di_state_sp_k, di_acc_sp_k = jl.seval("di_state_sp_k, di_acc_sp_k = Eware.DI_interpolate_traj(t_k, committed_tks_, committed_xks_, committed_uks_)")


        # Publish the setpoint]
        px4_msg = TrajectorySetpoint()
        px4_msg.raw_mode = False;

        # Tajectorty Setpoint in in NED frame
        # Set pos
        px4_msg.position[0] = di_state_sp_k[1]
        px4_msg.position[1] = di_state_sp_k[0]
        px4_msg.position[2] = - di_state_sp_k[2]

        # set vel
        px4_msg.velocity[0] = di_state_sp_k[4]
        px4_msg.velocity[1] = di_state_sp_k[3]
        px4_msg.velocity[2] = - di_state_sp_k[5]

        # set acc
        px4_msg.acceleration[1] = di_acc_sp_k[0]
        px4_msg.acceleration[0] = di_acc_sp_k[1]
        px4_msg.acceleration[2] = -di_acc_sp_k[2]
        px4_msg.jerk[0] = 0
        px4_msg.jerk[1] = 0
        px4_msg.jerk[2] = 0

        px4_msg.yaw = np.pi/2 - self.goal_yaw
        px4_msg.yawspeed = 0.0

        self.pub_px4_.publish(px4_msg)

        # self.get_logger().info(f"Setpoint publisher callback triggered. Current time: {t_k:.2f} seconds")



def main(args = None):
    rclpy.init(args=args)
    node = EwareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
