import time
import traceback

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter

from builtin_interfaces.msg import Time
import rclpy
import rclpy.clock
import rclpy.duration
from rclpy.executors import ExternalShutdownException
import rclpy.logging
from rclpy.node import Node
from pi3hat_moteus_int_msgs.msg import JointsCommand, JointsStates
from sensor_msgs.msg import JointState
import example_robot_data
import pinocchio
import crocoddyl
from termcolor import colored

class DDP_Controller(Node):
    '''
    This class creates a ROS2 node that is used to publish the joint states from the DDP controller to the robot.
    Next, it pusblishes positions, velocities and torques to the robot.
    Optionally it allows to implement the ILC controller.
    '''
    def __init__(self):
        '''
        Initialize the node and the parameters.
        '''
        super().__init__('ddp_controller_publisher')
        
        # ============================ Parameters ============================ #
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('csv_filename', 'big_data_vince_0.47'), # X init
                ('parent_folder', 'step_jump'),
                ('rate', 1.),
                ('topic_name', '/PD_control/command'),
                ('simulation', True),
                ('ilc', False),
                ('iter_number', 1),
                ('ilc_Kp', 0.5),
                ('ilc_Kd', 0.05),
                ('homing_duration', 5.0),
                ('robot_name', 'mulinex12'),                
                ('K', 500.),
                ('config', 'standing'),
                ('iter_ddp', 200),
                ('thres_ddp', 1e-2),
            ]
        )
                
        
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        simulation = self.get_parameter('simulation').get_parameter_value().bool_value
        
        self.ilc = self.get_parameter('ilc').get_parameter_value().bool_value
        self.iter_number = self.get_parameter('iter_number').get_parameter_value().integer_value
        self.ilc_Kp = self.get_parameter('ilc_Kp').get_parameter_value().double_value
        self.ilc_Kd = self.get_parameter('ilc_Kd').get_parameter_value().double_value
        
        self.homing_duration = self.get_parameter('homing_duration').get_parameter_value().double_value
        
        self.stiffness = self.get_parameter('K').get_parameter_value().double_value
        self.config = self.get_parameter('config').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        
        self.iter_ddp = self.get_parameter('iter_ddp').get_parameter_value().integer_value
        self.thres_ddp = self.get_parameter('thres_ddp').get_parameter_value().double_value
              
        # Check the correctness of the topics
        sim_condition = (simulation == True) and (topic_name == '/PD_control/command')
        real_condition = (simulation == False) and (topic_name == '/joint_controller/command')
        
        if sim_condition == False and real_condition == False:
            raise KeyboardInterrupt('Wrong Simulation flag or topic name')
    
        if self.robot_name != 'mulinex' and self.robot_name != 'mulinex12':
            raise KeyboardInterrupt('This Node just works with Mulinex or Mulinex12 ... Wrong robot name')
            
        # ============================ Publishers ============================ #
        
        if simulation:
            self.joint_states_msg = JointState()
            self.joint_target_pos_topic = topic_name
            self.pub_joint_states = self.create_publisher(JointState, self.joint_target_pos_topic, 10)
        else:
            self.joint_states_msg = JointsCommand()
            self.joint_target_pos_topic = topic_name
            self.pub_joint_states = self.create_publisher(JointsCommand, self.joint_target_pos_topic, 10)      
              
        # ============================ Variables ============================ #
        
        self.big_data, self.data_q_ddp, self.data_q_vel_ddp, self.data_ff_ddp, self.data_fb_ddp = [], [], [], [], []
                        
        self.homing = False
        self.i = 0  # index for the commanded joint states
        self.init_counter = 0
        self.iter = 0
        self.j = 0  # index for the measured joint states
        
        self.timer_period = 0.002  # seconds
        self.time = 0
        
        self.joint_names_x = ["LF_HFE",
                            "LF_KFE",
                            "LH_HFE",
                            "LH_KFE",
                            "RF_HFE",
                            "RF_KFE",
                            "RH_HFE",
                            "RH_KFE"]
    
        
        self.joint_names = self.joint_names_x
                    
        self.jnt_num = 8
        self.simulation = simulation
        self.topic_name = topic_name        
        self.nan_counter = 0
        
        # ============================ DDP optimization ============================ #
        
        self.ddp_controller()
                
        # ============================ Extract the reference joint states ============================ #
        
        self.reference_extract()
        
        # ============================ Subscribers ============================ #
        
        self.joint_position_meas = np.zeros((self.jnt_num, self.points, self.iter_number))
        self.joint_velocity_meas = np.zeros((self.jnt_num, self.points, self.iter_number))
        self.joint_effort_meas = np.zeros((self.jnt_num, self.points, self.iter_number))
        
        if self.simulation:
            self.joint_state_topic = '/joint_states'
            self.joint_sub = self.create_subscription(JointState, self.joint_state_topic, self.read_joint_states, 10)
        else:
            self.joint_state_topic = '/state_broadcaster/joints_state'
            self.joint_sub = self.create_subscription(JointsStates, self.joint_state_topic, self.read_joint_states, 10)
        
        # ============================ Publish the joint states ============================ #
        
        self.timer = self.create_timer(self.timer_period, self.publish_joint_states)
    
    def ddp_controller(self):
        '''Implement the DDP controller.'''
        self.mulinex = example_robot_data.load(self.robot_name)

        q0 = self.mulinex.model.referenceConfigurations[self.config].copy()
        self.mulinex.q0 = q0
        v0 = pinocchio.utils.zero(self.mulinex.model.nv)
        x0 = np.concatenate([q0, v0])
        
        lfFoot, rfFoot, lhFoot, rhFoot = "LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"
        
        if self.robot_name == 'mulinex12':
            nA = 12
            self.K = self.stiffness * np.eye(nA)
            gait = SimpleMulinexGaitProblem(self.mulinex.model, lfFoot, rfFoot, lhFoot, rhFoot, self.K, x0, nA=nA, robot_name=self.robot_name, node='ddp')
            
            GAITPHASES = [{"com_move_mp": {"comGoTo": 0.5, 'versor': [1.0, 0., 0.], "timeStep": self.timer_period, "numKnots": 140, "stepLength":0.05, "stepHeight":0.03}}]
        
        else:
            self.K = self.stiffness * np.eye(self.jnt_num)
            gait = SimpleMulinexGaitProblem(self.mulinex.model, lfFoot, rfFoot, lhFoot, rhFoot, self.K, x0)
            GAITPHASES = [
                    {"walking": {"stepLength": 0.15, "stepHeight": 0.05, "timeStep": self.timer_period, "stepKnots": 200, "supportKnots": 1}},
                    {"walking": {"stepLength": 0.10, "stepHeight": 0.05, "timeStep": self.timer_period, "stepKnots": 200, "supportKnots": 1}}
                ]
        self.solver = [None] * len(GAITPHASES)
        for i, phase in enumerate(GAITPHASES):
            for key, value in phase.items():
                if key == "walking":
                    print(colored(f"[INFO]:\t Walking phase", 'yellow'))
                    self.solver[i] = crocoddyl.SolverFDDP(gait.createWalkingProblem(x0, value["stepLength"], value["stepHeight"], value["timeStep"], value["stepKnots"], value["supportKnots"]))
                elif key == "trotting":
                    print(colored(f"[INFO]:\t Trotting phase", 'yellow'))
                    self.solver[i] = crocoddyl.SolverFDDP(gait.createTrottingProblem(x0, value["stepLength"], value["stepHeight"], value["timeStep"], value["stepKnots"], value["supportKnots"]))
                elif key == "pacing":
                    print(colored(f"[INFO]:\t Pacing phase", 'yellow'))
                    self.solver[i] = crocoddyl.SolverFDDP(gait.createPacingProblem(x0, value["stepLength"], value["stepHeight"], value["timeStep"], value["stepKnots"], value["supportKnots"]))
                elif key == "bounding":
                    print(colored(f"[INFO]:\t Bounding phase", 'yellow'))
                    self.solver[i] = crocoddyl.SolverFDDP(gait.createBoundingProblem(x0, value["stepLength"], value["stepHeight"], value["timeStep"], value["stepKnots"], value["supportKnots"]))
                elif key == "jumping":
                    print(colored(f"[INFO]:\t Jumping phase", 'yellow'))
                    #  crocoddyl.self.solverBoxFDDP
                    self.solver[i] = crocoddyl.SolverFDDP(gait.createJumpingProblem(x0, value["jumpHeight"], value["jumpLength"], value["timeStep"], value["groundKnots"], value["flyingKnots"]))
                elif key == "com_move_mp":
                    print(colored(f"[INFO]:\t CoM Move phase", 'yellow'))
                    self.solver[i] = crocoddyl.SolverFDDP(gait.createCoMProblemWalkingVersor(x0, value["comGoTo"], value["versor"], value["timeStep"], value["numKnots"], stepLength=value["stepLength"], stepHeight=value["stepHeight"]))    
                else:
                    raise ValueError("Unknown phase name.")   
            self.solver[i].problem.nthreads = 1
            self.solver[i].th_stop = self.thres_ddp # 1e-5         
            
            xs = [x0] * (self.solver[i].problem.T + 1)
            us = self.solver[i].problem.quasiStatic([x0] * self.solver[i].problem.T)
            self.solver[i].solve(xs, us, self.iter_ddp, False)
            
            if self.robot_name == 'mulinex12':
                print(colored(f"[INFO]:\t DDP Opt. completed for {self.robot_name}", 'cyan'))
                big_data, data_q_ddp, data_q_vel_ddp, data_ff_ddp, data_fb_ddp = dataCallBacks(self.solver[i], 
                                                                                            self.mulinex, 
                                                                                            self.K, 
                                                                                            dtDDP=self.timer_period,
                                                                                            robot_name=self.robot_name, 
                                                                                            nA=12, 
                                                                                            nState=19)
            else:
                print(colored(f"[INFO]:\t DDP Opt. completed for {self.robot_name}", 'cyan'))
                big_data, data_q_ddp, data_q_vel_ddp, data_ff_ddp, data_fb_ddp = dataCallBacks(self.solver[i], 
                                                                                            self.mulinex, 
                                                                                            self.K, 
                                                                                            dtDDP=self.timer_period)
                
            self.big_data.extend(big_data)
            self.data_q_ddp.extend(data_q_ddp)
            self.data_q_vel_ddp.extend(data_q_vel_ddp)
            self.data_ff_ddp.extend(data_ff_ddp)
            self.data_fb_ddp.extend(data_fb_ddp)

    def reference_extract(self, wanna_plot=False):
        '''
        Extract the reference joint states from the data.
        '''
        self.points = int(len(self.big_data) / (3 * self.jnt_num))
        
        self.joint_position = np.zeros((self.jnt_num, self.points))
        self.joint_velocity = np.zeros((self.jnt_num, self.points))
        self.joint_effort = np.zeros((self.jnt_num, self.points))
        
        for i in range(self.points):
            index = i * 3 * self.jnt_num
            pos = self.big_data[index : index + self.jnt_num]
            self.joint_position[:, i] = pos
            
            vel = self.big_data[index + self.jnt_num : index + 2 * self.jnt_num]
            self.joint_velocity[:, i] = vel
            
            # eff = self.big_data[index + 2 * self.jnt_num : index + 3 * self.jnt_num]
            # self.joint_effort[:, i] = eff
            
            # self.joint_velocity[:, i] = np.zeros((self.jnt_num))
            self.joint_effort[:, i] = np.zeros((self.jnt_num))
            
        if wanna_plot: 
            fig, axs = plt.subplots(2, 4, figsize=(20, 10))  # Create a 2x4 grid for plotting
            for i in range(2):
                for j in range(4):
                    joint_idx = 4 * i + j
                    axs[i, j].plot(self.joint_position[joint_idx, :], label='Ref', linewidth=1.5, linestyle='--', color='red')
                    axs[i, j].set_xlabel('Time')
                    axs[i, j].set_ylabel('Position')
                    axs[i, j].legend()
                    axs[i, j].grid()
                    axs[i, j].set_title(self.joint_names[joint_idx]) 

            fig.suptitle('Reference Joint Positions', fontsize=16)
            plt.tight_layout()
            plt.show()
        
            fig, axs = plt.subplots(2, 4, figsize=(20, 10))  # Create a 2x4 grid for plotting
            for i in range(2):
                for j in range(4):
                    joint_idx = 4 * i + j
                    axs[i, j].plot(self.joint_effort[joint_idx, :], linewidth=1.5, linestyle='--', color='green')
                    axs[i, j].set_xlabel('Time')
                    axs[i, j].set_ylabel('Effort')
                    axs[i, j].legend()
                    axs[i, j].grid()
                    axs[i, j].set_title(self.joint_names[joint_idx]) 

            fig.suptitle('Effort FF Joint', fontsize=16)
            plt.tight_layout()
            plt.show()
        
            
    def smooth_derivative(self, data, window_length=11, polyorder=4):
        '''
        Compute the smooth derivative of the given data using the Savitzky-Golay filter after interpolation.
        '''
        return savgol_filter(data, window_length, polyorder, deriv=1, delta=self.timer_period) 
        # plt.plot(range(0,len(data)), data, label='Original')
        # plt.plot(range(0,len(data)), data_interp, label='Interpolated')
        # plt.figure()
        # plt.plot(data, label='Filtered')
                 
    def update_ILC_controller(self):
        '''
        Implement the ILC controller.
        '''
        self.joint_position_error = np.zeros((self.jnt_num, self.points))
        self.joint_velocity_error = np.zeros((self.jnt_num, self.points))
        eff_ = np.zeros((self.jnt_num, self.points))
        vel_meas = np.zeros((self.jnt_num, self.points))
        
        # Compute the velocity measurements
        for jnt in range(self.jnt_num):
            vel_meas[jnt, :] = self.smooth_derivative(self.joint_position_meas[jnt, :, self.iter])
        
        # Compute the position and velocity errors component-wise
        for j in range(self.points):
            # Check the NaN presence/absence
            if not np.any(np.isnan(np.array(self.joint_position_meas[:, j, self.iter]))) and not np.any(np.isnan(np.array(self.joint_velocity_meas[:, j, self.iter]))):
                self.joint_position_error[:, j] = self.joint_position[:, j] - np.array(self.joint_position_meas[:, j, self.iter])
                self.joint_velocity_error[:, j] = self.joint_velocity[:, j] - vel_meas[:, j]
        
        # Compute the ILC control input and update the effort reference
        for j in range(0, self.points):
            self.joint_effort[:, j] += self.ilc_Kp * self.joint_position_error[:, j] + self.ilc_Kd * self.joint_velocity_error[:, j]
                
    def read_joint_states(self, msg):
        '''
        (Callback) Read the joint states from the robot in the correct order.
        '''
        if self.i < self.points:
            if self.iter < self.iter_number and self.j < self.points:
                # Reorder the joint states
                for k, joint_name in enumerate(msg.name):
                    self.joint_position_meas[self.joint_names.index(joint_name), self.j, self.iter] = msg.position[k]
                    self.joint_velocity_meas[self.joint_names.index(joint_name), self.j, self.iter] = msg.velocity[k]
                    self.joint_effort_meas[self.joint_names.index(joint_name), self.j, self.iter] = msg.effort[k] 
                self.j += 1
                # rclpy.logging.get_logger("states").info(f"Recording joint states: {self.joint_position_meas}")
        else:
            self.j = 0    
                
    def publish_joint_states(self):
        '''
        (Callback) Publish the joint states to the robot.
        '''
        self.joint_states_msg.header.stamp.sec = int(self.time // 1)
        self.joint_states_msg.header.stamp.nanosec = int((self.time % 1) * 1e9)
        
        # rclpy.logging.get_logger("states").info(f"Publishing joint states: {self.joint_states_msg}")
        # self.get_logger().info(f"Index: {self.i}")
        
        # Check if the end of the trajectory is reached.
        if self.i >= self.points:
            self.get_logger().info("Finished the feed-forward trajectory.")
            self.get_logger().info("Restarting homing.")
            self.get_logger().info(f"NaN Counted: {self.nan_counter}")
            if self.ilc and self.iter < self.iter_number-1:
                if not self.homing:
                    self.get_logger().info("Updating the ILC control.")
                    self.update_ILC_controller()
                self.homing_callback()
            else:
                raise KeyboardInterrupt()
        
        self.joint_states_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        if not self.homing:
            # If not in homing phase, publish the task references
            self.joint_states_msg.name = self.joint_names
            # Check if there are NaN
            self.nan_counter += np.count_nonzero(np.isnan(self.joint_position[:, self.i]))
            self.nan_counter += np.count_nonzero(np.isnan(self.joint_velocity[:, self.i]))
            self.nan_counter += np.count_nonzero(np.isnan(self.joint_effort[:, self.i]))
            
            self.joint_states_msg.position = self.joint_position[:, self.i].tolist()
            # rclpy.logging.get_logger("states").info(f"joint states: {self.joint_states_msg.position}")
            # rclpy.logging.get_logger("states").info(f"joint names : {self.joint_states_msg.name}")
            self.joint_states_msg.velocity = self.joint_velocity[:, self.i].tolist()
            self.joint_states_msg.effort = self.joint_effort[:, self.i].tolist()
            if not self.simulation:
                self.joint_states_msg.kp_scale = np.ones(self.jnt_num).tolist()
                self.joint_states_msg.kd_scale = np.ones(self.jnt_num).tolist()
                
            self.pub_joint_states.publish(self.joint_states_msg)
            self.i += 1
            # self.time += self.timer_period
        else:
            # If in homing phase, publish the homing references
            self.joint_states_msg.name = self.joint_names
            self.joint_states_msg.position = (self.joint_pos).tolist()
            self.joint_states_msg.velocity = np.zeros(self.jnt_num).tolist()
            self.joint_states_msg.effort = np.zeros(self.jnt_num).tolist()
            
            if not self.simulation:
                self.joint_states_msg.kp_scale = np.ones(self.jnt_num).tolist()
                self.joint_states_msg.kd_scale = np.ones(self.jnt_num).tolist()
                
            self.pub_joint_states.publish(self.joint_states_msg)
            self.get_logger().info(self.joint_states_msg)
            
        time.sleep(self.timer_period / self.rate)
        
    def homing_callback(self):
        '''
        This method performs the homing between two consecutive iterations
        '''
        self.homing = True
        WARMUP_ZONE = 200
        DEAD_ZONE = 10 # seconds
        # Read the last joint positions and set them as the starting values  
        start_pos = self.joint_position[:, -1]
        final_pos = self.joint_position[:, 0]
        if self.init_counter < WARMUP_ZONE:
            self.joint_pos = start_pos
        elif (self.init_counter >= WARMUP_ZONE) and (self.init_counter < WARMUP_ZONE + (self.homing_duration / self.timer_period)):
            # interpolate from current to default pos
            t = ((self.init_counter - WARMUP_ZONE) / (self.homing_duration / self.timer_period))
            # self.get_logger().info(f't: {t}')
            self.joint_pos = start_pos * (1 - t) + final_pos * t

        elif (self.init_counter >= WARMUP_ZONE + (self.homing_duration / self.timer_period)) and (self.init_counter < WARMUP_ZONE +  ((DEAD_ZONE + self.homing_duration) / self.timer_period)):
            # self.init_counter = WARMUP_ZONE +  (self.homing_duration / self.timer_period)
            # Exit from homing state and update the counters
            # input("Press Enter to continue with a new iteration...")
            pass
        else:
            self.homing = False
            self.iter += 1
            self.i = 0
            self.init_counter = 0

        # rclpy.logging.get_logger('rclpy.node').info(f'joint pos: {self.joint_position}')
        self.get_logger().info(f'init_counter: {self.init_counter}')

        self.init_counter += 1         
        
    def plot_joint_states_measurement(self):
        '''
        Plot the joint states at each iteration.
        '''
        fig, axs = plt.subplots(2, 4, figsize=(20, 10))
        
        for iter in range(self.iter_number):
            for i in range(2):
                for j in range(4):
                    axs[i, j].plot(self.joint_position_meas[4 * i + j, :, iter], label=f'Real', linewidth=1.5, linestyle='-', color='blue')
                    axs[i, j].plot(self.joint_position[4 * i + j, :], label='Ref', linewidth=1.5, linestyle='--', color='red')
                    axs[i, j].set_xlabel('Time')
                    axs[i, j].set_ylabel('Position')
                    axs[i, j].grid()
                    axs[i, j].legend()
                    axs[i, j].set_title(self.joint_names[4 * i + j])
            fig.suptitle(f'Iteration {self.i}', fontsize=16)
        plt.tight_layout()
        plt.show()
        
def main(args=None):
    rclpy.init(args=args)
    traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
    ddp_control_node = None 
    
    try:
        ddp_control_node = DDP_Controller()
        rclpy.spin(ddp_control_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        rclpy.shutdown()
    except:
        traceback_logger.error(traceback.format_exc())
    finally:
        if ddp_control_node is not None:  
            ddp_control_node.plot_joint_states_measurement()
            ddp_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()