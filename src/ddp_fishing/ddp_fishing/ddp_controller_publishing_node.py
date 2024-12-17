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
from ballistic_motion import GetBallistMotion 
import crocoddyl
import aslr_to

class DDP_Controller(Node):
    '''
    This class creates a ROS2 node that is used to publish the joint state from the DDP controller to the fihsing rod robot.
    Next, it pusblishes positions, velocities and torques to the robot. Optionally, it allows to implement the ILC controller.
    '''
    def __init__(self):
        '''Initialize the node and the parameters.'''
        super().__init__('ddp_controller_publisher')
        
        # ============================ Parameters ============================ #
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('parent_folder', 'step_jump'),
                ('rate', 1.),
                ('topic_name', '/PD_control/command'),
                ('simulation', True),
                ('ilc', False),
                ('iter_number', 1),
                ('ilc_Kp', 0.5),
                ('ilc_Kd', 0.05),
                ('homing_duration', 5.0),
                ('robot_name', 'fishing_rod_four'),                
                ('iter_ddp', 200),
                ('thres_ddp', 1e-2),
                ('X_des', 20),
                ('Z_des', 0),
                ('v_des', 10),
                ('T_f_des', 10), 
                ('steps', 7000),
                ('u_max',  10),
                ('L0', 2.687), 
            ]
        )
              
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        simulation = self.get_parameter('simulation').get_parameter_value().bool_value
        self.homing_duration = self.get_parameter('homing_duration').get_parameter_value().double_value # for experiments

        self.ilc = self.get_parameter('ilc').get_parameter_value().bool_value
        self.iter_number = self.get_parameter('iter_number').get_parameter_value().integer_value
        self.ilc_Kp = self.get_parameter('ilc_Kp').get_parameter_value().double_value
        self.ilc_Kd = self.get_parameter('ilc_Kd').get_parameter_value().double_value
        
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        
        self.iter_ddp = self.get_parameter('iter_ddp').get_parameter_value().integer_value
        self.thres_ddp = self.get_parameter('thres_ddp').get_parameter_value().double_value
        
        self.X_des = self.get_parameter('X_des').get_parameter_value().double_value
        self.v_des = self.get_parameter('v_des').get_parameter_value().double_value
        self.Z_des = self.get_parameter('Z_des').get_parameter_value().double_value
        self.T_f_des = self.get_parameter('T_f_des').get_parameter_value().double_value
        
        self.k_ii = 4 * np.array([0, 63.1656, 30.8417, 15.1807, 10.1850, 8.8447,
            6.3321, 5.2807, 4.7269, 4.4887, 4.1110, 4.0055, 3.8352, 3.2700, 
            2.6058, 1.7932, 1.3104, 1.0466, 0.9186, 0.6518, 0.3424])

        self.d_ii = 1e1 * np.array([0.087, 0.016, 0.0127, 0.0082, 0.007, 0.0075,
            0.0060, 0.0042, 0.0040, 0.0036, 0.0032, 0.0028, 0.0027, 0.0025, 0.0024,
            0.0020, 0.0018, 0.0016, 0.0015, 0.0012, 0.001])

        self.D = np.diag(self.d_ii) # + np.diag(d_ii[:-1]/2e1, k=-1) + np.diag(d_ii[:-1]/2e1, k=1) 
        self.K = np.diag(self.k_ii) + np.diag(self.k_ii[:-1] / 2e1, k=-1) + np.diag(self.k_ii[:-1] / 2e1, k=1)
        
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
        
        self.timer_period = 0.001 # seconds
        self.time = 0
        
        self.joint_names = ['Joint_1', ]
                                
        self.jnt_num = 1
        self.simulation = simulation
        self.topic_name = topic_name        
        self.nan_counter = 0
        
        test_motion = GetBallistMotion(self.X_des, self.Z_des, self.v_des, self.T_f_des)
        try_res = test_motion.getXZvt()
        state_des = try_res.x
        v_0x, X_0, Z_0, v_0z = try_res.x
        
        print('=============================================================================================')
        print('v_0x: {:.3}\nX_0: {:.3}\t\t X_des: {}\nZ_0: {}\t\t Z_des: {}\nv_0z: {:.3}\ntheta_0: {:.3}\nT: {}'\
            .format(v_0x, state_des[0], self.X_des, state_des[1], self.Z_des, v_0z, np.arctan(v_0x/v_0z), test_motion.T))
        err_X = abs(self.X_des - X_0 - v_0x * test_motion.T)
        err_Z = abs(self.Z_des - Z_0 - v_0z * test_motion.T + 0.5 * test_motion.GRAVITY * (test_motion.T) ** 2)
        print('=============================================================================================')
        print('err_X: {:.3}\nerr_Z: {:.3}'.format(err_X, err_Z))
        if try_res.success:
            print(colored('Success','green'))
            print('X_des: {}\nv_des: {}'.format(self.X_des, self.v_des))
        
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
            
        
        
        print(colored(f"[INFO]:\t CoM Move phase", 'yellow'))
        self.solver[i] = crocoddyl.SolverFDDP(gait.createCoMProblemWalkingVersor(x0, value["comGoTo"], value["versor"], value["timeStep"], value["numKnots"], stepLength=value["stepLength"], stepHeight=value["stepHeight"]))    
                
        self.solver[i].problem.nthreads = 1
        self.solver[i].th_stop = self.thres_ddp # 1e-5         
        
        xs = [x0] * (self.solver[i].problem.T + 1)
        us = self.solver[i].problem.quasiStatic([x0] * self.solver[i].problem.T)
        self.solver[i].solve(xs, us, self.iter_ddp, False)
            
        print(colored(f"[INFO]:\t DDP Opt. completed for {self.robot_name}", 'cyan'))
        big_data, data_q_ddp, data_q_vel_ddp, data_ff_ddp, data_fb_ddp = dataCallBacks(self.solver[i], 
                                                                                    self.mulinex, 
                                                                                    self.K, 
                                                                                    dtDDP=self.timer_period,
                                                                                    robot_name=self.robot_name, 
                                                                                    nA=12, 
                                                                                    nState=19)
            

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