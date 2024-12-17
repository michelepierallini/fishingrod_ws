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
from .ballistic_motion import GetBallistMotion 
import crocoddyl

import aslr_to
print(aslr_to.__path__)

from .utils import dataCallBacks

class DDP_Controller(Node):
    
    '''
    This class creates a ROS2 node that is used to publish the joint state from the DDP controller to the fihsing rod robot.
    Next, it pusblishes positions, velocities and torques to the robot. Optionally, it allows to implement the ILC controller.
    '''
    
    def __init__(self):
        '''
        Initialize the node and the parameters.
        '''
        super().__init__('ddp_controller_publisher_node')
        
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
                ('X_des', 20.0),
                ('Z_des', 0.0),
                ('v_des', 10.0),
                ('T_f_des', 10.0), 
                ('steps_ddp', 7000), 
                ('u_max', 10.0),
                ('L0', 2.687), 
                ('alpha', np.pi / 2),
                ('timer_period', 0.005), # 0.0001
                ('timer_period_ddp', 0.0001), 
                ('scale_Kp', 2.0),
                ('scale_Kv', 1.0),
            ]
        )
              
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        simulation = self.get_parameter('simulation').get_parameter_value().bool_value
        self.homing_duration = self.get_parameter('homing_duration').get_parameter_value().double_value # for experiments
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.timer_period_ddp = self.get_parameter('timer_period_ddp').get_parameter_value().double_value
        self.scale_Kp = self.get_parameter('scale_Kp').get_parameter_value().double_value
        self.scale_Kv = self.get_parameter('scale_Kv').get_parameter_value().double_value

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
        self.fixed_length = 0.18
        
        self.steps_ddp = self.get_parameter('steps_ddp').get_parameter_value().integer_value
        self.u_max = self.get_parameter('u_max').get_parameter_value().double_value
        self.L0 = self.get_parameter('L0').get_parameter_value().double_value
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value
        
        self.target_pos_ddp = np.array([self.L0 * np.cos(self.alpha) + self.fixed_length, 0, self.L0 * np.sin(self.alpha)])
        self.target_vel_ddp = np.array([self.v_des, -self.v_des / 2, 0])
        
        # Check the correctness of the topics
        sim_condition = (simulation == True) and (topic_name == '/PD_control/command')
        real_condition = (simulation == False) and (topic_name == '/joint_controller/command')
        
        if sim_condition == False and real_condition == False:
            raise KeyboardInterrupt('Wrong Simulation flag or topic name')

        # ============================ Publishers ============================ #
        
        if simulation:
            self.joint_states_msg = JointState()
            self.joint_target_pos_topic = topic_name
            self.pub_joint_states = self.create_publisher(JointState, self.joint_target_pos_topic, 10)
        else:
            self.joint_states_msg = JointsCommand()
            self.joint_target_pos_topic = topic_name
            self.pub_joint_states = self.create_publisher(JointsCommand, self.joint_target_pos_topic, 10)      
              
        # ============================ Variables zeros ============================ #
        
        self.big_data, self.data_q_ddp, self.data_q_vel_ddp, self.data_ff_ddp, self.data_fb_ddp = [], [], [], [], []
        self.WANNA_TEST = False
        self.homing = False
        self.i = 0  # index for the commanded joint states
        self.init_counter = 0
        self.iter = 0
        self.j = 0  # index for the measured joint states
        self.time = 0
        
        self.joint_names = ['Joint_1']
                                
        self.jnt_num = 1 # this is fixed
        self.simulation = simulation
        self.topic_name = topic_name        
        self.nan_counter = 0
        
        if self.WANNA_TEST:
            test_motion = GetBallistMotion(self.X_des, self.Z_des, self.v_des, self.T_f_des)
            try_res = test_motion.getXZvt()
            state_des = try_res.x
            v_0x, X_0, Z_0, v_0z = try_res.x
            
            self.get_logger().info('=============================================================================================')
            self.get_logger().info('v_0x: {:.3}\nX_0: {:.3}\t\t X_des: {}\nZ_0: {}\t\t Z_des: {}\nv_0z: {:.3}\ntheta_0: {:.3}\nT: {}'\
                .format(v_0x, state_des[0], self.X_des, state_des[1], self.Z_des, v_0z, np.arctan(v_0x/v_0z), test_motion.T))
            err_X = abs(self.X_des - X_0 - v_0x * test_motion.T)
            err_Z = abs(self.Z_des - Z_0 - v_0z * test_motion.T + 0.5 * test_motion.GRAVITY * (test_motion.T) ** 2)
            self.get_logger().info('=============================================================================================')
            self.get_logger().info('err_X: {:.3}\terr_Z: {:.3}'.format(err_X, err_Z))
            if try_res.success:
                self.get_logger().info(colored('Success','green'))
                self.get_logger().info('X_des: {}\tv_des: {}'.format(self.X_des, self.v_des))
                    
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
        '''
        Implement the DDP controller.
        '''        
        fishing_rod = example_robot_data.load(self.robot_name)
        robot_model = fishing_rod.model
        robot_model.gravity.linear = np.array([0, 0, -9.81]) # w.r.t. global frame
        state = crocoddyl.StateMultibody(robot_model)
        actuation = aslr_to.ASRFishing(state)
        nu = actuation.nu

        runningCostModel = crocoddyl.CostModelSum(state,nu)
        terminalCostModel = crocoddyl.CostModelSum(state,nu)
        xResidual = crocoddyl.ResidualModelState(state, state.zero(), nu)
        uResidual = crocoddyl.ResidualModelControl(state, nu)
        uRegCost = crocoddyl.CostModelResidual(state, uResidual)

        framePlacementResidual = crocoddyl.ResidualModelFramePlacement(state, robot_model.getFrameId("Link_EE"),
                                                               pinocchio.SE3(np.eye(3), self.target_pos_ddp), nu)
        framePlacementVelocity = crocoddyl.ResidualModelFrameVelocity(state, robot_model.getFrameId("Link_EE"),
                                                               pinocchio.Motion(self.target_vel_ddp, np.zeros(3)),
                                                               pinocchio.WORLD, nu)
        
        xActivation = crocoddyl.ActivationModelWeightedQuad(np.array([1e1] * state.nv + [1e0] * state.nv)) # 1e1
        xResidual = crocoddyl.ResidualModelState(state, state.zero(), nu)
        xRegCost = crocoddyl.CostModelResidual(state, xActivation, xResidual)
        goalTrackingCost = crocoddyl.CostModelResidual(state, framePlacementResidual)
        goalVelCost = crocoddyl.CostModelResidual(state, framePlacementVelocity)
        xRegCost = crocoddyl.CostModelResidual(state, xResidual)

        runningCostModel.addCost("gripperPose", goalTrackingCost, 1e2)
        runningCostModel.addCost("gripperVel", goalVelCost, 1e1)
        runningCostModel.addCost("xReg", xRegCost, 1e0)
        runningCostModel.addCost("uReg", uRegCost, 1e0) # increase to decrease the cost of the control
        terminalCostModel.addCost("gripperPose", goalTrackingCost, 1e2) 
        terminalCostModel.addCost("gripperVel", goalVelCost, 1e1)
                
        runningModel = crocoddyl.IntegratedActionModelEuler(
                                aslr_to.DAM2(state, actuation, runningCostModel, self.K, self.D), 
                                self.timer_period_ddp
                            )
        terminalModel = crocoddyl.IntegratedActionModelEuler(
                                aslr_to.DAM2(state, actuation, terminalCostModel, self.K, self.D), 
                                0
                            ) # dt) # need to rescale the problem

        runningModel.u_lb, runningModel.u_ub = np.array([-self.u_max]), np.array([self.u_max])

        q0 = np.zeros(state.nv)
        x0 = np.concatenate([q0,pinocchio.utils.zero(state.nv)])
        problem = crocoddyl.ShootingProblem(x0, [runningModel] * int(self.steps_ddp), terminalModel)
        solver = crocoddyl.SolverBoxFDDP(problem)  
                    
        solver.problem.nthreads = 1
        solver.th_stop = self.thres_ddp         
        
        xs = [x0] * (solver.problem.T + 1)
        us = [np.zeros(1)] * (solver.problem.T)
        
        self.get_logger().info(colored(f"DDP Opt. starting for {self.robot_name}", 'cyan'))
        solver.solve(xs, us, self.iter_ddp, False)
        
        # ============================ Results ============================ #
        
        pos_final = solver.problem.terminalData.differential.multibody.pinocchio.oMf[robot_model.getFrameId("Link_EE")].translation.T
        vel_final = pinocchio.getFrameVelocity(solver.problem.terminalModel.differential.state.pinocchio, 
                                                solver.problem.terminalData.differential.multibody.pinocchio, 
                                                robot_model.getFrameId("Link_EE")).linear

        self.get_logger().info('Reached Pos: {}\tReached Vel: {}'.format(np.round(pos_final, 3), np.round(vel_final, 3)))
        self.get_logger().info('Desired Pos: {}\tDesired Vel: {}'.format(np.round(self.target_pos_ddp, 3), np.round(self.target_vel_ddp, 3)))
        
        self.get_logger().info(colored(f"DDP Opt. completed for {self.robot_name}", 'cyan'))
        self.big_data, self.data_q_ddp, self.data_q_vel_ddp, self.data_ff_ddp = dataCallBacks(solver, fishing_rod)
        
    def reference_extract(self, wanna_plot=True):
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
            # rclpy.logging.get_logger("states").info(f"joint des: {pos}")
            
            vel = self.big_data[index + self.jnt_num : index + 2 * self.jnt_num]
            self.joint_velocity[:, i] = vel
            
            # eff = self.big_data[index + 2 * self.jnt_num : index + 3 * self.jnt_num]
            # self.joint_effort[:, i] = eff
            
            # self.joint_velocity[:, i] = np.zeros((self.jnt_num))
            self.joint_effort[:, i] = np.zeros((self.jnt_num))
            
        if wanna_plot: 
            line_width = 3.5
            fig, axs = plt.subplots(1, 3, figsize=(20, 10))  # Create a 1x3 grid for plotting
            axs[0].plot(self.joint_position[0,:], label='Joint 1', linewidth=line_width, linestyle='--', color='red')
            axs[0].set_xlabel('Time', fontsize=16)
            axs[0].set_ylabel('Position', fontsize=16)
            axs[0].legend()
            axs[0].grid()
            
            axs[1].plot(self.joint_velocity[0,:], label='Joint 1', linewidth=line_width, linestyle='--', color='red')
            axs[1].set_xlabel('Time', fontsize=16)
            axs[1].set_ylabel('Velocity', fontsize=16)
            axs[1].legend()
            axs[1].grid()
            
            axs[2].plot(self.joint_effort[0,:], label='Joint 1', linewidth=line_width, linestyle='--', color='red')
            axs[2].set_xlabel('Time', fontsize=16)
            axs[2].set_ylabel('Effort', fontsize=16)
            axs[2].legend()
            axs[2].grid()
        
            fig.suptitle('DDP', fontsize=16)
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
                self.joint_position_meas[0, self.j, self.iter] = msg.position[0]
                self.joint_velocity_meas[0, self.j, self.iter] = msg.velocity[0]
                self.joint_effort_meas[0, self.j, self.iter] = msg.effort[0] 
                self.j += 1
                # rclpy.logging.get_logger("states").info(f"joint_position_meas: {self.joint_position_meas[0, self.j, self.iter]}")
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
                self.joint_states_msg.kp_scale = self.scale_Kp * np.ones(self.jnt_num).tolist()
                self.joint_states_msg.kd_scale = self.scale_Kp * np.ones(self.jnt_num).tolist()
                
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
        final_pos = self.joint_position[:, 0] * 0 
        if self.init_counter < WARMUP_ZONE:
            self.joint_pos = start_pos
        elif (self.init_counter >= WARMUP_ZONE) and (self.init_counter < WARMUP_ZONE + (self.homing_duration / self.timer_period)):
            # interpolate from current to default pos
            t = ((self.init_counter - WARMUP_ZONE) / (self.homing_duration / self.timer_period))
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
        # self.get_logger().info(f'init_counter: {self.init_counter}')

        self.init_counter += 1         
        
    def plot_joint_states_measurement(self):
        '''
        Plot the joint states at each iteration.
        '''
        fig, ax = plt.subplots(figsize=(10, 5))
        line_width = 3.5
        for iter in range(self.iter_number):
            ## just the first joint is actuated in the fishing rod
            ax.plot(self.joint_position_meas[0, :, iter], label=f'Real {iter}', linewidth=line_width, linestyle='-', color='blue')
            ax.plot(self.joint_position[0, :], label=f'Ref {iter}', linewidth=line_width, linestyle='--', color='red')
        
        ax.set_xlabel('Time', fontsize=16)
        ax.set_ylabel('Position', fontsize=16)
        ax.grid()
        ax.legend()
        fig.suptitle(f'Joint States Measurement', fontsize=20)
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