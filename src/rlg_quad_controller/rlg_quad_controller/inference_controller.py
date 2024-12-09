import rclpy
# import torch
import numpy as np
import yaml
from rclpy.node import Node
from pi3hat_moteus_int_msgs.msg import JointsCommand, JointsStates
from sensor_msgs.msg import JointState, Imu
# from geometry_msgs.msg import Twist, Point
import time
from .utils.rlg_utils import build_rlg_model, run_inference
from termcolor import colored
"""
Simulations node.
This node subscribes to the joint states and tip features topic, and publishes the target joint positions.
first, joint_pos, tip pos, tip vel --> | inference_controller | --> joint_target_pos --> PD contr
"""

class InferenceController(Node):
    def __init__(self):
        super().__init__('inference_controller')
        self.time_init = time.time()
        self.prev_timestamp = None
        # Simulation flag
        self.declare_parameter('simulation', True)
        self.simulation = self.get_parameter('simulation').get_parameter_value().bool_value

        # Model path as pth file
        self.declare_parameter('model_path', '')
        self.declare_parameter('config_path', '')
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.config_path = self.get_parameter('config_path').get_parameter_value().string_value

        # Topic names
        self.declare_parameter('joint_state_topic', '/state_broadcaster/joint_states')
        self.declare_parameter('joint_target_pos_topic', '/joint_controller/command')
        self.joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        self.joint_target_pos_topic = self.get_parameter('joint_target_pos_topic').get_parameter_value().string_value
        
        self.DEBUGGING = False

        # Inference rate
        with open(self.config_path, 'r') as f:
            params = yaml.safe_load(f)
                
        self.rate = 1.0 / params['task']['sim']['dt']  
        self.rate_elegent = 1.0 / ( params['task']['sim']['dt'] * \
            ( params['task']['env']['control']['decimation'] + params['task']['env']['controlFrequencyInv']))
        rclpy.logging.get_logger('rclpy.node').info('Inference rate: {}'.format(self.rate))
        
        self.action_scale = params['task']['env']['control']['actionScale']    
        self.position_scale = params['task']['env']['learn']['positionTipScale']           
        self.velocity_scale = params['task']['env']['learn']['velLinTipScale']        
        self.timeEpisode = params['task']['env']['episodeLength']
        self.clip_obs = params['task']['env']['clipObservations']
        self.num_act = params['task']['env']['numActions']
        self.num_obs = params['task']['env']['numObservations']
        self.clip_act = params['task']['env']['control']['actionScale']  
        
        # I do not need the passive joints pos
        self.default_dof = np.array([0.0]) 
        self.previous_action = (self.default_dof / self.action_scale).tolist()
        
        # Initialize joint subscriber
        self.njoint = 1
        self.joint_names = ('Joint_1',)

        # this is what I am publishing 
        self.joint_kp = np.array([1.0])
        self.joint_kd = np.array([1.0])

        if self.simulation:
            self.joint_target_pos_pub = self.create_publisher(JointState, self.joint_target_pos_topic, 10)
            self.joint_sub = self.create_subscription(JointState, self.joint_state_topic, self.joint_state_callback, 10)
            self.imu_subscription = self.create_subscription(Imu, '/imu/data', self.imu_callback,10)
        else:
            self.joint_target_pos_pub = self.create_publisher(JointsCommand, self.joint_target_pos_topic, 10)
            self.joint_sub = self.create_subscription(JointsStates, self.joint_state_topic, self.joint_state_callback, 10)

        self.joint_pos = {self.joint_names[i]:0.0 for i in range(self.njoint)}
        self.joint_vel = {self.joint_names[i]:0.0 for i in range(self.njoint)}
        self._avg_default_dof = list(self.joint_pos.values()) 
        self.pos_des = [0.4]
        self.vel_des = [5.0]
        self.tip_vel_lin = [0.0 for _ in range(3)]
        self.tip_pos = [0.0 for _ in range(3)]   
        self.previous_action = [0.0 for _ in range(self.njoint)]
        self.tip_pos_previous = [0.0 for _ in range(3)]
        self.tip_vel_lin_previous = [0.0 for _ in range(3)]
        self.err_pos = [0.0 for _ in range(self.njoint)]
        self.err_vel = [0.0 for _ in range(self.njoint)]

        # Load PyTorch model and create timer
        rclpy.logging.get_logger('rclpy.node').info('Loading model from {}'.format(self.model_path))
        self.model = build_rlg_model(self.model_path, params)
        # start inference
        self.timer = self.create_timer(1.0 / self.rate, self.inference_callback)
        rclpy.logging.get_logger('rclpy.node').info('Model loaded. Node ready for inference.')
        self.startup_time = rclpy.clock.Clock().now()
        self.startup_time_obs = self.startup_time
        
    def joint_state_callback(self, msg):
        t = rclpy.clock.Clock().now()
        timestamp = t.nanoseconds / 1e9 # [s]
        for i in range(self.njoint):
            if (not np.isnan(msg.position[i]) and (not np.isnan(msg.velocity[i]))):
                self.joint_pos[msg.name[i]] = msg.position[i]
                # self.joint_vel[msg.name[i]] = msg.velocity[i]
        self.prev_timestamp = timestamp
    
    def imu_callback(self, msg):
        """Callback to process IMU data and extract tip position."""
        linear_velocity = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z])
        
        t = rclpy.clock.Clock().now()
        timestamp = t.nanoseconds / 1e9
        if self.prev_timestamp is None:
            self.prev_timestamp = timestamp
            return
        
        dt = timestamp - self.prev_timestamp
        self.prev_timestamp = timestamp
        self.tip_pos += linear_velocity * dt
        self.tip_vel = linear_velocity
        self.err_pos = self.pos_des - self.tip_pos[0]
        self.err_vel = self.vel_des - self.tip_vel[0]

    def inference_callback(self):
        """ Callback function for inference timer. Infers joints target_pos from model and publishes it. """  
        
        obs_list = np.concatenate((      
            np.fromiter(list(self.joint_pos.values()), dtype=float).reshape((self.njoint, 1)) / self.action_scale,
            np.fromiter(list(self.tip_vel_lin), dtype=float).reshape((3, 1)) / self.velocity_scale,
            np.fromiter(list(self.tip_pos), dtype=float).reshape((3, 1)) / self.position_scale,
            np.fromiter(list(self.tip_pos_previous), dtype=float).reshape((3, 1)) / self.position_scale,
            np.fromiter(list(self.tip_vel_lin_previous), dtype=float).reshape((3, 1)) / self.velocity_scale,
            np.fromiter(list(self.pos_des), dtype=float).reshape((1, 1)) / self.position_scale,
            np.fromiter(list(self.vel_des), dtype=float).reshape((1, 1)) / self.velocity_scale,
            np.fromiter(list(self.err_pos), dtype=float).reshape((1, 1)) / self.position_scale,
            np.fromiter(list(self.err_vel), dtype=float).reshape((1, 1)) / self.velocity_scale,
            np.reshape(self.previous_action, (self.njoint, 1)) #,
            # np.array([(self.timeEpisode - (rclpy.clock.Clock().now().nanoseconds / 1e9 - self.startup_time_obs.nanoseconds / 1e9)) / self.timeEpisode]).reshape((1,1)) 
        )).reshape((1, self.num_obs)) 
        
        obs_list = np.clip(obs_list, [-self.clip_obs] * self.num_obs, [self.clip_obs] * self.num_obs)         
        action = run_inference(self.model, obs_list)
        self.previous_action = np.reshape(action, (self.njoint, 1))
        action = np.clip(action, [-self.clip_act] * (self.njoint), [self.clip_act] * (self.njoint))
        self.tip_pos_previous = self.tip_pos.copy()
        self.tip_vel_lin_previous = self.tip_vel_lin.copy()
        
        if self.DEBUGGING:
            rclpy.logging.get_logger('rclpy.node').info(f"joint_pos              : {self.joint_pos}")
            rclpy.logging.get_logger('rclpy.node').info(f"tip_vel_lin            : {self.tip_vel_lin}")
            rclpy.logging.get_logger('rclpy.node').info(f"tip_pos                : {self.tip_pos}")
            rclpy.logging.get_logger('rclpy.node').info(f"action                 : {action}")
            # rclpy.logging.get_logger('rclpy.node').info(f"tip_pos_previous       : {self.tip_pos_previous}")
            # rclpy.logging.get_logger('rclpy.node').info(f"tip_vel_lin_previous   : {self.tip_vel_lin_previous}")
        
        if self.simulation:
            joint_msg = JointState()
        else:
            joint_msg = JointsCommand()
        joint_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        joint_msg.name = self.joint_names
        
        if rclpy.clock.Clock().now() < (self.startup_time + rclpy.duration.Duration(seconds=5.0)):
            self.startup_time_obs = rclpy.clock.Clock().now()
            action *= 0.0
            joint_msg.position = self._avg_default_dof
        else:               
            if rclpy.clock.Clock().now() > (self.startup_time_obs + rclpy.duration.Duration(seconds=2.0)):
                rclpy.logging.get_logger('rclpy.node').info(colored('Policy stops ...', 'yellow')) 
                joint_msg.position = self._avg_default_dof
            else:
                rclpy.logging.get_logger('rclpy.node').info(colored('Policy starts working ...', 'cyan'))                 
                joint_msg.position = [float(action[0, 0] * self.action_scale)]            
        if not self.simulation:
            joint_msg.kp_scale = self.joint_kp.tolist()
            joint_msg.kd_scale = self.joint_kd.tolist()
            
        joint_msg.velocity = np.zeros(self.njoint).tolist()
        joint_msg.effort = np.zeros(self.njoint).tolist()
        self.joint_target_pos_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    inference_controller = InferenceController()
    rclpy.spin(inference_controller)
    inference_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
