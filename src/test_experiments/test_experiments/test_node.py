import rclpy
# import torch
import numpy as np
from rclpy.node import Node
from pi3hat_moteus_int_msgs.msg import JointsCommand, JointsStates
from sensor_msgs.msg import JointState, Imu
import time
import signal
import matplotlib.pyplot as plt
"""
Simulations node.
This node subscribes to the joint states and tip features topic, and publishes the target joint positions.
"""

class TestController(Node):
    def __init__(self):
        super().__init__('test_node')
        self.time_init = time.time()
        self.prev_timestamp = None
        
        # Simulation flag
        self.declare_parameter('simulation', True)
        self.simulation = self.get_parameter('simulation').get_parameter_value().bool_value
        
        self.declare_parameter('trajectory', 'const') # 'const' # 'chirp' # 'sinusoidal'
        self.trajectory = self.get_parameter('trajectory').get_parameter_value().string_value
        self.time_traj = 0.0
        rclpy.logging.get_logger('rclpy.node').info(f"trajectory type : {self.trajectory}")
        
        self.declare_parameter('publication_rate', 1000)
        self.rate = self.get_parameter('publication_rate').get_parameter_value().integer_value
                
        # Topic names
        self.declare_parameter('joint_state_topic', '/state_broadcaster/joint_states')
        self.declare_parameter('joint_target_pos_topic', '/joint_controller/command')
        self.joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        self.joint_target_pos_topic = self.get_parameter('joint_target_pos_topic').get_parameter_value().string_value
        
        self.DEBUGGING = False
        
        self.default_dof = np.array([0.0]) 
        self.njoint = 1
        self.joint_names = ('Joint_1',)

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
        self.tip_vel_lin = [0.0 for _ in range(3)]
        self.tip_pos = [0.0000, 0.0700, 2.9417]
        
        self.tip_plot = [0.0]
        self.tip_vel_plot = [0.0]
        self.joint_pos_active_plot = list(self.joint_pos.values()) 
        self.joint_vel_active_plot = [0.0]
        
        if self.trajectory == 'const':
            self.joint_pos_des = [1.0]
            self.joint_vel_des = [0.0]
            self.time_traj = 5.0
        elif self.trajectory == 'sinusoidal':
            self.amplitude = np.pi / 8
            self.frequency = 0.5
            self.joint_pos_des = [0.0]
            self.joint_vel_des = [0.0]
            self.time_traj = 10.0
        elif self.trajectory == 'chirp':
            self.amplitude = np.pi / 8
            self.start_freq = 0.1
            self.end_freq = 1.0
            self.duration = 10.0
            self.joint_pos_des = [0.0]
            self.joint_vel_des = [0.0]
            self.time_traj = 10.0
        else:
            self.get_logger().error(f"Unknown traj: {self.trajectory}. Please select one <const> <sinusoidal> <chirp>")            
            return

        self.timer = self.create_timer(1.0 / self.rate, self.test_callback)
        self.startup_time = rclpy.clock.Clock().now()
        self.startup_time_obs = self.startup_time
        
    def joint_state_callback(self, msg):
        t = rclpy.clock.Clock().now()
        timestamp = t.nanoseconds / 1e9 # [s]
        for i in range(self.njoint):
            if (not np.isnan(msg.position[i]) and (not np.isnan(msg.velocity[i]))):
                self.joint_pos[msg.name[i]] = msg.position[i]
                self.joint_vel[msg.name[i]] = msg.velocity[i]
                if rclpy.clock.Clock().now() > (self.startup_time + rclpy.duration.Duration(seconds=3.0)):
                    self.joint_pos_active_plot.append(self.joint_pos[msg.name[i]])
                    self.joint_vel_active_plot.append(self.joint_vel[msg.name[i]])
        self.prev_timestamp = timestamp
    
    def imu_callback(self, msg):
        """Callback to process IMU data and extract tip position."""
        linear_acceleration = np.array([
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
        self.tip_vel_lin += linear_acceleration * dt
        self.tip_pos += self.tip_vel_lin * dt

    def test_callback(self):
        """ Callback function for test timer. Infers joints target_pos from model and publishes it. """  
        
        if self.DEBUGGING:
            rclpy.logging.get_logger('rclpy.node').info(f"joint_pos   : {self.joint_pos}")
            rclpy.logging.get_logger('rclpy.node').info(f"joint_vel   : {self.joint_vel}")
            rclpy.logging.get_logger('rclpy.node').info(f"tip_pos     : {self.tip_pos}")
            # rclpy.logging.get_logger('rclpy.node').info(f"tip_vel_lin : {self.tip_vel_lin}")
                        
        if self.simulation:
            joint_msg = JointState()
        else:
            joint_msg = JointsCommand()
        joint_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        joint_msg.name = self.joint_names
        
        if rclpy.clock.Clock().now() < (self.startup_time + rclpy.duration.Duration(seconds=3.0)):
            self.startup_time_obs = rclpy.clock.Clock().now()
            joint_msg.position = self._avg_default_dof
            joint_msg.velocity = np.zeros(self.njoint).tolist()
        else:               
            if rclpy.clock.Clock().now() > (self.startup_time_obs + rclpy.duration.Duration(seconds=self.time_traj)):
                rclpy.logging.get_logger('rclpy.node').info(f"Real traj END  ...")
                joint_msg.position = self._avg_default_dof
                joint_msg.velocity = np.zeros(self.njoint).tolist()
                
            else:
                rclpy.logging.get_logger('rclpy.node').info(f"Real traj INIT ...")
                if self.trajectory == 'const':
                    joint_msg.position = self.joint_pos_des 
                    joint_msg.velocity = [float(-value) for value in self.joint_vel.values()]
                    # joint_msg.velocity = self.joint_vel_des
                elif self.trajectory == 'sinusoidal':
                    t = (rclpy.clock.Clock().now() - self.startup_time).nanoseconds / 1e9
                    self.joint_pos_des[0] = self.amplitude * np.sin(2 * np.pi * self.frequency * t)
                    self.joint_vel_des[0] = self.amplitude * 2 * np.pi * self.frequency * np.cos(2 * np.pi * self.frequency * t)
                    joint_msg.position = [float(self.joint_pos_des[0] + self._avg_default_dof)]
                    joint_msg.velocity = [float(-self.joint_vel_des[0])]
                elif self.trajectory == 'chirp':
                    t = (rclpy.clock.Clock().now() - self.startup_time).nanoseconds / 1e9
                    k = (self.end_freq - self.start_freq) / self.duration
                    self.joint_pos_des[0] = self.amplitude * np.sin(2 * np.pi * (self.start_freq * t + 0.5 * k * t**2))
                    self.joint_vel_des[0] = self.amplitude * 2 * np.pi * (self.start_freq + k * t) * np.cos(2 * np.pi * (self.start_freq * t + 0.5 * k * t**2))
                    joint_msg.position = [float(self.joint_pos_des[0] + self._avg_default_dof)]
                    joint_msg.velocity = [float(-self.joint_vel_des[0])]
                else:
                    self.get_logger().error(f"Unknown traj: {self.trajectory}. Please select one <const> <sinusoidal> <chirp>")
                    return
                
            self.tip_plot.append(self.tip_pos[0])
            self.tip_vel_plot.append(self.tip_vel_lin[0])
            # self.joint_pos_active_plot.append(joint_msg.position[0]) # des
            # self.joint_vel_active_plot.append(joint_msg.velocity[0]) # des 
            
        if not self.simulation:
            joint_msg.kp_scale = self.joint_kp.tolist()
            joint_msg.kd_scale = self.joint_kd.tolist()
            
        # joint_msg.velocity = np.zeros(self.njoint).tolist()
        joint_msg.effort = np.zeros(self.njoint).tolist()
        self.joint_target_pos_pub.publish(joint_msg)
        
    def callback_plot(self, *args):
        """ Function to plot the data """

        from matplotlib import rc
        rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
        rc('text', usetex=True)
        plt.rcParams['text.usetex'] = True
        
        labelWidth = 4
        font_size = 18
        plt.figure(figsize=(16, 14))

        plt.subplot(2, 2, 1)
        # plt.plot(np.linspace(0, len(self.tip_plot) / self.rate, len(self.tip_plot)), self.tip_plot, linewidth=labelWidth, label='Tip Position')
        plt.plot(self.tip_plot, linewidth=labelWidth, label='Tip Position')
        plt.xlabel(r'Time $(nT)$')
        plt.ylabel(r'Tip Position $[m]$')
        plt.legend()
        plt.grid()
        
        plt.subplot(2, 2, 2)
        # plt.plot(np.linspace(0, len(self.tip_vel_plot) / self.rate, len(self.tip_vel_plot)), self.tip_vel_plot, linewidth=labelWidth, label='Tip Velocity')
        plt.plot(self.tip_vel_plot, linewidth=labelWidth, label='Tip Velocity')
        plt.xlabel(r'Time $(nT)$')
        plt.ylabel(r'Tip Velocity $[m/s]$')
        plt.legend()
        plt.grid()
        
        plt.subplot(2, 2, 3)
        # plt.plot(np.linspace(0,len(self.joint_pos_active_plot)) / self.rate, self.joint_pos_active_plot, linewidth=labelWidth, label='Joint Position')
        plt.plot(self.joint_pos_active_plot, linewidth=labelWidth, label='Joint Position')
        plt.xlabel(r'Time $(nT)$')
        plt.ylabel(r'Joint Position $[rad]$')
        plt.legend()
        plt.grid()
        
        plt.subplot(2, 2, 4)
        # plt.plot(np.linspace(0, len(self.joint_vel_active_plot) / self.rate, len(self.joint_vel_active_plot)), self.joint_vel_active_plot, linewidth=labelWidth, label='Joint Velocity')
        plt.plot(self.joint_vel_active_plot, linewidth=labelWidth, label='Joint Velocity')
        plt.xlabel(r'Time $(nT)$')
        plt.ylabel(r'Joint Velocity $[rad/s]$')
        plt.legend()
        plt.grid()
        
        plt.tick_params(labelsize=font_size)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    test_controller = TestController()
    
    def signal_handler(signum, frame):
        test_controller.callback_plot()

    signal.signal(signal.SIGINT, signal_handler)
    rclpy.spin(test_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
