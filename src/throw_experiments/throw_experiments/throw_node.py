""" 
Ros2 node that publishes cmd_height for experiments.
The node uses the get_tip_features_function(time), written by the user, to generate the cmd_height
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import time

####################################################################################################
# ThrowNode for the exoeriments 
# ToDo
####################################################################################################

class ThrowNode(Node):

    def __init__(self):
        super().__init__('throw_node')
        #### parameters
        self.declare_parameter('publication_rate', 500) # Hz rate at which to publish cmd_height
        self.declare_parameter('duration', 3.0)         # s net duration of the experiment
        self.declare_parameter('start_delay', 6.0)      # s delay before starting tp send ref
        self.declare_parameter('shutdown_delay', 2.0)   # s delay before shutting down the node
        self.declare_parameter('pos_des', 0.4)          # pos X m. This may be changed by the user   
        self.declare_parameter('vel_des', 5.0)          # vel X m/s. This may be changed by the user

        self.publication_rate = self.get_parameter('publication_rate').get_parameter_value().integer_value
        self.duration = self.get_parameter('duration').get_parameter_value().double_value
        self.start_delay = self.get_parameter('start_delay').get_parameter_value().double_value
        self.shutdown_delay = self.get_parameter('shutdown_delay').get_parameter_value().double_value
        self.pos_des = self.get_parameter('pos_des').get_parameter_value().double_value
        self.vel_des = self.get_parameter('vel_des').get_parameter_value().double_value

        # log parameters:
        self.get_logger().info('publication_rate: {}'.format(self.publication_rate))
        self.get_logger().info('duration: {}'.format(self.duration))
        self.get_logger().info('start_delay: {}'.format(self.start_delay))
        self.get_logger().info('shutdown_delay: {}'.format(self.shutdown_delay))

        self.publisher_ = self.create_publisher(Point, '/cmd_height', 10)
        self.startup_time = time.time()
        self.current_time = time.time()
        self.timer = self.create_timer(1.0 / self.publication_rate, self.timer_callback)
        self.get_logger().info('ThrowNode started at {} s'.format(self.startup_time))

    def timer_callback(self):
        """publishes the des pos and vel of the tip at the desired rate."""
        msg_pos = Point()
        msg_vel = Twist()
        t = time.time() - self.startup_time
        msg_pos.x, msg_vel.linear.x = self.get_tip_features_function(t) 
        msg_pos.z, msg_pos.y, msg_vel.linear.y, msg_vel.linear.z = 0.0, 0.0, 0.0, 0.0
        self.publisher_.publish(msg_pos)
        self.publisher_.publish(msg_vel)
        # self.get_logger().info('Publishing: "%s"' % msg_pos.z)

    def get_tip_features_function(self, t):
        """ Function that returns the desired pos X m and vel X m/s of the tip vector as a function of time.
        We could select different desired features. 
        """
        if t > self.start_delay and t < (self.start_delay + self.duration):
            return self.pos_des, self.vel_des
        elif t > (self.start_delay + self.duration + self.shutdown_delay):
            self.get_logger().info('throwNode shutting down at {} s'.format(time.time()))
            self.destroy_timer(self.timer)
            self.destroy_node() # Is this safe? ¯\(°_o)/¯
            return 0.0
        else:
            return 0.0 
        
def main(args=None):
    rclpy.init(args=args)
    throw_node = ThrowNode()
    rclpy.spin(throw_node)
    throw_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()