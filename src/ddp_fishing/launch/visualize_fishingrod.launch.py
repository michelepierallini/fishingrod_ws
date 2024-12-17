import os
import numpy as np

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    mulinex_descr_share_path = FindPackageShare('fishing_description')

    xacro_file_path = PathJoinSubstitution([
        mulinex_descr_share_path,
        LaunchConfiguration('xacro_file_path', default=os.path.join('urdf', 'mulinex.urdf.xacro'))
    ])
    
    config_file_path = PathJoinSubstitution([
        mulinex_descr_share_path,
        LaunchConfiguration('config_file_path', default=os.path.join('rviz', 'config.rviz'))
    ])
    
    # Load the path to the parameters file
    config = PathJoinSubstitution([
        FindPackageShare('ddp_fishing'),
        'config',
        'ddp_controller_params.yaml'
    ])
    
    bag_filename = LaunchConfiguration('bag_filename', default='010_move_base')
    
    rate = LaunchConfiguration('rate', default='1.')
    
    topic_name = LaunchConfiguration('topic_name', default='/joint_states')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    ### I never used this one, no idea about the configuration 

    default_dof = (0,)
    joint_names = ('Joint_1',)
    default_dict = dict(zip(joint_names, default_dof))

    default_joint_args = ""
    for key, value in default_dict.items():
        default_joint_args += key + ":=" + str(value) + " "

    # ======================================================================== #

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': ParameterValue(Command(['xacro ', xacro_file_path, ' use_gazebo:=True', ' ', default_joint_args]), value_type=str)}
            ],
        ),
        
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', config_file_path],
        ),
        
        Node(
            package='ddp_fishing',
            executable='ddp_controller_publisher_node',
            name='ddp_controller_publisher_node',
            parameters=[config],
            shell=True,
            emulate_tty=True,
            output = 'screen',
            ),
    ])
