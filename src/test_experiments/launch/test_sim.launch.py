import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():
        
        params = os.path.join(
                        get_package_share_directory('rlg_quad_controller'),
                        'config',
                        'fishingrod_simulation_config.yaml'
                        )

        test_node=Node(
                package = 'test_experiments',
                name = 'test_node',
                executable = 'test_node',
                parameters = [params,
                        {'trajectory': 'const'},
                        {'publication_rate': 1000},
                        {'duration': 5.0},
                        {'start_delay': 1.0}, 
                        {'simulation': True},
                        {'use_sim_time': False}]
        )


        return LaunchDescription([
                ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-a', '-o', 'bag_to_delete', '-s', 'mcap'],
                output='screen'
                        ),
                test_node,                
        ])