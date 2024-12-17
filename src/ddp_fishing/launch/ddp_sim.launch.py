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
                        get_package_share_directory('ddp_fishing'),
                        'config',
                        'ddp_controller_params.yaml'
                        )

        test_node=Node(
                package = 'ddp_fishing',
                name = 'test_node',
                executable = 'test_node',
                parameters = [params,
                        {'publication_rate': 1000},
                        {'duration': 5.0},
                        {'start_delay': 1.0}, 
                        {'simulation': True},
                        {'use_sim_time': False}]
        )
        
        bag_dir = os.path.join(get_package_share_directory('ddp_fishing'), 'bags')
        os.makedirs(bag_dir, exist_ok=True)

        record_bag = ExecuteProcess(
                cmd = [
                'ros2', 'bag', 'record', '-o', os.path.join(bag_dir, 'recorded_data'),
                '-a', '--storage', 'mcap'
                ],
                output='screen'
        )

        return LaunchDescription([
                test_node,  
                # record_bag,              
        ])