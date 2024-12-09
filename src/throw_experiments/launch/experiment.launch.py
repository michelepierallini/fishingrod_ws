import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,RegisterEventHandler, LogInfo, EmitEvent
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from datetime import datetime


def generate_launch_description():
        
        current_date = datetime.now()
        day = current_date.day
        month = current_date.month
        formatted_date = f"{day:02d}_{month:02d}"

        csv = LaunchConfiguration('csv', default='throw_experiments_data.csv')
        csv_declare = DeclareLaunchArgument(
                'csv',
                default_value='throw_experiments_data.csv',
                description='csv file to save info, including extension. Default: throw_experiments_data.csv'
                )

        exp = LaunchConfiguration('exp', default=formatted_date)
        exp_declare = DeclareLaunchArgument(
                'exp',
                default_value='experiment',
                description='experiment name'
                )

        ## why was 'vel' here ???
        pos_tip = LaunchConfiguration('pos_des', default=0.4) 
        
        pos_tip_declare = DeclareLaunchArgument(
                'pos_tip',
                default_value=formatted_date,
                description='pos_tip'
                )
        
                
        duration = LaunchConfiguration('duration', default=2.0)
        duration_declare = DeclareLaunchArgument(
                'duration',
                default_value=formatted_date,
                description='duration'
                )
        
        
        throw_node=Node(
                package = 'throw_experiments',
                name = 'throw_node',
                executable = 'throw_node',
                parameters =    [{'publication_rate': 200},
                                {'duration': duration},
                                {'start_delay': 6.0},
                                {'pos_tip_des': pos_tip}],
        )

        policy = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        [PathJoinSubstitution([FindPackageShare("rlg_quad_controller"), "launch", "softleg_inference.launch.py"])]
                ),
        )

        save_csv_process = ExecuteProcess(
                cmd=[
                    'echo', exp, pos_tip, duration, '>>', csv
                ]

        )
        bag_process = ExecuteProcess(
                # cmd=['ros2', 'bag', 'record', '-a', '-o', exp, '-s', 'sqlite3'],
                cmd=['ros2', 'bag', 'record', '-a', '-o', exp, '-s', 'mcap'],
                output='screen'
        )

        shutdown_event = RegisterEventHandler(
                event_handler= OnProcessExit(
                target_action=bag_process,
                on_exit=[
                LogInfo(
                        msg="BAG NODE CRASHED. STOPPING EXPERIMENT."),
                EmitEvent(
                        event=Shutdown())]))
        
        return LaunchDescription([
                exp_declare, 
                duration_declare,
                pos_tip_declare,
                csv_declare,
                save_csv_process,
                bag_process,
                throw_node,
                policy,
                shutdown_event,
        ])