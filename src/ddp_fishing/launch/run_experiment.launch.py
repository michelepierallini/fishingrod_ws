from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
        
    workspace_path = f"{get_package_share_directory('ddp_fishing')}/../../../../../tasks/"
    time = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    
    # Load the path to the parameters file
    config = os.path.join(
        get_package_share_directory('ddp_fishing'),
        'config',
        'ddp_controller_params.yaml'
    )
        
    # ======================================================================== #
    
    ddp_controller_pub = Node(
        package='ddp_fishing',
        executable='ddp_controller_publisher_node',
        name='ddp_controller_publisher_node',
        parameters=[config],
        shell=True,
        emulate_tty=True,
        output = 'screen',
    )
    
    bag_recorder = Node(
        package='ddp_fishing',
        executable='bag_recorder',
        name='bag_recorder',
        parameters=[
            config,
            {'time': time},
        ],
        shell=True,
        emulate_tty=True,
        output = 'screen',
    )

    return LaunchDescription([
        ddp_controller_pub,
        bag_recorder
    ])
