import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node

######### the model are in /home/michele/policy_to_test
def generate_launch_description():
    ld = LaunchDescription()

    params = os.path.join(
        get_package_share_directory('rlg_quad_controller'),
        'config',
        'fishingrod_simulation_config.yaml'
        )
    
    config_path = os.path.join(
        get_package_share_directory('rlg_quad_controller'),
        'models',
        'config.yaml'
        )
    
    weights_path = os.path.join(
        get_package_share_directory('rlg_quad_controller'),
        'models',
        'FishingRodPos_X_022_real_pos_vel.pth'
        )
    
    node=Node(
        package = 'rlg_quad_controller',
        name = 'inference_controller',
        executable = 'inference_controller',
        parameters = [params,
                      {'config_path': config_path},
                      {'model_path': weights_path}]
    )
    
    bag_dir = os.path.join(get_package_share_directory('rlg_quad_controller'), 'bags')
    os.makedirs(bag_dir, exist_ok=True)

    record_bag = ExecuteProcess(
            cmd = [
            'ros2', 'bag', 'record', '-o', os.path.join(bag_dir, 'recorded_data'),
            '-a', '--storage', 'mcap'
            ],
            output='screen'
    )

    ld.add_action(node)
    # ld.add_action(record_bag)
    return ld