import imp
import os
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command,LaunchConfiguration,PathJoinSubstitution
from launch.conditions import IfCondition,UnlessCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    #path for build fishingrod urdf
    fishingrod_robot_path = get_package_share_path("fishingrod_description")
    fishingrod_robot_path = os.path.join(fishingrod_robot_path,"urdf", "fishingrod.urdf.xacro") 
    # #path for rviz settings
    rviz_config_path = get_package_share_path("fishingrod_description")
    rviz_config_path = os.path.join(rviz_config_path, "rviz", "config.rviz")
    # world file
    world_file = get_package_share_path("fishingrod_description")
    world_file = os.path.join(
        # world_file, "world", "empty.world"
        world_file,"world","gap.world"
    )

    #declaration argument of launch
    use_gui = DeclareLaunchArgument(
        name="use_gui",
        default_value="true",
        description="Value use to enable joint publisher with GUI")

    fishingrod_model = DeclareLaunchArgument(
        name="fishingrod_urdf",
        default_value=str(fishingrod_robot_path)
    )

    # rviz_arg = DeclareLaunchArgument(
    #     name="rviz_config",
    #     default_value= str(rviz_config_path),
    #     description="configuration of Rviz for plot"
    # )

    #use command to create a parameter with urdf of fishingrod by xacro file
    robot_description = ParameterValue(
        Command(["xacro ",LaunchConfiguration("fishingrod_urdf")]),
        value_type=str
    )

    #node declaration
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    #launch gazebo 
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose','--pause' ,'-s', 'libgazebo_ros_factory.so', world_file], 
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'fishingrod', '-x', '0', '-y', '0', '-z', '0.12'],
        output='screen'
    )

    return LaunchDescription(
        [
            fishingrod_model,
            robot_state_pub,
            gazebo,
            spawn_entity
        ]
    )