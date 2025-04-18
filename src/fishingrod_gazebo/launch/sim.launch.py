import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
import launch_ros.descriptions
import launch


def generate_launch_description():

    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf = os.path.join(
        get_package_share_directory('fishingrod_description'),
        'urdf',
        'fishingrod.urdf'
    )

    print("urdf_file_name : {}".format(urdf))

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
            ),
            launch_arguments={"pause": "true", "verbose": "false"}.items(),
    )

    # default_dof = ( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    #               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    # joint_names = ('Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Joint_6', 
    #     'Joint_7', 'Joint_8', 'Joint_9', 'Joint_10', 'Joint_11', 'Joint_12', 
    #     'Joint_13', 'Joint_14', 'Joint_15', 'Joint_16', 'Joint_17', 'Joint_18', 'Joint_19', 'Joint_20', 'Joint_21')
    
    # default_dof = (0.0,)
    # joint_names = ('Joint_1',)
    # default_dict = dict(zip(joint_names, default_dof))

    # default_joint_args = ""
    # for key, value in default_dict.items():
    #     default_joint_args += key + ":=" + str(value) + " "
    #     # print(default_joint_args)

    # Get URDF via xacro
    # fishingrod_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="urdf")]),
    #         " ", urdf, " use_gazebo:=true ",
    #         default_joint_args
    #     ]
    # )
        
    fishingrod_description_content = open(os.path.join(get_package_share_directory('fishingrod_description'),
                                                'urdf',
                                                'fishingrod.urdf'
                                                )).read()
        
    robot_description = {"robot_description": fishingrod_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "fishingrod",
                    "-x", "0.0", "-y", "0.0", "-z", "0.0",
                    ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    PD_jnt_control = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["PD_control", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[PD_jnt_control],
            )
        ),
  ])