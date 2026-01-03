from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
import os

######################################################################################################################
def generate_launch_description():
    robot_description_path   = get_package_share_path('my_robot_description')
    robot_bringup_path       = get_package_share_path('my_robot_bringup')
    robot_moveit_config_path = get_package_share_path('my_robot_moveit_config')   
 
    urdf_path          = os.path.join(robot_description_path,   'urdf',   'my_robot.urdf.xacro')
    robot_controllers  = os.path.join(robot_bringup_path,       'config', 'my_robot_controllers.yaml')
    moveit_config_path = os.path.join(robot_moveit_config_path, 'launch', 'move_group.launch.py')
    rviz_config_path   = os.path.join(robot_description_path,   'rviz',   'my_robot.urdf_config.rviz')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_controllers,
            {'use_steady_clock_for_update': False},],
    )
    # check src/my_robot_bringup/config/my_robot_controllers.yaml 
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    leg1_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg1_controller"],
    )
    
    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="my_robot_moveit_config")
        .robot_description(file_path=urdf_path)
        .to_moveit_configs()
    )
    moveit_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_config_path),
        launch_arguments={}.items(),
    )
    commander_node = Node(
        package="my_robot_commander",
        executable="my_robot_commander",
        parameters=[
            moveit_config.robot_description,           # The URDF math
            moveit_config.robot_description_semantic,  # The SRDF (defines 'leg1')
            moveit_config.robot_description_kinematics,# The kinematics.yaml (THE MISSING PIECE)
            {"jump_threshold": 0.15},                  # for computeCartesianPath
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )
    
    return LaunchDescription([
        SetParameter(name='jump_threshold', value=0.15),
        SetParameter(name='use_sim_time', value=False),
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        leg1_controller_spawner,
        moveit_launcher,
        commander_node,
        rviz_node,
    ])

######################################################################################################################
