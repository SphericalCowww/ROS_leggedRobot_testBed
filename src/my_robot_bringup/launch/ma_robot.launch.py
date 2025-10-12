from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

######################################################################################################################
def generate_launch_description():
    robot_description_path   = get_package_share_path('my_robot_description')
    robot_bringup_path       = get_package_share_path('my_robot_bringup')
    robot_moveit_config_path = get_package_share_path('ma_robot_moveit_config')   
    urdf_path          = os.path.join(robot_description_path,   'urdf',   'ma_robot.urdf.xacro')
    robot_controllers  = os.path.join(robot_bringup_path,       'config', 'ma_robot_controllers.yaml')
    moveit_config_path = os.path.join(robot_moveit_config_path, 'launch', 'move_group.launch.py')
    rviz_config_path   = os.path.join(robot_description_path,   'rviz',   'ma_robot.urdf_config.rviz')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
    )
    # check src/my_robot_bringup/config/ma_robot_controllers.yaml 
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
    )
    
    moveit_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_config_path),
        launch_arguments={}.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        moveit_launcher,
        rviz_node,
    ])

######################################################################################################################
