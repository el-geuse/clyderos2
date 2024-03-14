from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

# WORKS - Displays Clyde in rviz

# displays Clyde inside RViz
def generate_launch_description():
    pkg_share = FindPackageShare(package='clyde_description')\
        .find('clyde_description')
    urdf_path = os.path.join(pkg_share, 'src/urdf/clyde.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz/test_config.rviz')

    declare_gui_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='False',
        description='Flag to enable joint_state_publisher_gui')
    
    declare_model_cmd = DeclareLaunchArgument(
        name='model',
        default_value=urdf_path,
        description='Absolute path to robot urdf file')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_path],
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    ld = LaunchDescription()

    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_model_cmd)

    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld   