import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find directories and files
    pkg_share = FindPackageShare(package='clyde_description').find('clyde_description')
    pkg_nav = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    launch_dir = os.path.join(pkg_nav, 'launch')

    # Configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam', default='false')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml') # Path to your modified nav2 params file

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_slam = DeclareLaunchArgument(
        'slam', default_value='false',
        description='Whether to run SLAM')

    # SLAM Toolbox node (optional, depending on your needs)
    start_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'slam_toolbox_launch.py')),
        condition=IfCondition(slam))

    # Navigation2 bringup
    start_nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'params_file': params_file}.items())

    ld = LaunchDescription()

    # Add actions to the launch description
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_slam)
    ld.add_action(start_slam_toolbox)
    ld.add_action(start_nav2_bringup)

    return ld
