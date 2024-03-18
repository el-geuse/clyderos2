import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

# WORKS WELL - Spawns Clyde in Gazebo

def generate_launch_description():
    # Find the 'gazebo_ros' package
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    # Find this package (e.g., 'clyde_description')
    pkg_share = FindPackageShare(package='clyde_description').find('clyde_description')

    # Adjusted path to the SDF file
    sdf_file_path = os.path.join(pkg_share, 'src/models/clyde_model/model.sdf')
    urdf_file_path = os.path.join(pkg_share, 'src/urdf/clyde.urdf')

    # Set the GAZEBO_MODEL_PATH to include the path to your models directory
    gazebo_models_path = os.path.join(pkg_share, 'src/models')

    # Set the path to the world file
    world_file_name = 'test-realm1'
    world_path = os.path.join(pkg_share, 'src/worlds/', world_file_name)
    
    # Launch configuration variables
    headless = LaunchConfiguration('headless')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    enable_audio = LaunchConfiguration('enable_audio')

    # Declare the launch arguments
    declare_headless_arg = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_arg = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_world_arg = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world file to load')

    declare_enable_audio_arg = DeclareLaunchArgument(
        name='enable_audio',
        default_value='False',
        description='Enable audio support in Gazebo')

    # Set the GAZEBO_MODEL_PATH environment variable for all child processes
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_models_path
    )

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world, 'enable_audio': enable_audio}.items())

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

    # Spawn entity from SDF file
    spawn_entity_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'clyde',
             '-file', urdf_file_path],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_headless_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_use_simulator_arg)
    ld.add_action(declare_world_arg)
    ld.add_action(declare_enable_audio_arg)
    ld.add_action(set_gazebo_model_path)

    # Add Gazebo server and client startup actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # Add the action to spawn the entity
    ld.add_action(spawn_entity_cmd)

    return ld
