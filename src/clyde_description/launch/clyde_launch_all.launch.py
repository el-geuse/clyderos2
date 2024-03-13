import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# CURRENT DOESNT WORK

def generate_launch_description():
    # Set the path to the Gazebo ROS package
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    # Set the path to this package.
    pkg_share = FindPackageShare(package='clyde_description').find('clyde_description')

    # Set the path to the world file
    env_file_name = 'test-realm1'
    world_path = os.path.join(pkg_share, 'src/worlds/', env_file_name)

    # Set the path to the map file
    map_path = os.path.join(pkg_share, 'src/maps/test-realm1.yaml')

    # Set the path to the URDF file
    urdf_file_name = 'clyde.urdf'
    urdf_path = os.path.join(pkg_share, 'src/urdf/', urdf_file_name)

    # Set the path to the RViz configuration file
    rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    # Set the path to the SDF model files.
    gazebo_models_path = os.path.join(pkg_share, 'src/models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # Launch configuration variables specific to simulation
    headless = LaunchConfiguration('headless')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    enable_audio = LaunchConfiguration('enable_audio')

    # Declare the launch arguments
    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')

    declare_enable_audio_cmd = DeclareLaunchArgument(
        name='enable_audio', 
        default_value='False',
        description='Enable audio support in Gazebo')
    
    declare_gui_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='False',
        description='Flag to enable joint_state_publisher_gui')
    
    declare_model_cmd = DeclareLaunchArgument(
        name='model',
        default_value=urdf_path,
        description='Absolute path to robot urdf file')

    declare_map_cmd = DeclareLaunchArgument(
        name='map',
        default_value=map_path,
        description='Absolute path to map yaml file')


    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world, 'enable_audio': enable_audio}.items())

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

    # Spawn entity
    spawn_entity_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'clyde',
             '-topic', '/robot_description'],
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path, 'r').read()}]
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

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': LaunchConfiguration('map')}],
        output='screen',
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_enable_audio_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_model_cmd)
    ld.add_action(declare_map_cmd)

    # Add any actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(rviz_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(map_server_node)
    ld.add_action(spawn_entity_cmd)

    return ld
