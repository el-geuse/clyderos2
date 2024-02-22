import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

# test launch script to spawn Clyde in Gazebo

#TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = 'clyde'
    world_file_name = 'new-env0'
    pkg_dir = get_package_share_directory("clyde_description")
    
    world = os.path.join(pkg_dir, 'src/worlds', world_file_name)

    urdf = os.path.join(pkg_dir, 'src/urdf', 'clyde_simple.urdf')

    xml = open(urdf, 'r').read()

    xml = xml.replace('"', '\\"')

    spawn_args = '{name: \"clyde\", xml: \"'  +  xml + '\" }'

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spawn_args],
            output='screen'),
    ])
