#!/bin/bash
# Entrypoint for ROS / Colcon Docker containers
 
# Source ROS 2
# source /opt/ros/${ROS_DISTRO}/setup.bash
# (Already sourced in base docker image)

# Required Husarnet ROS 2 variables
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/clyderos2/src/clyde_teleop/config/fastdds-ds-client.xml
export DISCOVERY_SERVER_PORT=11811
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=clydepc:11811

# Source Gazebo
echo "source /usr/share/gazebo/setup.bash" >> /home/ros/.bashrc

# Command to start server
echo "alias serverstart='fastdds discovery -x /workspaces/clyderos2/src/clyde_teleop/config/fastdds-ds-server.xml'" >> /home/ros/.bashrc

# With FastDDS discovery v2, ROS introspection tools do not work, as they require a node to be made,
# which then can't read the restricted topic. Running this command beforehand bypasses that.
echo "alias superclient='export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/clyderos2/src/clyde_teleop/config/super_client_configuration_file.xml'" >> /home/ros/.bashrc

