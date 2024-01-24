#!/bin/bash
set -e

vcs import < submodules/ros2.repos src
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro=$ROS_DISTRO
