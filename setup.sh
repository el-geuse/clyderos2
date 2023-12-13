#!/bin/bash
set -e

vcs import < ros2.repos
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y
