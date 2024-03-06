#!/bin/bash
set -e

./setup.sh

if [ "$LINTER" == "flake8" ]; then
    # Ignores some additional rules that come with ROS 2 files (E111, E114)
    ament_flake8 --config .vscode/.flake8 src/
else
    # Replace this line with the command you want to run for other values of LINTER
    ament_${LINTER} src/
fi