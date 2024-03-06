#!/bin/bash
set -e

./setup.sh
ament_${LINTER} --config .vscode/.flake8 src/
