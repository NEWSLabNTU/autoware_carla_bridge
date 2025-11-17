#!/bin/bash
set -e

echo "Installing colcon-cargo-ros2 plugin..."
pip install --user colcon-cargo-ros2

echo "Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y libclang-dev python3-vcstool

echo "Dependencies installed successfully!"
