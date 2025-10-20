#!/bin/bash
set -e

echo "Installing colcon cargo plugins..."
pip install --user git+https://github.com/colcon/colcon-cargo.git
pip install --user git+https://github.com/colcon/colcon-ros-cargo.git

echo "Installing cargo ament build plugin..."
cargo install --locked cargo-ament-build

echo "Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y libclang-dev python3-vcstool

echo "Dependencies installed successfully!"
