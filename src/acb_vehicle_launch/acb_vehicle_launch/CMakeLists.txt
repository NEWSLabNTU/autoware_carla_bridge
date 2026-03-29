cmake_minimum_required(VERSION 3.14)
project(acb_vehicle_launch)

# Vehicle interface launch package for CARLA simulation
# Provides minimal launch files (bridge handles control directly)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
