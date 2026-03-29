cmake_minimum_required(VERSION 3.14)
project(acb_vehicle_description)

# Vehicle description package for CARLA simulation
# Provides vehicle dimensions, URDF, and visualization

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_package(INSTALL_TO_SHARE
  config
  mesh
  urdf
)
