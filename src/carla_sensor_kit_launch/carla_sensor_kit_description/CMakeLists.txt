cmake_minimum_required(VERSION 3.14)
project(carla_sensor_kit_description)

# Sensor kit description for CARLA simulation
# Provides sensor URDF and TF calibrations (bridge reads to spawn sensors)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_package(INSTALL_TO_SHARE
  config  # sensor_kit_calibration.yaml - sensor TF transforms
  urdf    # sensor_kit.xacro - sensor link definitions
)
