cmake_minimum_required(VERSION 3.14)
project(carla_sensor_kit_launch)

# Sensor kit launch for CARLA simulation
# Provides sensor processing pipelines (bridge publishes raw data)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_package(INSTALL_TO_SHARE
  config  # Diagnostic and processing configs
  data    # (if any additional data files)
  launch  # Sensor processing launch files
)
