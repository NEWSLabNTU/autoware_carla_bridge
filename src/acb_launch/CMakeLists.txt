cmake_minimum_required(VERSION 3.14)
project(acb_launch)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# carla_simulator.launch.xml: the Autoware profile (autoware_sim_profile) and acb_bridge.
# config/blueprint/<vehicle_model>/: the pedal maps of the car's CARLA blueprint under its own
# physics, which the launch takes when carla_physics is none.
ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
