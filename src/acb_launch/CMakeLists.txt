cmake_minimum_required(VERSION 3.14)
project(acb_launch)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# `ros2 run acb_launch setup_autoware_data` -- mirrors Autoware's packaged data into a
# writable data_path (see the script's header).
install(PROGRAMS scripts/setup_autoware_data DESTINATION lib/${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
