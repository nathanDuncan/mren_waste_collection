cmake_minimum_required(VERSION 3.0.2)
project(unified_launcher)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/manipulator_preset_controller.py
  scripts/manipulator_direct_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
