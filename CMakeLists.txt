cmake_minimum_required(VERSION 3.0.2)
project(mren_waste_collector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  roscpp
  unitree_legged_msgs
  cv_bridge
  sensor_msgs
  image_transport
)

## Find the non-catkin SDK package
find_package(unitree_legged_sdk REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
# ... (message/service/action sections omitted for brevity) ...
## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
# ... (dynamic reconfigure sections omitted for brevity) ...
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )



#--- Add this for the C++ teleop node ---
add_executable(go1_teleop_key_node src/go1_teleop_key.cpp)
add_dependencies(go1_teleop_key_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(go1_teleop_key_node
  ${catkin_LIBRARIES}
  ${unitree_legged_sdk_LIBRARIES}
)
install(TARGETS go1_teleop_key_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
set_target_properties(go1_teleop_key_node
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}"
)
#--- End of new section ---

#--- Add this for the C++ simple servoing node ---
add_executable(simple_servoing_node src/simple_servoing_node.cpp)
add_dependencies(simple_servoing_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_servoing_node
  ${catkin_LIBRARIES}
  ${unitree_legged_sdk_LIBRARIES}
)
install(TARGETS simple_servoing_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
set_target_properties(simple_servoing_node
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}"
)
#--- End of new section ---




###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mren_waste_collector
  CATKIN_DEPENDS
    geometry_msgs
    rospy
    roscpp
    unitree_legged_msgs
    cv_bridge
    sensor_msgs
    image_transport
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${unitree_legged_sdk_INCLUDE_DIRS}
)
# ... (library/executable sections omitted for brevity) ...

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/webcam_publisher.py
  scripts/realsense_publisher.py
  scripts/wasd_teleop.py
  scripts/orange_error_publisher.py
  scripts/turtlesim_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
# ... (install executable sections omitted for brevity) ...
## Mark libraries for installation
# ... (install library sections omitted for brevity) ...
## Mark cpp header files for installation
# ... (install header sections omitted for brevity) ...
## Mark other files for installation (e.g. launch and bag files, etc.)
# ... (install other sections omitted for brevity) ...

#############
## Testing ##
#############
# ... (testing sections omitted for brevity) ...