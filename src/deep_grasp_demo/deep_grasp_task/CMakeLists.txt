cmake_minimum_required(VERSION 3.0.2)
project(deep_grasp_task)

# C++ 14
add_compile_options(-std=c++14)

# Warnings
add_definitions(-W -Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	actionlib
	moveit_core
	moveit_ros_planning_interface
	moveit_task_constructor_core
	moveit_task_constructor_msgs
  roscpp
  std_msgs
  moveit_task_constructor_gpd
	rosparam_shortcuts
  tf2_geometry_msgs
)

###################################
## Catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
	moveit_task_constructor_msgs
 	roscpp
  std_msgs
  tf2_geometry_msgs
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(deep_grasp_demo_sim
 							 src/deep_grasp_demo_sim.cpp
							 src/deep_pick_place_task_sim.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(deep_grasp_demo_sim
                      ${catkin_LIBRARIES}
)

# Declare a C++ executable
add_executable(deep_grasp_demo_real
 							 src/deep_grasp_demo_real.cpp
							 src/deep_pick_place_task_real.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(deep_grasp_demo_real
                      ${catkin_LIBRARIES}
)

add_executable(test2
 							 src/test.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(test2
                      ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    deep_grasp_demo_sim
    deep_grasp_demo_real
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/tf_transform_cylinder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# # Mark roslaunch files for installation
# install(
#   DIRECTORY
#     launch
#   DESTINATION
#     ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
#
# # Mark config files for installation
# install(
#   DIRECTORY
#     config
#   DESTINATION
#     ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
