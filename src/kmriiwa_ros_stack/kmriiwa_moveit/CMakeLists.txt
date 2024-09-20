cmake_minimum_required(VERSION 3.1.3)
project(kmriiwa_moveit)

# find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs)
# catkin_package( )
catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/merge_joint_states.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)