cmake_minimum_required(VERSION 2.8.3)
project(jaco_teleop)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  jaco_msgs
  rospy
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 CATKIN_DEPENDS actionlib geometry_msgs joy jaco_msgs rospy std_msgs sensor_msgs topic_tools message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS
  nodes/jaco_teleop_joy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/nodes
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/jaco_teleop.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
