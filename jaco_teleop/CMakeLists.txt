cmake_minimum_required(VERSION 2.8.3)
project(jaco_teleop)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  jaco_msgs
  rospy
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 CATKIN_DEPENDS actionlib joy jaco_msgs rospy sensor_msgs topic_tools message_runtime
)

catkin_python_setup()

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

install(PROGRAMS
  src/jaco_teleop/__init__.py
  src/jaco_teleop/joystick_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/jaco_teleop.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
