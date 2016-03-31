cmake_minimum_required(VERSION 2.8.3)
project(kinova_driver)

find_package(catkin REQUIRED dynamic_reconfigure COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  kinova_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
generate_dynamic_reconfigure_options(
  #cfg/KinovaPositionConfig.cfg
)

link_directories(${PROJECT_SOURCE_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES kinova_driver
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs kinova_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#Testers
#add_executable(test_kinova_arm_control src/testers/test_kinova_arm_controller.cpp )
#add_dependencies(test_kinova_arm_control ${PROJECT_NAME}_gencfg)
#add_dependencies(test_kinova_arm_control ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_executable(test_kinova_arm_vel src/testers/test_kinova_arm_vel.cpp )
#add_executable(test_kinova_arm_car_vel src/testers/test_kinova_arm_car_vel.cpp )

# Nodes
# Note, the headers are shown here so they show up in the project view when using
# Qt Creator. There must be a better way...
add_executable(kinova_arm_driver
  src/nodes/kinova_arm_driver.cpp
  src/kinova_api.cpp
  src/kinova_arm.cpp
  src/kinova_comm.cpp
  src/kinova_tool_pose_action.cpp
  src/kinova_joint_angles_action.cpp
  src/kinova_fingers_action.cpp
  src/kinova_ros_types.cpp
)

add_executable(kinova_tf_updater
  src/nodes/kinova_tf_updater.cpp
  src/kinova_arm_kinematics.cpp
)

add_dependencies(kinova_arm_driver ${catkin_EXPORTED_TARGETS})
add_dependencies(kinova_tf_updater ${catkin_EXPORTED_TARGETS})

target_link_libraries(kinova_arm_driver ${catkin_LIBRARIES} dl)
target_link_libraries(kinova_tf_updater ${catkin_LIBRARIES})

#############
## Install ##
#############

install(PROGRAMS
  test/angle_action_client.py
  test/finger_action_client.py
  test/pose_action_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Copy over Kinova .so binary blobs to /usr/lib
install(FILES
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/Kinova.API.CommLayerUbuntu.so
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/Kinova.API.USBCommandLayerUbuntu.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


## Mark executables and/or libraries for installation
install(TARGETS kinova_arm_driver kinova_tf_updater
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/jaco_arm.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kinova_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
#catkin_add_nosetests(test)
