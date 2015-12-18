cmake_minimum_required(VERSION 2.8.3)
project(jaco_driver)

find_package(catkin REQUIRED dynamic_reconfigure COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  jaco_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  ecl_geometry
)
generate_dynamic_reconfigure_options(
  #cfg/JacoPositionConfig.cfg
)

link_directories(${PROJECT_SOURCE_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES jaco_driver
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs jaco_msgs roscpp rospy sensor_msgs std_msgs tf
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
#add_executable(test_jaco_arm_control src/testers/test_jaco_arm_controller.cpp )
#add_dependencies(test_jaco_arm_control ${PROJECT_NAME}_gencfg)
#add_dependencies(test_jaco_arm_control ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_executable(test_jaco_arm_vel src/testers/test_jaco_arm_vel.cpp )
#add_executable(test_jaco_arm_car_vel src/testers/test_jaco_arm_car_vel.cpp )

# Nodes
# Note, the headers are shown here so they show up in the project view when using
# Qt Creator. There must be a better way...
add_executable(jaco_arm_driver
  src/nodes/jaco_arm_driver.cpp
  src/jaco_api.cpp
  src/jaco_arm.cpp
  src/jaco_comm.cpp
  src/jaco_pose_action.cpp
  src/jaco_angles_action.cpp
  src/jaco_trajectory_angles_action.cpp
  src/jaco_fingers_action.cpp
  src/jaco_gripper_action_server.cpp
  src/jaco_types.cpp
  include/jaco_driver/jaco_angles_action.h
  include/jaco_driver/jaco_api.h
  include/jaco_driver/jaco_arm.h
  include/jaco_driver/jaco_comm.h
  include/jaco_driver/jaco_fingers_action.h
  include/jaco_driver/jaco_gripper_action_server.h
  include/jaco_driver/jaco_trajectory_angles_action.h
  include/jaco_driver/jaco_pose_action.h
  include/jaco_driver/jaco_types.h
  include/kinova/Kinova.API.CommLayerUbuntu.h
  include/kinova/Kinova.API.UsbCommandLayerUbuntu.h
  include/kinova/KinovaTypes.h
)

add_executable(jaco_tf_updater
  src/nodes/jaco_tf_updater.cpp
  src/jaco_arm_kinematics.cpp
  include/jaco_driver/jaco_tf_updater.h
  include/jaco_driver/jaco_arm_kinematics.h
)

add_dependencies(jaco_arm_driver ${catkin_EXPORTED_TARGETS})
add_dependencies(jaco_tf_updater ${catkin_EXPORTED_TARGETS})

target_link_libraries(jaco_arm_driver ${catkin_LIBRARIES} dl)
target_link_libraries(jaco_tf_updater ${catkin_LIBRARIES})

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
install(TARGETS jaco_arm_driver jaco_tf_updater
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_jaco_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
#catkin_add_nosetests(test)
