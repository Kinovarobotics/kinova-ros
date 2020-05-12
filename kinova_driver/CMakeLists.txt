cmake_minimum_required(VERSION 2.8.3)
project(kinova_driver)
add_compile_options(-std=c++11)

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
  interactive_markers
  visualization_msgs
)
generate_dynamic_reconfigure_options(
  #cfg/KinovaPositionConfig.cfg
)

link_directories(${PROJECT_SOURCE_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE})

# export library to other packages
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kinova_driver
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs kinova_msgs roscpp rospy sensor_msgs std_msgs tf interactive_markers visualization_msgs
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

add_library(kinova_driver
  src/kinova_api.cpp
  src/kinova_arm.cpp
  src/kinova_comm.cpp
  src/kinova_tool_pose_action.cpp
  src/kinova_joint_angles_action.cpp
  src/kinova_fingers_action.cpp
  src/kinova_ros_types.cpp
  src/kinova_joint_trajectory_controller.cpp
  include/kinova_driver/kinova_joint_trajectory_controller.h
  include/kinova_driver/kinova_joint_angles_action.h
  include/kinova_driver/kinova_api.h
  include/kinova_driver/kinova_arm.h
  include/kinova_driver/kinova_comm.h
  include/kinova_driver/kinova_fingers_action.h
  include/kinova_driver/kinova_tool_pose_action.h
  include/kinova_driver/kinova_ros_types.h
  include/kinova/Kinova.API.USBCommLayerUbuntu.h
  include/kinova/Kinova.API.USBCommandLayerUbuntu.h
  include/kinova/KinovaTypes.h
)

add_executable(kinova_arm_driver
  src/nodes/kinova_arm_driver.cpp
)

add_executable(joint_trajectory_action_server 
  src/joint_trajectory_action/joint_trajectory_action_server.cpp
  include/kinova_driver/joint_trajectory_action_server.h)
target_link_libraries(joint_trajectory_action_server 
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS joint_trajectory_action_server 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(joint_trajectory_action_server ${catkin_EXPORTED_TARGETS})

add_executable(gripper_command_action_server 
  src/joint_trajectory_action/gripper_command_action_server.cpp 
  include/kinova_driver/gripper_command_action_server.h)
target_link_libraries(gripper_command_action_server 
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})
install(TARGETS gripper_command_action_server 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(gripper_command_action_server ${catkin_EXPORTED_TARGETS})

add_executable(kinova_tf_updater
  src/kinova_ros_types.cpp
  src/nodes/kinova_tf_updater.cpp
  src/kinova_arm_kinematics.cpp
  include/kinova_driver/kinova_ros_types.h
  include/kinova_driver/kinova_tf_updater.h
  include/kinova_driver/kinova_arm_kinematics.h
)
add_dependencies(kinova_driver ${catkin_EXPORTED_TARGETS})
add_dependencies(kinova_arm_driver ${catkin_EXPORTED_TARGETS})
add_dependencies(kinova_tf_updater ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinova_driver ${catkin_LIBRARIES} dl)
#target_link_libraries(kinova_arm_driver ${catkin_LIBRARIES} dl)
target_link_libraries(kinova_arm_driver ${catkin_LIBRARIES} kinova_driver)
target_link_libraries(kinova_tf_updater ${catkin_LIBRARIES})


add_executable(kinova_interactive_control src/nodes/kinova_interactive_control.cpp)
add_dependencies(kinova_interactive_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinova_interactive_control ${catkin_LIBRARIES} kinova_driver)

#############
## Install ##
#############

# Copy over Kinova .so binary blobs to /usr/lib
install(FILES
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/USBCommLayerUbuntu.so
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/USBCommandLayerUbuntu.so
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/EthCommLayerUbuntu.so
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/EthCommandLayerUbuntu.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


## Mark executables and/or libraries for installation
install(TARGETS kinova_arm_driver kinova_tf_updater
kinova_interactive_control
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
#  install(FILES
## myfile1
## myfile2
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}

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
