cmake_minimum_required(VERSION 2.8.3)
project(jaco_driver)

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
)
generate_dynamic_reconfigure_options(
  #cfg/JacoPositionConfig.cfg
)

link_directories(${PROJECT_SOURCE_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   actionlib_msgs   geometry_msgs   jaco_msgs   sensor_msgs   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES jaco_driver
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs jaco_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(jaco_driver
#   src/${PROJECT_NAME}/jaco_driver.cpp
# )

add_library(jacoapi src/jaco_api.cpp)
add_library(jacokinematics src/jaco_arm_kinematics.cpp)
add_library(jaco_arm src/jaco_arm.cpp)
add_library(jaco_comm src/jaco_comm.cpp)
add_library(jaco_pose src/jaco_pose_action.cpp)
add_library(jaco_angles src/jaco_angles_action.cpp)
add_library(jaco_fingers src/jaco_fingers_action.cpp)
add_library(jaco_types src/jaco_types.cpp)

add_dependencies(jacoapi ${catkin_EXPORTED_TARGETS})
add_dependencies(jaco_comm ${catkin_EXPORTED_TARGETS})
add_dependencies(jaco_types ${catkin_EXPORTED_TARGETS})
add_dependencies(jaco_pose ${catkin_EXPORTED_TARGETS})
add_dependencies(jaco_angles ${catkin_EXPORTED_TARGETS})
add_dependencies(jaco_fingers ${catkin_EXPORTED_TARGETS})
add_dependencies(jaco_arm ${catkin_EXPORTED_TARGETS})

#Testers
#add_executable(test_jaco_arm_control src/testers/test_jaco_arm_controller.cpp )
#add_dependencies(test_jaco_arm_control ${PROJECT_NAME}_gencfg)
#add_dependencies(test_jaco_arm_control ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_executable(test_jaco_arm_vel src/testers/test_jaco_arm_vel.cpp )
#add_executable(test_jaco_arm_car_vel src/testers/test_jaco_arm_car_vel.cpp ) 

#Nodes
add_executable(jaco_arm_driver src/nodes/jaco_arm_driver.cpp )
add_executable(jaco_tf_updater src/nodes/jaco_tf_updater.cpp )

#add_dependencies(jaco_arm_driver ${catkin_EXPORTED_TARGETS})
#add_dependencies(jaco_tf_updater ${catkin_EXPORTED_TARGETS})

target_link_libraries(jaco_arm_driver jacoapi)
target_link_libraries(jaco_comm jacoapi)
target_link_libraries(jaco_arm_driver jaco_arm)
target_link_libraries(jaco_arm_driver jaco_comm)
target_link_libraries(jaco_arm_driver jaco_pose)
target_link_libraries(jaco_arm_driver jaco_angles)
target_link_libraries(jaco_arm_driver jaco_fingers)
target_link_libraries(jaco_arm_driver jaco_types)
target_link_libraries(jaco_tf_updater jacokinematics)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

## Specify libraries to link a library or executable target against
 target_link_libraries(jaco_arm_driver
   ${catkin_LIBRARIES}
 )

 target_link_libraries(jaco_tf_updater
   ${catkin_LIBRARIES}	
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
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
   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
   launch/jaco_arm.launch
   # myfile2
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
