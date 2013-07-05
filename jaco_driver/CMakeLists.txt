cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE})

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#add dynamic reconfigure api
#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()


rosbuild_add_library(jacoapi src/jaco_api.cpp)
rosbuild_add_library(jacokinematics src/jaco_arm_kinematics.cpp)

rosbuild_add_library(jaco_arm_driver src/jaco_arm.cpp)
rosbuild_add_library(jaco_arm_driver src/jaco_comm.cpp)
rosbuild_add_library(jaco_arm_driver src/jaco_pose_action.cpp)
rosbuild_add_library(jaco_arm_driver src/jaco_types.cpp)

#Testers
#rosbuild_add_executable(test_jaco_arm_control src/testers/test_jaco_arm_controller.cpp )
#rosbuild_add_executable(test_jaco_arm_vel src/testers/test_jaco_arm_vel.cpp )
#rosbuild_add_executable(test_jaco_arm_car_vel src/testers/test_jaco_arm_car_vel.cpp )


#Nodes
rosbuild_add_executable(jaco_arm_driver src/nodes/jaco_arm_driver.cpp )
rosbuild_add_executable(jaco_tf_updater src/nodes/jaco_tf_updater.cpp )


target_link_libraries(jaco_arm_driver jacoapi)
target_link_libraries(jaco_tf_updater jacokinematics)

