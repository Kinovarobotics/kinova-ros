cmake_minimum_required(VERSION 3.0.2)
project(uml_hri_nerve_pick_and_place)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  geometry_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

catkin_package(
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(basic_manipulation_and_2d_perception
	src/nodes/basic_manipulation_and_2d_perception.cpp
	src/functions/manipulation_class.cpp
	src/functions/perception_class.cpp)

target_link_libraries(basic_manipulation_and_2d_perception ${catkin_LIBRARIES})

add_executable(basic_manipulation_and_3d_perception
	src/nodes/basic_manipulation_and_3d_perception.cpp
	src/functions/manipulation_class.cpp
	src/functions/perception_class.cpp)

target_link_libraries(basic_manipulation_and_3d_perception ${catkin_LIBRARIES})
