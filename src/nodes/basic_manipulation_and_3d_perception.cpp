// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// basic_manipulation_and_3d_perception.cpp
//
// manipulation_class & perception_class node
//
// Utilizing the simulated ARMada workstation and a simulated robot, use any vision systems available
// to perform a very simple task - move the robot to a few set positions (created in the robot's moveit
// config package in its SRDF) and then concatenate the pointclouds and publish the result on a new topic
// // This node focuses on 3D vision using PCL
// ********************************************************************************************

#include "manipulation_class.hpp"
#include "perception_class.hpp"

int main(int argc, char** argv, string planning_group)
{
  // ros initialization
  ros::init(argc,argv,"basic_manipulation_and_3d_perception_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();  

  // Create manipulation and perception objects
  Manipulation manipulation(nh, planning_group);
  ROS_INFO("planning group is: ", planning_group);
  Perception perception(nh);

  // Wait for spinner to start
  ros::Duration(1.0).sleep();

  // Transform listener
  perception.transform_listener_ptr = TransformListenerPtr(
      new tf::TransformListener());
  perception.init_subscriber(nh);

  // Planning scene interface
  manipulation.planning_scene_ptr = PlanningScenePtr(
      new moveit::planning_interface::PlanningSceneInterface());

  // Moveit interface
  manipulation.move_group_ptr = MoveGroupPtr(
      new moveit::planning_interface::MoveGroupInterface(manipulation.PLANNING_GROUP));
  
  // Set useful variables before robot manipulation begins
  manipulation.move_group_ptr->setPlanningTime(45.0);			// This will give the robot the opportunity to plan for a while
  manipulation.move_group_ptr->setMaxVelocityScalingFactor(0.25);	// This will limit the robot to moving at 1/4 its max speed (they can go very fast)
  manipulation.move_group_ptr->setPoseReferenceFrame("world");		// We want to establish that the common reference frame is "world" for any path planning
  manipulation.move_group_ptr->setPlannerId("RRTConnect");		// There are numerous default planners, you can experiment or research which will suit your needs

  // Move robot into starting position and wait a moment
  // You can create a function within the Manipulation class to move the robot to a default position or one specified within the SRDF
  // You can find the SRDF in the following location for robots with a moveit_config package typically:
  // <robot_name>_moveit_config >> config >> <robot_name>.srdf
  // 
  // You can use either an existing position (typically a position called "Home" will exist) or you can create your own
  // This is also usually performed when making a custom moveit_config package from within the moveit_setup_assistant
  // Although these positions are frequently automatically generated after you use the visualizer to move the robot to certain 
  //   joint positions, you can also do this by hand by manually editing the SRDF
  // 
  // You may end up with something like this:
  // manipulation.go_to_home();
  // ros::Duration(1).sleep();
  //
  // The go_to_home() function does not currently exist, but you could create it within the manipulation_class header and cpp file
  
  while(ros::ok())
  {
    // Put some code in here!
    // The purpose of this node is to use 3d vision to accomplish a simple pick and place task
    // You will probably want to move into several positions and halt for a moment to capture a pointcloud snapshot
  }  

  ros::waitForShutdown();
  return 0;

}
