// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// manipulation_class.hpp
// ********************************************************************************************

#ifndef MANIPULATION_CLASS_HPP
#define MANIPULATION_CLASS_HPP

#include <boost/filesystem.hpp>
#include <math.h>
#include <stdlib.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <array>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <moveit/move_group/capability_names.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Int8.h>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;

class Manipulation
{
  public:

    // Default Constructor
    Manipulation(std::string planning_group)
    {
      PLANNING_GROUP = planning_group;
    }

    // Cooler Constructor
    Manipulation(ros::NodeHandle nodeHandle, std::string planning_group)
    {
      PLANNING_GROUP = planning_group;
      // this is where you could implement any other service clients, publishers, subscribers, etc.
    }

    // Member Variables
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    MoveGroupPtr move_group_ptr;
    PlanningScenePtr planning_scene_ptr;
    std::string PLANNING_GROUP;
    std::vector<double> joint_group_positions;
    moveit::core::RobotStatePtr current_state;
    tf2::Quaternion q;
    geometry_msgs::Pose target_pose;
    geometry_msgs::Vector3 orientation;
    bool pose_success;
    double x_pos;
    double y_pos;
    double z_pos;
    double th_x;
    double th_y;
    double th_z;
    ros::ServiceClient clearOctomap;
    std_srvs::Empty srv;

    // Publisher(s)

    // Subscriber(s)

    // Functions
    // Create some functions!
};

#endif // MANIPULATION_CLASS
