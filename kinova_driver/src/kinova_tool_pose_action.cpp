/**
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™
 *
 *  File: kinova_tool_pose_action.cpp
 *  Desc: Class for moving/querying kinova arm.
 *  Auth: Alex Bencz, Jeff Schmidt
 *
 *  Copyright (c) 2013, Clearpath Robotics, Inc.
 *  All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com
 *
 */

#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_ros_types.h"
#include <string>

#include <ros/console.h>


namespace kinova
{

KinovaPoseActionServer::KinovaPoseActionServer(KinovaComm &arm_comm, const ros::NodeHandle &nh, const std::string &kinova_robotType, const std::string &kinova_robotName)
    : arm_comm_(arm_comm),
      node_handle_(nh, "pose_action"),
      kinova_robotType_(kinova_robotType),
      kinova_robotName_(kinova_robotName),
      action_server_(node_handle_, "tool_pose",
                     boost::bind(&KinovaPoseActionServer::actionCallback, this, _1), false)
{
    double position_tolerance;
    double EulerAngle_tolerance;
    node_handle_.param<double>("stall_interval_seconds", stall_interval_seconds_, 1.0);
    node_handle_.param<double>("stall_threshold", stall_threshold_, 0.005);
    node_handle_.param<double>("rate_hz", rate_hz_, 10.0);
    node_handle_.param<double>("position_tolerance", position_tolerance, 0.01);
    node_handle_.param<double>("EulerAngle_tolerance", EulerAngle_tolerance, 2.0*M_PI/180);

    //    tf_prefix_ = kinova_robotType_ + "_" + boost::lexical_cast<string>(same_type_index); // in case of multiple same_type robots
    tf_prefix_ = kinova_robotName_ + "_";

    position_tolerance_ = static_cast<float>(position_tolerance);
    EulerAngle_tolerance_ = static_cast<float>(EulerAngle_tolerance);
    std::stringstream ss;
    ss << tf_prefix_ << "link_base";
    link_base_frame_ = ss.str();

    action_server_.start();

//    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
//    {
//        ros::console::notifyLoggerLevelsChanged();
//    }
}


KinovaPoseActionServer::~KinovaPoseActionServer()
{
}


void KinovaPoseActionServer::actionCallback(const kinova_msgs::ArmPoseGoalConstPtr &goal)
{
    kinova_msgs::ArmPoseFeedback feedback;
    kinova_msgs::ArmPoseResult result;
    feedback.pose.header.frame_id = goal->pose.header.frame_id;
    result.pose.header.frame_id = goal->pose.header.frame_id;

    ros::Time current_time = ros::Time::now();
    KinovaPose current_pose;
    geometry_msgs::PoseStamped local_pose;
    local_pose.header.frame_id = link_base_frame_;

    try
    {
        // Put the goal pose into the frame used by the arm
        if (ros::ok()
                && !listener.canTransform(link_base_frame_, goal->pose.header.frame_id,
                                          goal->pose.header.stamp))
        {
            ROS_ERROR("Could not get transfrom from %s to %s, aborting cartesian movement",
                      link_base_frame_.c_str(), goal->pose.header.frame_id.c_str());
            action_server_.setAborted(result);
            ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
            return;
        }
        listener.transformPose(local_pose.header.frame_id, goal->pose, local_pose);
        arm_comm_.getCartesianPosition(current_pose);
        if (arm_comm_.isStopped())
        {
            ROS_INFO("Could not complete cartesian action because the arm is 'stopped'.");
            local_pose.pose = current_pose.constructPoseMsg();
            listener.transformPose(result.pose.header.frame_id, local_pose, result.pose);
            action_server_.setAborted(result);
            ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
            return;
        }

        last_nonstall_time_ = current_time;
        last_nonstall_pose_ = current_pose;

        KinovaPose target(local_pose.pose);
        ROS_DEBUG_STREAM(std::endl << std::endl << "***-----------------------***" << std::endl << __PRETTY_FUNCTION__ << ":  target X " << target.X << "; Y "<< target.Y << "; Z "<< target.Z << "; ThetaX " << target.ThetaX << "; ThetaY " << target.ThetaY  << "; ThetaZ " << target.ThetaZ << std::endl << "***-----------------------***" << std::endl );
        arm_comm_.setCartesianPosition(target);
        while (ros::ok())
        {
            // without setCartesianPosition() in while loop, robot stopped in the half way, and the goal won't be reached.
            arm_comm_.setCartesianPosition(target);
            ros::spinOnce();

	    if (arm_comm_.isStopped())
            {
                ROS_DEBUG_STREAM("" << __PRETTY_FUNCTION__ << ": arm_comm_.isStopped()");
                result.pose = feedback.pose;
                action_server_.setAborted(result);
                ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
                return;
            }
            else if (action_server_.isPreemptRequested() || !ros::ok())
            {
                ROS_DEBUG_STREAM("" << __PRETTY_FUNCTION__ << ": action server isPreemptRequested");
                result.pose = feedback.pose;
                arm_comm_.stopAPI();
                arm_comm_.startAPI();
                action_server_.setPreempted(result);
                ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setPreempted ");
                return;
            }

            arm_comm_.getCartesianPosition(current_pose);
            current_time = ros::Time::now();
            local_pose.pose = current_pose.constructPoseMsg();
            listener.transformPose(feedback.pose.header.frame_id, local_pose, feedback.pose);
//            action_server_.publishFeedback(feedback);

            ROS_DEBUG_STREAM("" << __PRETTY_FUNCTION__ << ": current_pose X " << current_pose.X << "; Y "<< current_pose.Y << "; Z "<< current_pose.Z << "; ThetaX " << current_pose.ThetaX << "; ThetaY " << current_pose.ThetaY  << "; ThetaZ " << current_pose.ThetaZ );

            if (target.isCloseToOther(current_pose, position_tolerance_, EulerAngle_tolerance_))
            {
                ROS_DEBUG_STREAM("" << __PRETTY_FUNCTION__ << ": arm_comm_.isCloseToOther");
                result.pose = feedback.pose;
                action_server_.setSucceeded(result);
                ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setSucceeded ");
                return;
            }
            else if (!last_nonstall_pose_.isCloseToOther(current_pose, stall_threshold_, stall_threshold_))
            {
                // Check if we are outside of a potential stall condition
                last_nonstall_time_ = current_time;
                last_nonstall_pose_ = current_pose;
            }
            else if ((current_time - last_nonstall_time_).toSec() > stall_interval_seconds_)
            {
                ROS_DEBUG_STREAM("" << __PRETTY_FUNCTION__ << ": stall_interval_seconds_");
                // Check if the full stall condition has been meet
                result.pose = feedback.pose;
                if (!arm_comm_.isStopped())
                {
                	arm_comm_.stopAPI();
                	arm_comm_.startAPI();
		}
                //why preemted, if the robot is stalled, trajectory/action failed!
                /*
                action_server_.setPreempted(result);
                ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setPreempted ");
                */
                action_server_.setAborted(result);
                ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", Trajectory command failed ");
                return;
            }
            ros::Rate(rate_hz_).sleep();
        }
    }
    catch(const std::exception& e)
    {
        result.pose = feedback.pose;
        ROS_ERROR_STREAM(e.what());
        action_server_.setAborted(result);
        ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
    }
}

}  // namespace kinova
