/**
 *
 *  File: kinova_advanced_pose_action.cpp
 *  Desc: Action server for kinova arm who take a trajectory point as an input.
 *  Auth: Mathieu Lauret
 *
 *  Copyright (c) 2016, Mathieu Lauret
 *  All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from this
 *       software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * history:
 *  22/08/16 : First Creation
 *
 */

#include <kinova_driver/kinova_advanced_pose_action.h>
#include <kinova/KinovaTypes.h>
#include "kinova_driver/kinova_ros_types.h"
#include <string>


namespace kinova
{

KinovaAdvancedPoseActionServer::KinovaAdvancedPoseActionServer(KinovaComm &arm_comm, const ros::NodeHandle &nh, const std::string &kinova_robotType)
    : arm_comm_(arm_comm),
      node_handle_(nh, "advanced_pose_action"),
      kinova_robotType_(kinova_robotType),
      action_server_(node_handle_, "advanced_tool_pose",
                     boost::bind(&KinovaAdvancedPoseActionServer::actionCallback, this, _1), false)
{
    double position_tolerance;
    double EulerAngle_tolerance;
    node_handle_.param<double>("stall_interval_seconds", stall_interval_seconds_, 0.5);
    node_handle_.param<double>("stall_threshold", stall_threshold_, 0.005);
    node_handle_.param<double>("rate_hz", rate_hz_, 100.0);
    node_handle_.param<double>("position_tolerance", position_tolerance, 0.001);
    node_handle_.param<double>("EulerAngle_tolerance", EulerAngle_tolerance, 1.0*M_PI/180);

    //    tf_prefix_ = kinova_robotType_ + "_" + boost::lexical_cast<string>(same_type_index); // in case of multiple same_type robots
    tf_prefix_ = kinova_robotType_ + "_";

    position_tolerance_ = static_cast<float>(position_tolerance);
    EulerAngle_tolerance_ = static_cast<float>(EulerAngle_tolerance);
    std::stringstream ss;
    ss << tf_prefix_ << "link_base";
    link_base_frame_ = ss.str();

    action_server_.start();
}


KinovaAdvancedPoseActionServer::~KinovaAdvancedPoseActionServer()
{
}


void KinovaAdvancedPoseActionServer::actionCallback(const kinova_msgs::Arm_KinovaTrajectoryGoalConstPtr &goal)
{
    kinova_msgs::Arm_KinovaTrajectoryFeedback feedback;
    kinova_msgs::Arm_KinovaTrajectoryResult result;
    feedback.pose.header.frame_id = goal->header.frame_id;
    result.pose.header.frame_id = goal->header.frame_id;

    ros::Time current_time = ros::Time::now();
    KinovaPose current_pose;
    geometry_msgs::PoseStamped local_pose;
    local_pose.header.frame_id = link_base_frame_;

    try
    {
        // Put the goal pose into the frame used by the arm
        if (ros::ok()
                && !listener.canTransform(link_base_frame_, goal->header.frame_id,
                                          goal->header.stamp))
        {
            ROS_ERROR("Could not get transfrom from %s to %s, aborting cartesian movement",
                      link_base_frame_.c_str(), goal->header.frame_id.c_str());
            action_server_.setAborted(result);
            return;
        }

        geometry_msgs::PoseStamped goalStamped;
        goalStamped.header = goal->header;
        goalStamped.pose = goal->trajectoryPoint.pose;
        listener.transformPose(local_pose.header.frame_id, goalStamped, local_pose);
        arm_comm_.getCartesianPosition(current_pose);

        if (arm_comm_.isStopped())
        {
            ROS_INFO("Could not complete advanced cartesian action because the arm is 'stopped'.");
            local_pose.pose = current_pose.constructPoseMsg();
            listener.transformPose(result.pose.header.frame_id, local_pose, result.pose);
            action_server_.setAborted(result);
            return;
        }

        last_nonstall_time_ = current_time;
        last_nonstall_pose_ = current_pose;

        KinovaPose target(local_pose.pose);
        TrajectoryPoint trajectoryPoint;

        trajectoryPoint.Position.Type = static_cast<POSITION_TYPE>(goal->trajectoryPoint.PositionType);
        trajectoryPoint.Position.Delay = goal->trajectoryPoint.Delay;

        trajectoryPoint.Position.CartesianPosition.X = target.X;
        trajectoryPoint.Position.CartesianPosition.Y = target.Y;
        trajectoryPoint.Position.CartesianPosition.Z = target.Z;
        trajectoryPoint.Position.CartesianPosition.ThetaX = target.ThetaX;
        trajectoryPoint.Position.CartesianPosition.ThetaY = target.ThetaY;
        trajectoryPoint.Position.CartesianPosition.ThetaZ = target.ThetaZ;

        trajectoryPoint.Position.HandMode = static_cast<HAND_MODE>(goal->trajectoryPoint.HandMode);
        trajectoryPoint.Position.Fingers.Finger1 = goal->trajectoryPoint.Fingers.finger1;
        trajectoryPoint.Position.Fingers.Finger2 = goal->trajectoryPoint.Fingers.finger2;
        trajectoryPoint.Position.Fingers.Finger3 = goal->trajectoryPoint.Fingers.finger3;

        trajectoryPoint.LimitationsActive = goal->trajectoryPoint.LimitationsActive;
        trajectoryPoint.SynchroType = goal->trajectoryPoint.SynchroType;

        trajectoryPoint.Limitations.speedParameter1 = goal->trajectoryPoint.speedParameter1;
        trajectoryPoint.Limitations.speedParameter2 = goal->trajectoryPoint.speedParameter2;


        arm_comm_.setCartesianPosition(trajectoryPoint);

        while (true)
        {
            ros::spinOnce();

            if (action_server_.isPreemptRequested() || !ros::ok())
            {
                result.pose = feedback.pose;
                arm_comm_.stopAPI();
                arm_comm_.startAPI();
                action_server_.setPreempted(result);
                return;
            }
            else if (arm_comm_.isStopped())
            {
                result.pose = feedback.pose;
                action_server_.setAborted(result);
                return;
            }

            arm_comm_.getCartesianPosition(current_pose);
            current_time = ros::Time::now();
            local_pose.pose = current_pose.constructPoseMsg();
            listener.transformPose(feedback.pose.header.frame_id, local_pose, feedback.pose);
            action_server_.publishFeedback(feedback);

            if (target.isCloseToOther(current_pose, position_tolerance_, EulerAngle_tolerance_))
            {
                result.pose = feedback.pose;
                action_server_.setSucceeded(result);
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
                // Check if the full stall condition has been meet
                result.pose = feedback.pose;
                arm_comm_.stopAPI();
                arm_comm_.startAPI();
                action_server_.setPreempted(result);
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
    }
}

}  // namespace kinova
