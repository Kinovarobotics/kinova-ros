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
 *  File: jaco_pose_action.cpp
 *  Desc: Class for moving/querying jaco arm.
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

#include "jaco_driver/jaco_pose_action.h"
#include <kinova/KinovaTypes.h>
#include "jaco_driver/jaco_types.h"
#include <string>


namespace jaco
{

JacoPoseActionServer::JacoPoseActionServer(JacoComm &arm_comm, const ros::NodeHandle &nh)
    : node_handle_(nh, "arm_pose"),
      arm_comm_(arm_comm),
      action_server_(node_handle_, "arm_pose",
                     boost::bind(&JacoPoseActionServer::actionCallback, this, _1), false)
{
    double tolerance;
    node_handle_.param<double>("stall_interval_seconds", stall_interval_seconds_, 1.0);
    node_handle_.param<double>("stall_threshold", stall_threshold_, 0.005);
    node_handle_.param<double>("rate_hz", rate_hz_, 10.0);
    node_handle_.param<double>("tolerance", tolerance, 0.01);
    node_handle_.param<std::string>("tf_prefix", tf_prefix_, "jaco_");

    tolerance_ = static_cast<float>(tolerance);
    std::stringstream ss;
    ss << tf_prefix_ << "api_origin";
    api_origin_frame_ = ss.str();

    action_server_.start();
}


JacoPoseActionServer::~JacoPoseActionServer()
{
}


void JacoPoseActionServer::actionCallback(const jaco_msgs::ArmPoseGoalConstPtr &goal)
{
    jaco_msgs::ArmPoseFeedback feedback;
    jaco_msgs::ArmPoseResult result;
    feedback.pose.header.frame_id = goal->pose.header.frame_id;
    result.pose.header.frame_id = goal->pose.header.frame_id;

    ros::Time current_time = ros::Time::now();
    JacoPose current_pose;
    geometry_msgs::PoseStamped local_pose;
    local_pose.header.frame_id = api_origin_frame_;

    try
    {
        // Put the goal pose into the frame used by the arm
        if (ros::ok()
                && !listener.canTransform(api_origin_frame_, goal->pose.header.frame_id,
                                          goal->pose.header.stamp))
        {
            ROS_ERROR("Could not get transfrom from %s to %s, aborting cartesian movement",
                      api_origin_frame_.c_str(), goal->pose.header.frame_id.c_str());
            action_server_.setAborted(result);
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
            return;
        }

        last_nonstall_time_ = current_time;
        last_nonstall_pose_ = current_pose;

        JacoPose target(local_pose.pose);
        arm_comm_.setCartesianPosition(target);

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

            if (target.isCloseToOther(current_pose, tolerance_))
            {
                result.pose = feedback.pose;
                action_server_.setSucceeded(result);
                return;
            }
            else if (!last_nonstall_pose_.isCloseToOther(current_pose, stall_threshold_))
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

}  // namespace jaco
