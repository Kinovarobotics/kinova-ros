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
 *  File: kinova_joint_angles_action.cpp
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


#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_ros_types.h"


namespace kinova
{

KinovaAnglesActionServer::KinovaAnglesActionServer(KinovaComm &arm_comm, const ros::NodeHandle &nh)
    : arm_comm_(arm_comm),
      node_handle_(nh, "joints_action"),
      action_server_(node_handle_, "joint_angles",
                     boost::bind(&KinovaAnglesActionServer::actionCallback, this, _1), false)
{
    double tolerance;
    node_handle_.param<double>("stall_interval_seconds", stall_interval_seconds_, 0.5);
    node_handle_.param<double>("stall_threshold", stall_threshold_, 1.0);
    node_handle_.param<double>("rate_hz", rate_hz_, 10.0);
    node_handle_.param<double>("tolerance", tolerance, 2.0);
    nh.param<double>("jointSpeedLimitParameter1",jointSpeedLimitJoints123,20);
    nh.param<double>("jointSpeedLimitParameter2",jointSpeedLimitJoints456,20);
    tolerance_ = (float)tolerance;

    action_server_.start();
}


KinovaAnglesActionServer::~KinovaAnglesActionServer()
{
}


void KinovaAnglesActionServer::actionCallback(const kinova_msgs::ArmJointAnglesGoalConstPtr &goal)
{
    kinova_msgs::ArmJointAnglesFeedback feedback;
    kinova_msgs::ArmJointAnglesResult result;
    KinovaAngles current_joint_angles;
    ros::Time current_time = ros::Time::now();

    bool action_is_over = false;

    try
    {
        arm_comm_.getJointAngles(current_joint_angles);

        if (arm_comm_.isStopped())
        {
            ROS_INFO("Could not complete joint angle action because the arm is 'stopped'.");
            result.angles = current_joint_angles.constructAnglesMsg();
            action_server_.setAborted(result);
            ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
            return;
        }

        last_nonstall_time_ = current_time;
        last_nonstall_angles_ = current_joint_angles;

        KinovaAngles target(goal->angles);
        arm_comm_.setJointAngles(target,jointSpeedLimitJoints123,jointSpeedLimitJoints456);

        // Loop until the action completed, is preempted, or fails in some way.
        // timeout is left to the caller since the timeout may greatly depend on
        // the context of the movement.
        while (!action_is_over)
        {
            ros::spinOnce();
	        if (arm_comm_.isStopped())
            {
                result.angles = current_joint_angles.constructAnglesMsg();
                action_server_.setAborted(result);
                ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
                action_is_over = true;
            }
            else if (action_server_.isPreemptRequested() || !ros::ok())
            {
                result.angles = current_joint_angles.constructAnglesMsg();
                arm_comm_.stopAPI();
                arm_comm_.startAPI();
                action_server_.setPreempted(result);
                ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setPreempted ");
                action_is_over = true;
            }

            arm_comm_.getJointAngles(current_joint_angles);
            current_time = ros::Time::now();
            feedback.angles = current_joint_angles.constructAnglesMsg();
//            action_server_.publishFeedback(feedback);

            if (target.isCloseToOther(current_joint_angles, tolerance_))
            {
                // Check if the action has succeeeded
                result.angles = current_joint_angles.constructAnglesMsg();
                action_server_.setSucceeded(result);
                ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setSucceeded ");
                action_is_over = true;
            }
            else if (!last_nonstall_angles_.isCloseToOther(current_joint_angles, stall_threshold_))
            {
                // Check if we are outside of a potential stall condition
                last_nonstall_time_ = current_time;
                last_nonstall_angles_ = current_joint_angles;
            }
            else if ((current_time - last_nonstall_time_).toSec() > stall_interval_seconds_)
            {
                // Check if the full stall condition has been meet
                result.angles = current_joint_angles.constructAnglesMsg();
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
                action_is_over = true;
            }

            ros::Rate(rate_hz_).sleep();
        }
    }
    catch(const std::exception& e)
    {
        result.angles = current_joint_angles.constructAnglesMsg();
        ROS_ERROR_STREAM(e.what());
        action_server_.setAborted(result);
        ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
    }

    // Put back the arm in Cartesian position mode
    KinovaPose pose;
    arm_comm_.getCartesianPosition(pose);
    arm_comm_.setCartesianPosition(pose, 0, false);
}

}  // namespace kinova
