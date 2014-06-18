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
 *  File: jaco_angles_action.cpp
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

#include "jaco_driver/jaco_angles_action.h"
#include <kinova/KinovaTypes.h>
#include "jaco_driver/jaco_types.h"

namespace jaco
{

JacoAnglesActionServer::JacoAnglesActionServer(JacoComm &arm_comm, ros::NodeHandle &n) :
    arm_comm_(arm_comm),
    action_server_(n, "arm_joint_angles", boost::bind(&JacoAnglesActionServer::actionCallback, this, _1), false),
    rate_hz_(10),    // TODO: Make this a parameter
    tolerance_(2.0)  // TODO: make this a parameter dead zone for position
{
    action_server_.start();
}

JacoAnglesActionServer::~JacoAnglesActionServer()
{
}

void JacoAnglesActionServer::actionCallback(const jaco_msgs::ArmJointAnglesGoalConstPtr &goal)
{
    jaco_msgs::ArmJointAnglesFeedback feedback;
    jaco_msgs::ArmJointAnglesResult result;
    JacoAngles current_joint_angles;

    // TODO: Look into the mismatch between joint numbering
    ROS_INFO("Got an angular goal for the arm: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
             goal->angles.joint1, goal->angles.joint2, goal->angles.joint3,
             goal->angles.joint4, goal->angles.joint5, goal->angles.joint6);

    if (arm_comm_.isStopped())
    {
        ROS_INFO("Could not complete joint angle action because the arm is 'stopped'.");
        arm_comm_.getJointAngles(current_joint_angles);
        result.angles = current_joint_angles.constructAnglesMsg();
        action_server_.setAborted(result);
        return;
    }

    JacoAngles target(goal->angles);
    arm_comm_.setJointAngles(target);

    // Loop until the action completed, is preempted, or fails in some way.
    // timeout is left to the caller since the timeout may greatly depend on
    // the context of the movement.
    while (true)
    {
        ros::spinOnce();
        if (action_server_.isPreemptRequested() || !ros::ok())
        {
            arm_comm_.stop();
            arm_comm_.start();
            action_server_.setPreempted();
            return;
        }

        if (arm_comm_.isStopped())
        {
            result.angles = current_joint_angles.constructAnglesMsg();
            action_server_.setAborted(result);
            return;
        }

        arm_comm_.getJointAngles(current_joint_angles);
        feedback.angles = current_joint_angles.constructAnglesMsg();
        action_server_.publishFeedback(feedback);

        if (target.isCloseToOther(current_joint_angles, tolerance_))
        {
            ROS_INFO("Angular Control Complete.");
            result.angles = current_joint_angles.constructAnglesMsg();
            action_server_.setSucceeded(result);
            return;
        }

        rate_hz_.sleep();
    }
}

}  // namespace jaco
