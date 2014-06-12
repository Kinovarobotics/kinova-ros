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
#include <KinovaTypes.h>
#include "jaco_driver/jaco_types.h"

namespace jaco
{

JacoPoseActionServer::JacoPoseActionServer(JacoComm &arm_comm, ros::NodeHandle &n) :
    arm(arm_comm),
    as_(n, "arm_pose", boost::bind(&JacoPoseActionServer::ActionCallback, this, _1), false)
{
    as_.start();
}

JacoPoseActionServer::~JacoPoseActionServer()
{

}

void JacoPoseActionServer::ActionCallback(const jaco_msgs::ArmPoseGoalConstPtr &goal)
{
    jaco_msgs::ArmPoseFeedback feedback;
    jaco_msgs::ArmPoseResult result;
    feedback.pose.header.frame_id = goal->pose.header.frame_id;
    result.pose.header.frame_id = goal->pose.header.frame_id;

    ROS_INFO("Got a cartesian goal for the arm");

    ROS_DEBUG("Raw goal");
    ROS_DEBUG("X = %f", goal->pose.pose.position.x);
    ROS_DEBUG("Y = %f", goal->pose.pose.position.y);
    ROS_DEBUG("Z = %f", goal->pose.pose.position.z);

    ROS_DEBUG("RX = %f", goal->pose.pose.orientation.x);
    ROS_DEBUG("RY = %f", goal->pose.pose.orientation.y);
    ROS_DEBUG("RZ = %f", goal->pose.pose.orientation.z);
    ROS_DEBUG("RW = %f", goal->pose.pose.orientation.w);

    if (ros::ok()
            && !listener.canTransform("/jaco_api_origin", goal->pose.header.frame_id,
                    goal->pose.header.stamp))
    {
        ROS_ERROR("Could not get transfrom from /jaco_api_origin to %s, aborting cartesian movement", goal->pose.header.frame_id.c_str());
        return;
    }

    geometry_msgs::PoseStamped local_pose;
    local_pose.header.frame_id = "/jaco_api_origin";
    listener.transformPose(local_pose.header.frame_id, goal->pose, local_pose);

    ROS_DEBUG("Transformed MSG");
    ROS_DEBUG("X = %f", local_pose.pose.position.x);
    ROS_DEBUG("Y = %f", local_pose.pose.position.y);
    ROS_DEBUG("Z = %f", local_pose.pose.position.z);

    ROS_DEBUG("RX = %f", local_pose.pose.orientation.x);
    ROS_DEBUG("RY = %f", local_pose.pose.orientation.y);
    ROS_DEBUG("RZ = %f", local_pose.pose.orientation.z);
    ROS_DEBUG("RW = %f", local_pose.pose.orientation.w);

    JacoPose cur_position;		//holds the current position of the arm

    if (arm.isStopped())
    {
        arm.getCartesianPosition(cur_position);
        local_pose.pose = cur_position.constructPoseMsg();

        listener.transformPose(result.pose.header.frame_id, local_pose, result.pose);
        as_.setAborted(result);
        return;
    }

    JacoPose target(local_pose.pose);
    arm.setCartesianPosition(target);

    ros::Rate r(10);

    const float tolerance = 0.05; 	//dead zone for position

    //while we have not timed out
    while (true)
    {
        ros::spinOnce();
        if (as_.isPreemptRequested() || !ros::ok())
        {
            arm.stop();
            arm.start();
            as_.setPreempted();
            return;
        }

        arm.getCartesianPosition(cur_position);
        local_pose.pose = cur_position.constructPoseMsg();

        listener.transformPose(feedback.pose.header.frame_id, local_pose, feedback.pose);

        if (arm.isStopped())
        {
            result.pose = feedback.pose;
            as_.setAborted(result);
            return;
        }

        as_.publishFeedback(feedback);

        if (target.compareToOther(cur_position, tolerance))
        {
            ROS_INFO("Cartesian Control Complete.");

            result.pose = feedback.pose;
            as_.setSucceeded(result);
            return;
        }

        r.sleep();
    }
}

}
