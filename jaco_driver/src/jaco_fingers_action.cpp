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
 *  File: jaco_fingers_action.cpp
 *  Desc: Class for moving/querying jaco arm fingers.
 *  Auth: Jeff Schmidt
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

#include "jaco_driver/jaco_fingers_action.h"
#include <kinova/KinovaTypes.h>
#include "jaco_driver/jaco_types.h"


namespace jaco {

JacoFingersActionServer::JacoFingersActionServer(JacoComm &arm_comm, ros::NodeHandle &nh)
    : arm_comm_(arm_comm),
      action_server_(nh, "finger_joint_angles",
                     boost::bind(&JacoFingersActionServer::actionCallback, this, _1), false),
      rate_hz_(10),    // TODO: Make this a parameter
      tolerance_(2.0)  // TODO: make this a parameter dead zone for position
{
    ROS_INFO_STREAM("File: " << __FILE__ << ", line: " << __LINE__ << ", function: " << __PRETTY_FUNCTION__);
    action_server_.start();
}

JacoFingersActionServer::~JacoFingersActionServer() {
    ROS_INFO_STREAM("File: " << __FILE__ << ", line: " << __LINE__ << ", function: " << __PRETTY_FUNCTION__);
}

void JacoFingersActionServer::actionCallback(const jaco_msgs::SetFingersPositionGoalConstPtr &goal) {
    ROS_INFO_STREAM("File: " << __FILE__ << ", line: " << __LINE__ << ", function: " << __PRETTY_FUNCTION__);
    ROS_INFO("Got a finger goal: (%f, %f, %f, tolerance: %f",
             goal->fingers.finger1, goal->fingers.finger2, goal->fingers.finger3,
             tolerance_);

    jaco_msgs::SetFingersPositionFeedback feedback;
    jaco_msgs::SetFingersPositionResult result;
    FingerAngles current_finger_positions;

    if (arm_comm_.isStopped()) {
        ROS_INFO("\tCould not complete finger action because the arm is stopped");
        arm_comm_.getFingerPositions(current_finger_positions);
        result.fingers = current_finger_positions.constructFingersMsg();
        action_server_.setAborted(result);
        return;
    }

    FingerAngles target(goal->fingers);
    arm_comm_.setFingerPositions(target);

    // Loop until the action completed, is preempted, or fails in some way.
    // timeout is left to the caller since the timeout may greatly depend on
    // the context of the movement.
    while (true) {
        ros::spinOnce();
        if (action_server_.isPreemptRequested() || !ros::ok()) {
            ROS_INFO("\tPreempting finger action");
            arm_comm_.stop();
            arm_comm_.start();
            action_server_.setPreempted();
            return;
        } else if (arm_comm_.isStopped()) {
            ROS_INFO("\tAborting finger action because the arm is stopped");
            result.fingers = current_finger_positions.constructFingersMsg();
            action_server_.setAborted(result);
            return;
        }

        action_server_.publishFeedback(feedback);
        arm_comm_.getFingerPositions(current_finger_positions);
        feedback.fingers = current_finger_positions.constructFingersMsg();

        ROS_INFO("\tCurrent finger positions: %f, %f, %f",
                 current_finger_positions.Finger1,
                 current_finger_positions.Finger2,
                 current_finger_positions.Finger3);

        if (target.isCloseToOther(current_finger_positions, tolerance_)) {
            ROS_INFO("\tFinger action complete.");
            result.fingers = current_finger_positions.constructFingersMsg();
            action_server_.setSucceeded(result);
            return;
        }

        rate_hz_.sleep();
    }
}

}  // namespace jaco
