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
#include <jaco_driver/KinovaTypes.h>
#include "jaco_driver/jaco_types.h"


// TODO: Remove me when done (used for syscall(SYS_gettid))
#include <unistd.h>
#include <sys/syscall.h>
#include <sys/types.h>


namespace jaco {

JacoFingersActionServer::JacoFingersActionServer(JacoComm &arm_comm, ros::NodeHandle &n)
    : arm_(arm_comm),
      action_server_(n, "finger_joint_angles",
                     boost::bind(&JacoFingersActionServer::actionCallback, this, _1), false)
{

    ROS_INFO_STREAM("file: " << __FILE__ << ", line: " << __LINE__ << ", "
                    "process: " << getpid() << ", thread: " << syscall(SYS_gettid));

    action_server_.start();

    ROS_INFO_STREAM("file: " << __FILE__ << ", line: " << __LINE__ << ", "
                    "process: " << getpid() << ", thread: " << syscall(SYS_gettid));

}

JacoFingersActionServer::~JacoFingersActionServer()
{

    ROS_INFO_STREAM("file: " << __FILE__ << ", line: " << __LINE__ << ", "
                    "process: " << getpid() << ", thread: " << syscall(SYS_gettid));

}

void JacoFingersActionServer::actionCallback(const jaco_msgs::SetFingersPositionGoalConstPtr &goal) {
    jaco_msgs::SetFingersPositionFeedback feedback;
    jaco_msgs::SetFingersPositionResult result;


    ROS_INFO_STREAM("file: " << __FILE__ << ", line: " << __LINE__ << ", "
                    "process: " << getpid() << ", thread: " << syscall(SYS_gettid));

    ROS_INFO("Got a finger goal: (%f, %f, %f)",
             goal->fingers.finger1, goal->fingers.finger2, goal->fingers.finger3);

    FingerAngles current_finger_positions;  // holds the current position of the fingers


    ROS_INFO_STREAM("file: " << __FILE__ << ", line: " << __LINE__ << ", "
                    "process: " << getpid() << ", thread: " << syscall(SYS_gettid));

    if (arm_.isStopped()) {
        ROS_INFO("Could not complete finger action because the arm is stopped");

        ROS_INFO_STREAM("file: " << __FILE__ << ", line: " << __LINE__ << ", "
                        "process: " << getpid() << ", thread: " << syscall(SYS_gettid));

        arm_.getFingers(current_finger_positions);

        ROS_INFO_STREAM("file: " << __FILE__ << ", line: " << __LINE__ << ", "
                        "process: " << getpid() << ", thread: " << syscall(SYS_gettid));

        result.fingers = current_finger_positions.constructFingersMsg();

        ROS_INFO_STREAM("file: " << __FILE__ << ", line: " << __LINE__ << ", "
                        "process: " << getpid() << ", thread: " << syscall(SYS_gettid));

        action_server_.setAborted(result);

        ROS_INFO_STREAM("file: " << __FILE__ << ", line: " << __LINE__ << ", "
                        "process: " << getpid() << ", thread: " << syscall(SYS_gettid));

        return;
    }

    ROS_INFO_STREAM("file: " << __FILE__ << ", line: " << __LINE__ << ", "
                    "process: " << getpid() << ", thread: " << syscall(SYS_gettid));

    FingerAngles target(goal->fingers);
    arm_.setFingers(target);

    ROS_INFO_STREAM("file: " << __FILE__ << ", line: " << __LINE__ << ", "
                    "process: " << getpid() << ", thread: " << syscall(SYS_gettid));

    ros::Rate rate_hz(10);
    const float tolerance = 2.0;  // dead zone for position


    // while we have not timed out
    while (true) {

        ROS_INFO_STREAM("file: " << __FILE__ << ", line: " << __LINE__ << ", "
                        "process: " << getpid() << ", thread: " << syscall(SYS_gettid));

        ros::spinOnce();
        if (action_server_.isPreemptRequested() || !ros::ok()) {
            ROS_INFO("Preempting finger action");

            ROS_INFO_STREAM("file: " << __FILE__ << ", line: " << __LINE__ << ", "
                            "process: " << getpid() << ", thread: " << syscall(SYS_gettid));

            arm_.stop();
            arm_.start();
            action_server_.setPreempted();
            return;
        }

        ROS_INFO_STREAM("file: " << __FILE__ << ", line: " << __LINE__ << ", "
                        "process: " << getpid() << ", thread: " << syscall(SYS_gettid));

        arm_.getFingers(current_finger_positions);

        ROS_INFO("Current finger positions: %f, %f, %f, tolerance: %f",
                 current_finger_positions.Finger1,
                 current_finger_positions.Finger2,
                 current_finger_positions.Finger3,
                 tolerance);

        feedback.fingers = current_finger_positions.constructFingersMsg();

        ROS_INFO_STREAM("file: " << __FILE__ << ", line: " << __LINE__ << ", "
                        "process: " << getpid() << ", thread: " << syscall(SYS_gettid));

        if (arm_.isStopped()) {
            ROS_INFO("Aborting finger action because the arm is stopped");
            result.fingers = current_finger_positions.constructFingersMsg();
            action_server_.setAborted(result);
            return;
        }

        action_server_.publishFeedback(feedback);

        ROS_INFO_STREAM("file: " << __FILE__ << ", line: " << __LINE__ << ", "
                        "process: " << getpid() << ", thread: " << syscall(SYS_gettid));

        if (target.compareToOther(current_finger_positions, tolerance)) {
            ROS_INFO("Finger action complete.");

            result.fingers = current_finger_positions.constructFingersMsg();
            action_server_.setSucceeded(result);
            return;
        }


        ROS_INFO_STREAM("file: " << __FILE__ << ", line: " << __LINE__ << ", "
                        "process: " << getpid() << ", thread: " << syscall(SYS_gettid));

        rate_hz.sleep();
    }
}

}
