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
 *  File: kinova_fingers_action.cpp
 *  Desc: Class for moving/querying kinova arm fingers.
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

#include "kinova_driver/kinova_fingers_action.h"
#include "kinova_driver/kinova_ros_types.h"


namespace kinova
{

KinovaFingersActionServer::KinovaFingersActionServer(KinovaComm &arm_comm, const ros::NodeHandle &nh)
    : arm_comm_(arm_comm),
      node_handle_(nh, "fingers_action"),
      action_server_(node_handle_, "finger_positions",
                     boost::bind(&KinovaFingersActionServer::actionCallback, this, _1), false)
{
    double tolerance;
    node_handle_.param<double>("stall_interval_seconds", stall_interval_seconds_, 0.5);
    node_handle_.param<double>("stall_threshold", stall_threshold_, 1.0);
    node_handle_.param<double>("rate_hz", rate_hz_, 10.0);
    node_handle_.param<double>("tolerance", tolerance, 6400.0*0.01);
    tolerance_ = static_cast<float>(tolerance);

    action_server_.start();
}


KinovaFingersActionServer::~KinovaFingersActionServer()
{
}


void KinovaFingersActionServer::actionCallback(const kinova_msgs::SetFingersPositionGoalConstPtr &goal)
{
    if ((arm_comm_.numFingers() < 3) && (goal->fingers.finger3 != 0.0))
    {
        ROS_WARN("Detected that the third finger command was non-zero even though there "
                 "are only two fingers on the gripper. The goal for the third finger "
                 "should be set to zero or you make experience delays in action results.");
    }

    kinova_msgs::SetFingersPositionFeedback feedback;
    kinova_msgs::SetFingersPositionResult result;
    FingerAngles current_finger_positions;
    ros::Time current_time = ros::Time::now();

    try
    {
        arm_comm_.getFingerPositions(current_finger_positions);

        if (arm_comm_.isStopped())
        {
            ROS_INFO("Could not complete finger action because the arm is stopped");
            result.fingers = current_finger_positions.constructFingersMsg();
            ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
            action_server_.setAborted(result);
            return;
        }

        last_nonstall_time_ = current_time;
        last_nonstall_finger_positions_ = current_finger_positions;

        FingerAngles target(goal->fingers);
        arm_comm_.setFingerPositions(target);

        // Loop until the action completed, is preempted, or fails in some way.
        // timeout is left to the caller since the timeout may greatly depend on
        // the context of the movement.
        while (true)
        {
            ros::spinOnce();

	    if (arm_comm_.isStopped())
            {
                result.fingers = current_finger_positions.constructFingersMsg();
                action_server_.setAborted(result);
                ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
                return;
            }
            else if (action_server_.isPreemptRequested() || !ros::ok())
            {
                result.fingers = current_finger_positions.constructFingersMsg();
                arm_comm_.stopAPI();
                arm_comm_.startAPI();
                action_server_.setPreempted(result);
                ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setPreempted ");
                return;
            }

            arm_comm_.getFingerPositions(current_finger_positions);
            current_time = ros::Time::now();
            feedback.fingers = current_finger_positions.constructFingersMsg();
//            action_server_.publishFeedback(feedback);
            if (target.isCloseToOther(current_finger_positions, tolerance_))
            {
                // Check if the action has succeeeded
                result.fingers = current_finger_positions.constructFingersMsg();
                action_server_.setSucceeded(result);
                ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setSucceeded ");
                return;
            }
            else if (!last_nonstall_finger_positions_.isCloseToOther(current_finger_positions, stall_threshold_))
            {
                // Check if we are outside of a potential stall condition
                last_nonstall_time_ = current_time;
                last_nonstall_finger_positions_ = current_finger_positions;
            }
            else if ((current_time - last_nonstall_time_).toSec() > stall_interval_seconds_)
            {
                // Check if the full stall condition has been meet
                result.fingers = current_finger_positions.constructFingersMsg();
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
                ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", Trajectory command failed ");
                return;
            }

            ros::Rate(rate_hz_).sleep();
        }
    }
    catch(const std::exception& e)
    {
        result.fingers = current_finger_positions.constructFingersMsg();
        ROS_ERROR_STREAM(e.what());
        action_server_.setAborted(result);
        ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
    }
}

}  // namespace kinova
