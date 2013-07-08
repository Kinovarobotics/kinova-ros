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
#include <jaco_driver/KinovaTypes.h>
#include "jaco_driver/jaco_types.h"

namespace jaco
{

JacoAnglesActionServer::JacoAnglesActionServer(JacoComm &arm_comm, ros::NodeHandle &n) : 
    arm(arm_comm), 
    as_(n, "arm_joint_angles", boost::bind(&JacoAnglesActionServer::ActionCallback, this, _1), false)
{
    as_.start();
}

JacoAnglesActionServer::~JacoAnglesActionServer()
{

}

void JacoAnglesActionServer::ActionCallback(const jaco_driver::ArmJointAnglesGoalConstPtr &goal)
{
	jaco_driver::ArmJointAnglesFeedback feedback;
	jaco_driver::ArmJointAnglesResult result;

	ROS_INFO("Got an angular goal for the arm");

	JacoAngles cur_position;		//holds the current position of the arm

	if (arm.Stopped())
	{
		arm.GetAngles(cur_position);
		result.angles = cur_position.Angles();

		as_.setAborted(result);
		return;
	}

	JacoAngles target(goal->angles);
	arm.SetAngles(target);

	ros::Rate r(10);
 
	const float tolerance = 2.0; 	//dead zone for position (degrees)

	//while we have not timed out
	while (true)
	{
		ros::spinOnce();
		if (as_.isPreemptRequested() || !ros::ok())
		{
			arm.Stop();
			arm.Start();
			as_.setPreempted();
			return;
		}

		arm.GetAngles(cur_position);
		feedback.angles = cur_position.Angles();

		if (arm.Stopped())
		{
			result.angles = cur_position.Angles();
			as_.setAborted(result);
			return;
		}

		as_.publishFeedback(feedback);

		if (target.Compare(cur_position, tolerance))
		{
			ROS_INFO("Angular Control Complete.");

			result.angles = cur_position.Angles();
			as_.setSucceeded(result);
			return;
		}

		r.sleep();
	}
}

}
