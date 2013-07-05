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
 *  File: jaco_types.cpp
 *  Desc: Wrappers around Kinova structs to facilitate easier conversion to ROS
 *		  types.
 *  Auth: Alex Bencz
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

#include <jaco_driver/jaco_types.h>
#include <tf/tf.h>
#include <math.h>

namespace jaco
{

JacoPose::JacoPose(geometry_msgs::Pose &pose)
{
	double tx, ty, tz;
	tf::Quaternion q;
	tf::quaternionMsgToTF(pose.orientation, q);

	tf::Matrix3x3 bt_q(q);

	bt_q.getEulerYPR(tz, ty, tx);

	X = (float) pose.position.x;
	Y = (float) pose.position.y;
	Z = (float) pose.position.z;

	ThetaX = Normalize(tx);
	ThetaY = Normalize(ty);
	ThetaZ = Normalize(tz);
}

JacoPose::JacoPose(CartesianInfo &pose)
{
	X = pose.X;
	Y = pose.Y;
	Z = pose.Z;

	ThetaX = Normalize(pose.ThetaX);
	ThetaY = Normalize(pose.ThetaY);
	ThetaZ = Normalize(pose.ThetaZ);
}

geometry_msgs::Pose JacoPose::Pose()
{
	geometry_msgs::Pose pose;
	tf::Quaternion position_quaternion;

	position_quaternion.setRPY(ThetaX, ThetaY, ThetaZ);
	tf::quaternionTFToMsg(position_quaternion, pose.orientation);

	pose.position.x = X;
	pose.position.y = Y;
	pose.position.z = Z;

	return pose;
}

bool JacoPose::Compare(const JacoPose &other, float tolerance)
{
	bool status = true;

	status = status && CompareValues(X, other.X, tolerance);
	status = status && CompareValues(Y, other.Y, tolerance);
	status = status && CompareValues(Z, other.Z, tolerance);
	status = status && CompareValues(ThetaX, other.ThetaX, tolerance);
	status = status && CompareValues(ThetaY, other.ThetaY, tolerance);
	status = status && CompareValues(ThetaZ, other.ThetaZ, tolerance);

	return status;
}

bool JacoPose::CompareValues(float first, float second, float tolerance)
{
	return ((first <= second + tolerance) && (first >= second - tolerance));
}

float JacoPose::Normalize(float value)
{
	while (value > 2 * M_PI)
		value -= 2 * M_PI;
	while (value < 0)
		value += 2 * M_PI;

	return value;
}

}
