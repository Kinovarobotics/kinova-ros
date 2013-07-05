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

bool CompareValues(float first, float second, float tolerance)
{
	return ((first <= second + tolerance) && (first >= second - tolerance));
}


JacoPose::JacoPose(const geometry_msgs::Pose &pose)
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

JacoPose::JacoPose(const CartesianInfo &pose)
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

bool JacoPose::Compare(const JacoPose &other, float tolerance) const
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

float JacoPose::Normalize(float value)
{
	while (value > 2 * M_PI)
		value -= 2 * M_PI;
	while (value < 0)
		value += 2 * M_PI;

	return value;
}

JacoAngles::JacoAngles(const jaco_driver::JointAngles &angles)
{
	Actuator1 = Normalize(180.0 - (angles.Angle_J1 * (180.0 / M_PI)));
	Actuator2 = Normalize((angles.Angle_J2 * (180.0 / M_PI)) + 270.0);
	Actuator3 = Normalize(90.0 - (angles.Angle_J3 * (180.0 / M_PI)));
	Actuator4 = Normalize(180.0 - (angles.Angle_J4 * (180.0 / M_PI)));
	Actuator5 = Normalize(180.0 - (angles.Angle_J5 * (180.0 / M_PI)));
	Actuator6 = Normalize(260.0 - (angles.Angle_J6 * (180.0 / M_PI)));
}

JacoAngles::JacoAngles(const AngularInfo &angles)
{
	Actuator1 = Normalize(angles.Actuator1);
	Actuator2 = Normalize(angles.Actuator2);
	Actuator3 = Normalize(angles.Actuator3);
	Actuator4 = Normalize(angles.Actuator4);
	Actuator5 = Normalize(angles.Actuator5);
	Actuator6 = Normalize(angles.Actuator6);
}

jaco_driver::JointAngles JacoAngles::Angles()
{
	jaco_driver::JointAngles angles;
	angles.Angle_J1 = (180.0 - Actuator1) / (180.0 / M_PI);
	angles.Angle_J2 = (Actuator2 - 270.0) / (180.0 / M_PI);
	angles.Angle_J3 = (90.0 - Actuator3) / (180.0 / M_PI);
	angles.Angle_J4 = (180.0 - Actuator4) / (180.0 / M_PI);
	angles.Angle_J5 = (180.0 - Actuator5) / (180.0 / M_PI);
	angles.Angle_J6 = (260.0 - Actuator6) / (180.0 / M_PI);

	return angles;
}

bool JacoAngles::Compare(const JacoAngles &other, float tolerance) const 
{
	bool status = true;

	status = status && CompareValues(Actuator1, other.Actuator1, tolerance);
	status = status && CompareValues(Actuator2, other.Actuator2, tolerance);
	status = status && CompareValues(Actuator3, other.Actuator3, tolerance);
	status = status && CompareValues(Actuator4, other.Actuator4, tolerance);
	status = status && CompareValues(Actuator5, other.Actuator5, tolerance);
	status = status && CompareValues(Actuator6, other.Actuator6, tolerance);

	return status;
}

float JacoAngles::Normalize(float value)
{
	while (value > 360.0)
		value -= 360.0;
	while (value < 0.0)
		value += 360.0;

	return value;
}

}
