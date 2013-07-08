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
 *  File: jaco_types.h
 *  Desc: Wrappers around Kinova types.
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

#ifndef _JACO_TYPES_H_
#define _JACO_TYPES_H_

#include <jaco_driver/KinovaTypes.h>
#include <geometry_msgs/Pose.h>
#include <jaco_driver/JointAngles.h>
#include <jaco_driver/FingerPosition.h>

namespace jaco
{

class JacoPose : public CartesianInfo
{
	public:
	JacoPose() {}
	JacoPose(const geometry_msgs::Pose &);
	JacoPose(const CartesianInfo &);

	geometry_msgs::Pose Pose();
	bool Compare(const JacoPose &, float) const;

	private:
	float Normalize(float);
};

class JacoAngles : public AngularInfo
{
	public:
	JacoAngles() {}
	JacoAngles(const jaco_driver::JointAngles &);
	JacoAngles(const AngularInfo &);

	jaco_driver::JointAngles Angles();
	bool Compare(const JacoAngles &, float) const;

	private:
	float Normalize(float);
};


class FingerAngles : public FingersPosition
{
	public:
	FingerAngles() {}
	FingerAngles(const jaco_driver::FingerPosition &);
	FingerAngles(const FingersPosition &);

	jaco_driver::FingerPosition Fingers();
	bool Compare(const FingerAngles &, float) const;
};


}

#endif // _JACO_TYPES_H_
