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

#ifndef JACO_DRIVER_JACO_TYPES_H
#define JACO_DRIVER_JACO_TYPES_H

#include <kinova/KinovaTypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <jaco_msgs/JointAngles.h>
#include <jaco_msgs/FingerPosition.h>

#include <string>


namespace jaco
{

class JacoException : public std::exception {};


class JacoCommException : public JacoException
{
 public:
    explicit JacoCommException(const std::string& message, const int error_code);
    ~JacoCommException() throw() {}

    const char* what() const throw();
 private:
    std::string desc_;
};


class JacoPose : public CartesianInfo
{
 public:
    JacoPose() {}
    explicit JacoPose(const geometry_msgs::Pose &pose);
    explicit JacoPose(const CartesianInfo &pose);

    geometry_msgs::Pose   constructPoseMsg();
    geometry_msgs::Wrench constructWrenchMsg();

    bool isCloseToOther(const JacoPose &, float tolerance) const;
};


class JacoAngles : public AngularInfo
{
 public:
    JacoAngles() {}
    explicit JacoAngles(const jaco_msgs::JointAngles &angles);
    explicit JacoAngles(const AngularInfo &angles);

    jaco_msgs::JointAngles constructAnglesMsg();
    bool isCloseToOther(const JacoAngles &, float tolerance) const;
};


class FingerAngles : public FingersPosition
{
 public:
    FingerAngles() {}
    explicit FingerAngles(const jaco_msgs::FingerPosition &position);
    explicit FingerAngles(const FingersPosition &angle);

    jaco_msgs::FingerPosition constructFingersMsg();
    bool isCloseToOther(const FingerAngles &, float tolerance) const;
};

}  // namespace jaco
#endif  // JACO_DRIVER_JACO_TYPES_H
