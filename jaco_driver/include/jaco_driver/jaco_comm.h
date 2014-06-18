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
 *  File: jaco_comm.h
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

#ifndef _JACO_COMM_H_
#define _JACO_COMM_H_

#include <kinova/KinovaTypes.h>
#include <jaco_driver/jaco_types.h>
#include "jaco_driver/jaco_api.h"
#include <boost/thread/recursive_mutex.hpp>


namespace jaco {

class JacoComm {
 public:
    JacoComm(JacoAngles);
    ~JacoComm();

    bool isHomed(void);
    void homeArm(void);
    void initFingers(void);
    void setJointAngles(JacoAngles &angles, int timeout = 0, bool push = true);
    void setCartesianPosition(JacoPose &position, int timeout = 0, bool push = true);
    void setFingerPositions(FingerAngles &fingers, int timeout = 0, bool push = true);
    void setVelocities(AngularInfo joint_vel);
    void setCartesianVelocities(CartesianInfo velocities);
    void setConfig(ClientConfigurations config);
    void getJointAngles(JacoAngles &angles);
    void getCartesianPosition(JacoPose &position);
    void getFingerPositions(FingerAngles &fingers);
    void getQuickStatus(QuickStatus &quick_status);
    // void getForcesInfo(ForcesInfo &forces);
    void getConfig(ClientConfigurations &config);
    void printAngles(JacoAngles &angles);
    void printPosition(JacoPose &position);
    void printFingers(FingersPosition fingers);
    void printConfig(ClientConfigurations config);
    void stop();
    void start();
    bool isStopped();

 private:
    void waitForHome(int);

    boost::recursive_mutex api_mutex_;
    jaco::JacoAPI jaco_api_;
    bool is_software_stop_;
    JacoAngles home_position_;
};

}
#endif // _JACO_COMM_H_
