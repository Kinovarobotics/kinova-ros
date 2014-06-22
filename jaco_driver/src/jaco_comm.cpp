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
 *  File: jaco_comm.cpp
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

#include <ros/ros.h>
#include "jaco_driver/jaco_comm.h"


namespace jaco
{


JacoComm::JacoComm(const ros::NodeHandle& node_handle,
                   boost::recursive_mutex& api_mutex,
                   const bool is_movement_on_start)
    : is_software_stop_(false), api_mutex_(api_mutex)  //, home_position_(getHomePosition(nodeHandle))
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    // Get the serial number parameter for the arm we wish to connec to
    std::string serial_number = "";
    node_handle.getParam("serial_number", serial_number);

    std::vector<int> api_version;
    int result = jaco_api_.getAPIVersion(api_version);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get the Kinova API version", result);
    }

    ROS_INFO_STREAM("Initializing Kinova API (header version: " << COMMAND_LAYER_VERSION << ", library version: "
                    << api_version[0] << "." << api_version[1] << "." << api_version[2] << ")");

    result = jaco_api_.initAPI();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not initialize Kinova API", result);
    }

    std::vector<KinovaDevice> devices_list;
    result = NO_ERROR_KINOVA;
    jaco_api_.getDevices(devices_list, result);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get devices list", result);
    }

    bool found_arm = false;
    for(int device_i = 0; device_i < devices_list.size(); device_i++)
    {
        // If no device is specified, just use the first available device
        if ((serial_number == "")
            || (std::strcmp(serial_number.c_str(), devices_list[device_i].SerialNumber) == 0))
        {
            result = jaco_api_.setActiveDevice(devices_list[device_i]);
            if (result != NO_ERROR_KINOVA)
            {
                throw JacoCommException("Could not set the active device", result);
            }

            GeneralInformations general_info;
            result = jaco_api_.getGeneralInformations(general_info);
            if (result != NO_ERROR_KINOVA)
            {
                throw JacoCommException("Could not get general information about the device", result);
            }

            ClientConfigurations configuration;
            getConfig(configuration);

            QuickStatus quick_status;
            getQuickStatus(quick_status);

            if ((quick_status.RobotType != 0) && (quick_status.RobotType != 1))
            {
                ROS_ERROR("Could not get the type of the arm from the quick status, expected "
                          "either type 0 (JACO), or type 1 (MICO), got %d", quick_status.RobotType);
                throw JacoCommException("Could not get the type of the arm", quick_status.RobotType);
            }

            num_fingers_ = quick_status.RobotType == 0 ? 3 : 2;

            ROS_INFO_STREAM("Found " << devices_list.size() << " device(s), using device at index " << device_i
                            << " (model: " << configuration.Model
                            << ", serial number: " << devices_list[device_i].SerialNumber
                            << ", code version: " << general_info.CodeVersion
                            << ", code revision: " << general_info.CodeRevision << ")");

            found_arm = true;
            break;
        }
    }

    if (!found_arm)
    {
        ROS_ERROR("Could not find the specified arm (serial: %s) among the %d attached devices",
                  serial_number.c_str(), (int)devices_list.size());
        throw JacoCommException("Could not find the specified arm", 0);
    }

    // On a cold boot the arm may not respond to commands from the API right away.
    // This kick-starts the Control API so that it's ready to go.
    startAPI();
    stopAPI();
    startAPI();

    // Set the angular velocity of each of the joints to zero
    TrajectoryPoint jaco_velocity;
    memset(&jaco_velocity, 0, sizeof(jaco_velocity));
    setCartesianVelocities(jaco_velocity.Position.CartesianPosition);

    if (is_movement_on_start)
    {
        initFingers();
    }
    else
    {
        ROS_WARN("Movement on connection to the arm has been suppressed on initialization. You may "
                 "have to home the arm (through the home service) before movement is possible");
    }
}


JacoComm::~JacoComm()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    ROS_INFO("Closing API");
    jaco_api_.closeAPI();
}


/*!
 * \brief Determines whether the arm has returned to its "Home" state.
 *
 * Checks the current joint angles, then compares them to the known "Home"
 * joint angles.
 */
bool JacoComm::isHomed(void)
{
    QuickStatus quick_status;
    ROS_INFO_STREAM("Getting quick status to check if arm is homed");
    getQuickStatus(quick_status);

    if (quick_status.RetractType == 1)
    {
        ROS_INFO("Arm is homed");
        return 1;
    }
    else
    {
        ROS_INFO("Arm is not homed");
        return 0;
    }
}


/*!
 * \brief Send the arm to the "home" position.
 *
 * The code replicates the function of the "home" button on the user controller
 * by "pressing" the home button long enough for the arm to return to the home
 * position.
 *
 * Fingers are homed by manually opening them fully, then returning them to a
 * half-open position.
 */
void JacoComm::homeArm(void)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("Arm is stopped, cannot home");
        return;
    }
    else if (isHomed())
    {
        ROS_INFO("Arm is already in \"home\" position");
        return;
    }

    stopAPI();
    ros::Duration(1.0).sleep();
    startAPI();

    ROS_INFO("Homing the arm");
    int result = jaco_api_.moveHome();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Move home failed", result);
    }
}


/*!
 * \brief Initialize finger actuators.
 *
 * Move fingers to the full-open position to initialize them for use.
 * Note, The this routine requires firmware version 5.05.x (or higher?).
 */
void JacoComm::initFingers(void)
{
    ROS_INFO("Initializing fingers...this will take a few seconds and the fingers should open completely");
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = jaco_api_.initFingers();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not init fingers", result);
    }
    return;
}


/*!
 * \brief Sends a joint angle command to the Jaco arm.
 *
 * Waits until the arm has stopped moving before releasing control of the API.
 */
void JacoComm::setJointAngles(JacoAngles &angles, int timeout, bool push)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The angles could not be set because the arm is stopped");
        return;
    }

    int result = NO_ERROR_KINOVA;
    TrajectoryPoint jaco_position;
    jaco_position.InitStruct();
    memset(&jaco_position, 0, sizeof(jaco_position));  // zero structure

    if (push)
    {
        result = jaco_api_.eraseAllTrajectories();
        if (result != NO_ERROR_KINOVA)
        {
            throw JacoCommException("Could not erase trajectories", result);
        }
    }

    startAPI();

    result = jaco_api_.setAngularControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not set angular control", result);
    }

    //Jaco_Position.LimitationsActive = false;
    jaco_position.Position.Delay = 0.0;
    jaco_position.Position.Type = ANGULAR_POSITION;
    jaco_position.Position.Actuators = angles;

    result = jaco_api_.sendAdvanceTrajectory(jaco_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not send adanced joint angle trajectory", result);
    }
}


/*!
 * \brief Sends a cartesian coordinate trajectory to the Jaco arm.
 *
 * Waits until the arm has stopped moving before releasing control of the API.
 */
void JacoComm::setCartesianPosition(JacoPose &position, int timeout, bool push)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The position could not be set because the arm is stopped");
        return;
    }

    int result = NO_ERROR_KINOVA;
    TrajectoryPoint jaco_position;
    jaco_position.InitStruct();
    memset(&jaco_position, 0, sizeof(jaco_position));  // zero structure

    if (push)
    {
        result = jaco_api_.eraseAllTrajectories();
        if (result != NO_ERROR_KINOVA)
        {
            throw JacoCommException("Could not erase trajectories", result);
        }
    }

    startAPI();

    result = jaco_api_.setCartesianControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not set Cartesian control", result);
    }

    jaco_position.Position.Delay = 0.0;
    jaco_position.Position.Type = CARTESIAN_POSITION;
    jaco_position.Position.HandMode = HAND_NOMOVEMENT;

    // These values will not be used but are initialized anyway.
    jaco_position.Position.Actuators.Actuator1 = 0.0f;
    jaco_position.Position.Actuators.Actuator2 = 0.0f;
    jaco_position.Position.Actuators.Actuator3 = 0.0f;
    jaco_position.Position.Actuators.Actuator4 = 0.0f;
    jaco_position.Position.Actuators.Actuator5 = 0.0f;
    jaco_position.Position.Actuators.Actuator6 = 0.0f;

    jaco_position.Position.CartesianPosition = position;
    //Jaco_Position.Position.CartesianPosition.ThetaZ += 0.0001; // A workaround for a bug in the Kinova API

    result = jaco_api_.sendBasicTrajectory(jaco_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not send basic trajectory", result);
    }
}


/*!
 * \brief Sets the finger positions
 */
void JacoComm::setFingerPositions(FingerAngles &fingers, int timeout, bool push)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The fingers could not be set because the arm is stopped");
        return;
    }

    int result = NO_ERROR_KINOVA;
    TrajectoryPoint jaco_position;
    jaco_position.InitStruct();
    memset(&jaco_position, 0, sizeof(jaco_position));  // zero structure

    if (push)
    {
        result = jaco_api_.eraseAllTrajectories();
        if (result != NO_ERROR_KINOVA)
        {
            throw JacoCommException("Could not erase trajectories", result);
        }
    }

    startAPI();

    result = jaco_api_.setCartesianControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not set Cartesian control", result);
    }

    // Initialize Cartesian control of the fingers
    jaco_position.Position.HandMode = POSITION_MODE;
    jaco_position.Position.Type = CARTESIAN_POSITION;
    jaco_position.Position.Fingers = fingers;
    jaco_position.Position.Delay = 0.0;
    jaco_position.LimitationsActive = 0;

    // These values will not be used but are initialized anyway.
    jaco_position.Position.Actuators.Actuator1 = 0.0f;
    jaco_position.Position.Actuators.Actuator2 = 0.0f;
    jaco_position.Position.Actuators.Actuator3 = 0.0f;
    jaco_position.Position.Actuators.Actuator4 = 0.0f;
    jaco_position.Position.Actuators.Actuator5 = 0.0f;
    jaco_position.Position.Actuators.Actuator6 = 0.0f;

    // When loading a cartesian position for the fingers, values are required for the arm joints
    // as well or the arm goes nuts.  Grab the current position and feed it back to the arm.
    JacoPose pose;
    getCartesianPosition(pose);

    jaco_position.Position.CartesianPosition.X = pose.X;
    jaco_position.Position.CartesianPosition.Y = pose.Y;
    jaco_position.Position.CartesianPosition.Z = pose.Z;
    jaco_position.Position.CartesianPosition.ThetaX = pose.ThetaX;
    jaco_position.Position.CartesianPosition.ThetaY = pose.ThetaY;
    jaco_position.Position.CartesianPosition.ThetaZ = pose.ThetaZ;

    result = jaco_api_.sendAdvanceTrajectory(jaco_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not send adanced finger trajectory", result);
    }
}


/*!
 * \brief Set the angular velocity of the joints
 */
void JacoComm::setJointVelocities(AngularInfo joint_vel)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The velocities could not be set because the arm is stopped");
        return;
    }

    TrajectoryPoint jaco_velocity;
    jaco_velocity.InitStruct();

    memset(&jaco_velocity, 0, sizeof(jaco_velocity));  // zero structure

    startAPI();
    jaco_velocity.Position.Type = ANGULAR_VELOCITY;

    // confusingly, velocity is passed in the position struct
    jaco_velocity.Position.Actuators = joint_vel;

    int result = jaco_api_.sendAdvanceTrajectory(jaco_velocity);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not send adanced joint velocity trajectory", result);
    }
}


/*!
 * \brief Set the cartesian velocity of the tool tip
 */
void JacoComm::setCartesianVelocities(CartesianInfo velocities)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The cartesian velocities could not be set because the arm is stopped");
        jaco_api_.eraseAllTrajectories();
        return;
    }

    TrajectoryPoint jaco_velocity;
    jaco_velocity.InitStruct();

    memset(&jaco_velocity, 0, sizeof(jaco_velocity));  // zero structure

    startAPI();
    jaco_velocity.Position.Type = CARTESIAN_VELOCITY;

    // confusingly, velocity is passed in the position struct
    jaco_velocity.Position.CartesianPosition = velocities;

    int result = jaco_api_.sendAdvanceTrajectory(jaco_velocity);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not send adanced Cartesian velocity trajectory", result);
    }
}


/*!
 * \brief Obtains the current arm configuration.
 *
 * This is the configuration which are stored on the arm itself. Many of these
 * configurations may be set using the Windows interface.
 */
void JacoComm::setConfig(ClientConfigurations config)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = jaco_api_.setClientConfigurations(config);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not set the client configuration", result);
    }
}


/*!
 * \brief API call to obtain the current angular position of all the joints.
 */
void JacoComm::getJointAngles(JacoAngles &angles)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    AngularPosition jaco_angles;
    memset(&jaco_angles, 0, sizeof(jaco_angles));  // zero structure

    ROS_INFO_STREAM(__LINE__);
    int result = jaco_api_.getAngularPosition(jaco_angles);
    ROS_INFO_STREAM(__LINE__);
    if (result != NO_ERROR_KINOVA)
    {
        ROS_INFO_STREAM(__LINE__);
        throw JacoCommException("Could not get the angular position", result);
        ROS_INFO_STREAM(__LINE__);
    }
    ROS_INFO_STREAM(__LINE__);

    angles = jaco_angles.Actuators;
}


/*!
 * \brief API call to obtain the current cartesian position of the arm.
 */
void JacoComm::getCartesianPosition(JacoPose &position)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    CartesianPosition jaco_cartesian_position;
    memset(&jaco_cartesian_position, 0, sizeof(jaco_cartesian_position));  // zero structure

    ROS_INFO_STREAM(__LINE__);
    int result = jaco_api_.getCartesianPosition(jaco_cartesian_position);
    ROS_INFO_STREAM(__LINE__);
    if (result != NO_ERROR_KINOVA)
    {
        ROS_INFO_STREAM(__LINE__);
        throw JacoCommException("Could not get the Cartesian position", result);
        ROS_INFO_STREAM(__LINE__);
    }
    ROS_INFO_STREAM(__LINE__);

    position = JacoPose(jaco_cartesian_position.Coordinates);
}


/*!
 * \brief API call to obtain the current finger positions.
 */
void JacoComm::getFingerPositions(FingerAngles &fingers)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    CartesianPosition jaco_cartesian_position;
    memset(&jaco_cartesian_position, 0, sizeof(jaco_cartesian_position));  // zero structure

    int result = jaco_api_.getCartesianPosition(jaco_cartesian_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get Cartesian finger position", result);
    }

    if (num_fingers_ == 2)
    {
        jaco_cartesian_position.Fingers.Finger3 = 0.0;
    }

    fingers = jaco_cartesian_position.Fingers;
}


/*!
 * \brief API call to obtain the current client configuration.
 */
void JacoComm::getConfig(ClientConfigurations &config)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&config, 0, sizeof(config));  // zero structure

    int result = jaco_api_.getClientConfigurations(config);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get client configuration", result);
    }
}


/*!
 * \brief API call to obtain the current "quick status".
 */
void JacoComm::getQuickStatus(QuickStatus &quick_status)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&quick_status, 0, sizeof(quick_status));  // zero structure
    int result = jaco_api_.getQuickStatus(quick_status);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get quick status", result);
    }
}


void JacoComm::stopAPI()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    is_software_stop_ = true;

    int result = jaco_api_.stopControlAPI();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not stop the control API", result);
    }

    result = jaco_api_.eraseAllTrajectories();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not erase all trajectories", result);
    }
}


void JacoComm::startAPI()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    if (is_software_stop_)
    {
        is_software_stop_ = false;
        jaco_api_.stopControlAPI();
        ros::Duration(0.05).sleep();
    }

    int result = jaco_api_.startControlAPI();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not start the control API", result);
    }
}


int JacoComm::numFingers()
{
    return num_fingers_;
}


/*!
 * \brief Dumps the current joint angles onto the screen.
 */
void JacoComm::printAngles(JacoAngles &angles)
{
    ROS_INFO("Joint angles (deg) -- J1: %f, J2: %f J3: %f, J4: %f, J5: %f, J6: %f",
             angles.Actuator1, angles.Actuator2, angles.Actuator3,
             angles.Actuator4, angles.Actuator5, angles.Actuator6);
}


/*!
 * \brief Dumps the current cartesian positions onto the screen.
 */
void JacoComm::printPosition(JacoPose &position)
{
    ROS_INFO("Arm position\n"
             "\tposition (m) -- x: %f, y: %f z: %f\n"
             "\trotation (rad) -- theta_x: %f, theta_y: %f, theta_z: %f",
             position.X, position.Y, position.Z,
             position.ThetaX, position.ThetaY, position.ThetaZ);
}


/*!
 * \brief Dumps the current finger positions onto the screen.
 */
void JacoComm::printFingers(FingersPosition fingers)
{
    ROS_INFO("Finger positions -- F1: %f, F2: %f, F3: %f",
             fingers.Finger1, fingers.Finger2, fingers.Finger3);
}


/*!
 * \brief Dumps the client configuration onto the screen.
 */
void JacoComm::printConfig(ClientConfigurations config)
{
    ROS_INFO_STREAM("Arm configuration:\n"
                    "\tClientID: " << config.ClientID <<
                    "\n\tClientName: " << config.ClientName <<
                    "\n\tOrganization: " << config.Organization <<
                    "\n\tSerial:" << config.Serial <<
                    "\n\tModel: " << config.Model <<
                    "\n\tMaxForce: " << config.MaxForce <<
                    "\n\tSensibility: " <<  config.Sensibility <<
                    "\n\tDrinkingHeight: " << config.DrinkingHeight <<
                    "\n\tComplexRetractActive: " << config.ComplexRetractActive <<
                    "\n\tRetractedPositionAngle: " << config.RetractedPositionAngle <<
                    "\n\tRetractedPositionCount: " << config.RetractedPositionCount <<
                    "\n\tDrinkingDistance: " << config.DrinkingDistance <<
                    "\n\tFingers2and3Inverted: " << config.Fingers2and3Inverted <<
                    "\n\tDrinkingLength: " << config.DrinkingLenght <<
                    "\n\tDeletePreProgrammedPositionsAtRetract: " <<
                    config.DeletePreProgrammedPositionsAtRetract <<
                    "\n\tEnableFlashErrorLog: " << config.EnableFlashErrorLog <<
                    "\n\tEnableFlashPositionLog: " << config.EnableFlashPositionLog);
}


bool JacoComm::isStopped()
{
    return is_software_stop_;
}


///*!
// * \brief Wait for the arm to reach the "home" position.
// *
// * \param timeout Timeout after which to give up waiting for arm to finish "homing".
// */
//void JacoComm::waitForHome(int timeout)
//{
//    double start_secs;
//    double current_secs;

//    // If ros is still running use rostime, else use system time
//    if (ros::ok())
//    {
//        start_secs = ros::Time::now().toSec();
//        current_secs = ros::Time::now().toSec();
//    }
//    else
//    {
//        start_secs = (double) time(NULL);
//        current_secs = (double) time(NULL);
//    }

//    // while we have not timed out
//    while ((current_secs - start_secs) < timeout)
//    {
//        ros::Duration(0.5).sleep();

//        //If ros is still running use rostime, else use system time
//        if (ros::ok())
//        {
//            current_secs = ros::Time::now().toSec();
//            ros::spinOnce();
//        }
//        else
//        {
//            current_secs = (double) time(NULL);
//        }

//        if (isHomed())
//        {
//            ROS_INFO("Arm has homed");
//            ros::Duration(1.0).sleep();  // Grants a bit more time for the arm to "settle"
//            return;
//        }
//        else
//        {
//            ROS_INFO("Still homing the arm");
//        }
//    }

//    ROS_WARN("Timed out waiting for arm to return home");
//}

}  // namespace jaco
