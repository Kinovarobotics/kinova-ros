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
#include <string>
#include <vector>


namespace jaco
{

JacoComm::JacoComm(const ros::NodeHandle& node_handle,
                   boost::recursive_mutex &api_mutex,
                   const bool is_movement_on_start)
    : is_software_stop_(false), api_mutex_(api_mutex)
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
    for (int device_i = 0; device_i < devices_list.size(); device_i++)
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

            robot_type_ = quick_status.RobotType;
            if ((robot_type_ != 0) && (robot_type_ != 1) && (robot_type_ != 3))
            {
                ROS_ERROR("Could not get the type of the arm from the quick status, expected "
                          "either type 0 (JACO), or type 1 (MICO), got %d", quick_status.RobotType);
                throw JacoCommException("Could not get the type of the arm", quick_status.RobotType);
            }

            switch (robot_type_) {
                case 0:
                case 3:
                    num_fingers_ = 3;
                    break;
                case 1:
                    num_fingers_ = 2;
                    break;
                default:
                    break;
            }

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
                  serial_number.c_str(), static_cast<int>(devices_list.size()));
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
    getQuickStatus(quick_status);

    if (quick_status.RetractType == 1)
    {
        return true;
    }
    else
    {
        return false;
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
void JacoComm::setJointAngles(const JacoAngles &angles, int timeout, bool push)
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

    //startAPI();

    result = jaco_api_.setAngularControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not set angular control", result);
    }

    jaco_position.Position.Delay = 0.0;
    jaco_position.Position.Type = ANGULAR_POSITION;
    jaco_position.Position.Actuators = angles;

    result = jaco_api_.sendAdvanceTrajectory(jaco_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not send advanced joint angle trajectory", result);
    }
}


/*!
 * \brief Sends a cartesian coordinate trajectory to the Jaco arm.
 *
 * Waits until the arm has stopped moving before releasing control of the API.
 */
void JacoComm::setCartesianPosition(const JacoPose &position, int timeout, bool push)
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

    //startAPI();

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

    result = jaco_api_.sendBasicTrajectory(jaco_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not send basic trajectory", result);
    }
}


/*!
 * \brief Sets the finger positions
 */
void JacoComm::setFingerPositions(const FingerAngles &fingers, int timeout, bool push)
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

    //startAPI();

    result = jaco_api_.setAngularControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not set Cartesian control", result);
    }

    // Initialize Cartesian control of the fingers
    jaco_position.Position.HandMode = POSITION_MODE;
    jaco_position.Position.Type = ANGULAR_POSITION;
    jaco_position.Position.Fingers = fingers;
    jaco_position.Position.Delay = 0.0;
    jaco_position.LimitationsActive = 0;

    AngularPosition jaco_angles;
    memset(&jaco_angles, 0, sizeof(jaco_angles));  // zero structure

    result = jaco_api_.getAngularPosition(jaco_angles);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get the angular position", result);
    }


    jaco_position.Position.Actuators = jaco_angles.Actuators;

    // When loading a cartesian position for the fingers, values are required for the arm joints
    // as well or the arm goes nuts.  Grab the current position and feed it back to the arm.
    JacoPose pose;
    getCartesianPosition(pose);
    jaco_position.Position.CartesianPosition = pose;

    result = jaco_api_.sendAdvanceTrajectory(jaco_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not send advanced finger trajectory", result);
    }
}


/*!
 * \brief Set the angular velocity of the joints
 */
void JacoComm::setJointVelocities(const AngularInfo &joint_vel)
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

    //startAPI();
    jaco_velocity.Position.Type = ANGULAR_VELOCITY;

    // confusingly, velocity is passed in the position struct
    jaco_velocity.Position.Actuators = joint_vel;

    int result = jaco_api_.sendAdvanceTrajectory(jaco_velocity);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not send advanced joint velocity trajectory", result);
    }
}


/*!
 * \brief Set the cartesian velocity of the tool tip
 */
void JacoComm::setCartesianVelocities(const CartesianInfo &velocities)
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

    //startAPI();
    jaco_velocity.Position.Type = CARTESIAN_VELOCITY;

    // confusingly, velocity is passed in the position struct
    jaco_velocity.Position.CartesianPosition = velocities;

    int result = jaco_api_.sendAdvanceTrajectory(jaco_velocity);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not send advanced Cartesian velocity trajectory", result);
    }
}


/*!
 * \brief Obtains the current arm configuration.
 *
 * This is the configuration which are stored on the arm itself. Many of these
 * configurations may be set using the Windows interface.
 */
void JacoComm::setConfig(const ClientConfigurations &config)
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

    int result = jaco_api_.getAngularPosition(jaco_angles);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get the angular position", result);
    }

    angles = JacoAngles(jaco_angles.Actuators);
}

/*!
 * \brief API call to obtain the current angular velocities of all the joints.
 */
void JacoComm::getJointVelocities(JacoAngles &vels)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    AngularPosition jaco_vels;
    memset(&jaco_vels, 0, sizeof(jaco_vels));  // zero structure

    int result = jaco_api_.getAngularVelocity(jaco_vels);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get the angular velocity", result);
    }

    vels = JacoAngles(jaco_vels.Actuators);
}

/*!
 * \brief API call to obtain the current torque of all the joints.
 */
void JacoComm::getJointTorques(JacoAngles &tqs)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    AngularPosition jaco_tqs;
    memset(&jaco_tqs, 0, sizeof(jaco_tqs));  // zero structure

    int result = jaco_api_.getAngularForce(jaco_tqs);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get the joint torques", result);
    }

    tqs = JacoAngles(jaco_tqs.Actuators);
}
/*!
 * \brief API call to obtain the current cartesian position of the arm.
 */
void JacoComm::getCartesianPosition(JacoPose &position)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    CartesianPosition jaco_cartesian_position;
    memset(&jaco_cartesian_position, 0, sizeof(jaco_cartesian_position));  // zero structure

    int result = jaco_api_.getCartesianPosition(jaco_cartesian_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get the Cartesian position", result);
    }

    position = JacoPose(jaco_cartesian_position.Coordinates);
}

/*!
 * \brief API call to obtain the current cartesian force of the arm.
 */
void JacoComm::getCartesianForce(JacoPose &cart_force)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    CartesianPosition jaco_cartesian_force;
    memset(&jaco_cartesian_force, 0, sizeof(jaco_cartesian_force));  // zero structure

    int result = jaco_api_.getCartesianForce(jaco_cartesian_force);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get the Cartesian force", result);
    }

    cart_force = JacoPose(jaco_cartesian_force.Coordinates);
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

    fingers = FingerAngles(jaco_cartesian_position.Fingers);
}

/*!
 * \brief Set the cartesian inertia and damping parameters for force control.
 */
void JacoComm::setCartesianInertiaDamping(const CartesianInfo &inertia, const CartesianInfo& damping)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = jaco_api_.setCartesianInertiaDamping(inertia, damping);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not set cartesian inertia and damping", result);
    }
}

/*!
 * \brief Set the cartesian min and max force parameters for force control.
 */
void JacoComm::setCartesianForceMinMax(const CartesianInfo &min, const CartesianInfo& max)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = jaco_api_.setCartesianForceMinMax(min, max);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not set cartesian min/max force.", result);
    }
}

/*!
 * \brief Start cartesian force control.
 */
void JacoComm::startForceControl()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = jaco_api_.startForceControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not start force control.", result);
    }
}

/*!
 * \brief Stop cartesian force control.
 */
void JacoComm::stopForceControl()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = jaco_api_.stopForceControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not stop force control.", result);
    }
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

int JacoComm::robotType()
{
    return robot_type_;
}

/*!
 * \brief Dumps the current joint angles onto the screen.
 */
void JacoComm::printAngles(const JacoAngles &angles)
{
    ROS_INFO("Joint angles (deg) -- J1: %f, J2: %f J3: %f, J4: %f, J5: %f, J6: %f",
             angles.Actuator1, angles.Actuator2, angles.Actuator3,
             angles.Actuator4, angles.Actuator5, angles.Actuator6);
}


/*!
 * \brief Dumps the current cartesian positions onto the screen.
 */
void JacoComm::printPosition(const JacoPose &position)
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
void JacoComm::printFingers(const FingersPosition &fingers)
{
    ROS_INFO("Finger positions -- F1: %f, F2: %f, F3: %f",
             fingers.Finger1, fingers.Finger2, fingers.Finger3);
}


/*!
 * \brief Dumps the client configuration onto the screen.
 */
void JacoComm::printConfig(const ClientConfigurations &config)
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


}  // namespace jaco
