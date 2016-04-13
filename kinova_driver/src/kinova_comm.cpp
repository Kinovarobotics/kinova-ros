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
 *  File: kinova_comm.cpp
 *  Desc: Class for moving/querying kinova arm.
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
#include "kinova_driver/kinova_comm.h"
#include <string>
#include <vector>


namespace kinova
{

KinovaComm::KinovaComm(const ros::NodeHandle& node_handle,
                   boost::recursive_mutex &api_mutex,
                   const bool is_movement_on_start)
    : is_software_stop_(false), api_mutex_(api_mutex)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    int result = NO_ERROR_KINOVA;

    // Get the serial number parameter for the arm we wish to connec to
    std::string serial_number = "";
    node_handle.getParam("serial_number", serial_number);

    int api_version[API_VERSION_COUNT];
    result = kinova_api_.getAPIVersion(api_version);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the Kinova API version", result);
    }

    ROS_INFO_STREAM("Initializing Kinova API (header version: " << COMMAND_LAYER_VERSION << ", library version: "
                    << api_version[0] << "." << api_version[1] << "." << api_version[2] << ")");

    result = kinova_api_.initAPI();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not initialize Kinova API", result);
    }

    KinovaDevice devices_list[MAX_KINOVA_DEVICE];
    result = NO_ERROR_KINOVA;
    kinova_api_.getDevices(devices_list, result);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get devices list", result);
    }

    int devices_count = kinova_api_.getDeviceCount(result);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get devices list count.", result);
    }

    bool found_arm = false;
    for (int device_i = 0; device_i < devices_count; device_i++)
    {
        // If no device is specified, just use the first available device
        if ((serial_number == "")
            || (std::strcmp(serial_number.c_str(), devices_list[device_i].SerialNumber) == 0))
        {
            result = kinova_api_.setActiveDevice(devices_list[device_i]);
            if (result != NO_ERROR_KINOVA)
            {
                throw KinovaCommException("Could not set the active device", result);
            }

            GeneralInformations general_info;
            result = kinova_api_.getGeneralInformations(general_info);
            if (result != NO_ERROR_KINOVA)
            {
                throw KinovaCommException("Could not get general information about the device", result);
            }

            ClientConfigurations configuration;
            getConfig(configuration);

            QuickStatus quick_status;
            getQuickStatus(quick_status);

            robot_type_ = quick_status.RobotType;
            switch (robot_type_) {
                case 0:
                case 3:
                case 4:
                case 6:
                    num_fingers_ = 3;
                    break;
                case 1:
                case 2:
                case 5:
                    num_fingers_ = 2;
                    break;
                default:
                    ROS_ERROR("Unknown robot type: %d", quick_status.RobotType);
                    throw KinovaCommException("Could not recognize the type of the arm", quick_status.RobotType);
                    break;
            };

            ROS_INFO_STREAM("Found " << devices_count << " device(s), using device at index " << device_i
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
                  serial_number.c_str(), devices_count);
        throw KinovaCommException("Could not find the specified arm", 0);
    }

    // On a cold boot the arm may not respond to commands from the API right away.
    // This kick-starts the Control API so that it's ready to go.
    startAPI();
    stopAPI();
    startAPI();

    // Set the angular velocity of each of the joints to zero
    TrajectoryPoint kinova_velocity;
    memset(&kinova_velocity, 0, sizeof(kinova_velocity));
    setCartesianVelocities(kinova_velocity.Position.CartesianPosition);

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


KinovaComm::~KinovaComm()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    kinova_api_.closeAPI();
}


/*!
 * \brief Determines whether the arm has returned to its "Home" state.
 *
 * Checks the current joint angles, then compares them to the known "Home"
 * joint angles.
 */
bool KinovaComm::isHomed(void)
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
void KinovaComm::homeArm(void)
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
    int result = kinova_api_.moveHome();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Move home failed", result);
    }
}


/*!
 * \brief Initialize finger actuators.
 *
 * Move fingers to the full-open position to initialize them for use.
 * Note, The this routine requires firmware version 5.05.x (or higher?).
 */
void KinovaComm::initFingers(void)
{
    ROS_INFO("Initializing fingers...this will take a few seconds and the fingers should open completely");
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = kinova_api_.initFingers();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not init fingers", result);
    }
    return;
}


/*!
 * \brief Sends a joint angle command to the Kinova arm.
 *
 * Waits until the arm has stopped moving before releasing control of the API.
 */
void KinovaComm::setJointAngles(const KinovaAngles &angles, int timeout, bool push)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The angles could not be set because the arm is stopped");
        return;
    }

    int result = NO_ERROR_KINOVA;
    TrajectoryPoint kinova_position;
    kinova_position.InitStruct();
    memset(&kinova_position, 0, sizeof(kinova_position));  // zero structure

    if (push)
    {
        result = kinova_api_.eraseAllTrajectories();
        if (result != NO_ERROR_KINOVA)
        {
            throw KinovaCommException("Could not erase trajectories", result);
        }
    }

    //startAPI();

    result = kinova_api_.setAngularControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set angular control", result);
    }

    kinova_position.Position.Delay = 0.0;
    kinova_position.Position.Type = ANGULAR_POSITION;
    kinova_position.Position.Actuators = angles;

    result = kinova_api_.sendAdvanceTrajectory(kinova_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send advanced joint angle trajectory", result);
    }
}


/*!
 * \brief Sends a cartesian coordinate trajectory to the Kinova arm.
 *
 * Waits until the arm has stopped moving before releasing control of the API.
 */
void KinovaComm::setCartesianPosition(const KinovaPose &position, int timeout, bool push)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The position could not be set because the arm is stopped");
        return;
    }

    int result = NO_ERROR_KINOVA;
    TrajectoryPoint kinova_position;
    kinova_position.InitStruct();
    memset(&kinova_position, 0, sizeof(kinova_position));  // zero structure

    if (push)
    {
        result = kinova_api_.eraseAllTrajectories();
        if (result != NO_ERROR_KINOVA)
        {
            throw KinovaCommException("Could not erase trajectories", result);
        }
    }

    //startAPI();

    result = kinova_api_.setCartesianControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set Cartesian control", result);
    }

    kinova_position.Position.Delay = 0.0;
    kinova_position.Position.Type = CARTESIAN_POSITION;
    kinova_position.Position.HandMode = HAND_NOMOVEMENT;

    // These values will not be used but are initialized anyway.
    kinova_position.Position.Actuators.Actuator1 = 0.0f;
    kinova_position.Position.Actuators.Actuator2 = 0.0f;
    kinova_position.Position.Actuators.Actuator3 = 0.0f;
    kinova_position.Position.Actuators.Actuator4 = 0.0f;
    kinova_position.Position.Actuators.Actuator5 = 0.0f;
    kinova_position.Position.Actuators.Actuator6 = 0.0f;

    kinova_position.Position.CartesianPosition = position;

    result = kinova_api_.sendBasicTrajectory(kinova_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send basic trajectory", result);
    }
}


/*!
 * \brief Sets the finger positions
 */
void KinovaComm::setFingerPositions(const FingerAngles &fingers, int timeout, bool push)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The fingers could not be set because the arm is stopped");
        return;
    }

    int result = NO_ERROR_KINOVA;
    TrajectoryPoint kinova_position;
    kinova_position.InitStruct();
    memset(&kinova_position, 0, sizeof(kinova_position));  // zero structure

    if (push)
    {
        result = kinova_api_.eraseAllTrajectories();
        if (result != NO_ERROR_KINOVA)
        {
            throw KinovaCommException("Could not erase trajectories", result);
        }
    }

    //startAPI();

    result = kinova_api_.setAngularControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set Cartesian control", result);
    }

    // Initialize Cartesian control of the fingers
    kinova_position.Position.HandMode = POSITION_MODE;
    kinova_position.Position.Type = ANGULAR_POSITION;
    kinova_position.Position.Fingers = fingers;
    kinova_position.Position.Delay = 0.0;
    kinova_position.LimitationsActive = 0;

    AngularPosition kinova_angles;
    memset(&kinova_angles, 0, sizeof(kinova_angles));  // zero structure

    result = kinova_api_.getAngularPosition(kinova_angles);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the angular position", result);
    }


    kinova_position.Position.Actuators = kinova_angles.Actuators;

    // When loading a cartesian position for the fingers, values are required for the arm joints
    // as well or the arm goes nuts.  Grab the current position and feed it back to the arm.
    KinovaPose pose;
    getCartesianPosition(pose);
    kinova_position.Position.CartesianPosition = pose;

    result = kinova_api_.sendAdvanceTrajectory(kinova_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send advanced finger trajectory", result);
    }
}


/*!
 * \brief Set the angular velocity of the joints
 */
void KinovaComm::setJointVelocities(const AngularInfo &joint_vel)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The velocities could not be set because the arm is stopped");
        return;
    }

    TrajectoryPoint kinova_velocity;
    kinova_velocity.InitStruct();

    memset(&kinova_velocity, 0, sizeof(kinova_velocity));  // zero structure

    //startAPI();
    kinova_velocity.Position.Type = ANGULAR_VELOCITY;

    // confusingly, velocity is passed in the position struct
    kinova_velocity.Position.Actuators = joint_vel;

    int result = kinova_api_.sendAdvanceTrajectory(kinova_velocity);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send advanced joint velocity trajectory", result);
    }
}


/*!
 * \brief Set the cartesian velocity of the tool tip
 */
void KinovaComm::setCartesianVelocities(const CartesianInfo &velocities)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The cartesian velocities could not be set because the arm is stopped");
        kinova_api_.eraseAllTrajectories();
        return;
    }

    TrajectoryPoint kinova_velocity;
    kinova_velocity.InitStruct();

    memset(&kinova_velocity, 0, sizeof(kinova_velocity));  // zero structure

    //startAPI();
    kinova_velocity.Position.Type = CARTESIAN_VELOCITY;

    // confusingly, velocity is passed in the position struct
    kinova_velocity.Position.CartesianPosition = velocities;

    int result = kinova_api_.sendAdvanceTrajectory(kinova_velocity);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send advanced Cartesian velocity trajectory", result);
    }
}


/*!
 * \brief Obtains the current arm configuration.
 *
 * This is the configuration which are stored on the arm itself. Many of these
 * configurations may be set using the Windows interface.
 */
void KinovaComm::setConfig(const ClientConfigurations &config)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = kinova_api_.setClientConfigurations(config);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set the client configuration", result);
    }
}


/*!
 * \brief API call to obtain the current angular position of all the joints.
 */
void KinovaComm::getJointAngles(KinovaAngles &angles)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    AngularPosition kinova_angles;
    memset(&kinova_angles, 0, sizeof(kinova_angles));  // zero structure

    int result = kinova_api_.getAngularPosition(kinova_angles);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the angular position", result);
    }

    angles = KinovaAngles(kinova_angles.Actuators);
}

/*!
 * \brief API call to obtain the current angular velocities of all the joints.
 */
void KinovaComm::getJointVelocities(KinovaAngles &vels)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    AngularPosition kinova_vels;
    memset(&kinova_vels, 0, sizeof(kinova_vels));  // zero structure

    int result = kinova_api_.getAngularVelocity(kinova_vels);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the angular velocity", result);
    }

    vels = KinovaAngles(kinova_vels.Actuators);
}

/*!
 * \brief API call to obtain the current torque of all the joints.
 */
void KinovaComm::getJointTorques(KinovaAngles &tqs)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    AngularPosition kinova_tqs;
    memset(&kinova_tqs, 0, sizeof(kinova_tqs));  // zero structure

    int result = kinova_api_.getAngularForce(kinova_tqs);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the joint torques", result);
    }

    tqs = KinovaAngles(kinova_tqs.Actuators);
}
/*!
 * \brief API call to obtain the current cartesian position of the arm.
 */
void KinovaComm::getCartesianPosition(KinovaPose &position)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    CartesianPosition kinova_cartesian_position;
    memset(&kinova_cartesian_position, 0, sizeof(kinova_cartesian_position));  // zero structure

    int result = kinova_api_.getCartesianPosition(kinova_cartesian_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the Cartesian position", result);
    }

    position = KinovaPose(kinova_cartesian_position.Coordinates);
}

/*!
 * \brief API call to obtain the current cartesian force of the arm.
 */
void KinovaComm::getCartesianForce(KinovaPose &cart_force)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    CartesianPosition kinova_cartesian_force;
    memset(&kinova_cartesian_force, 0, sizeof(kinova_cartesian_force));  // zero structure

    int result = kinova_api_.getCartesianForce(kinova_cartesian_force);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the Cartesian force", result);
    }

    cart_force = KinovaPose(kinova_cartesian_force.Coordinates);
}

/*!
 * \brief API call to obtain the current finger positions.
 */
void KinovaComm::getFingerPositions(FingerAngles &fingers)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    CartesianPosition kinova_cartesian_position;
    memset(&kinova_cartesian_position, 0, sizeof(kinova_cartesian_position));  // zero structure

    int result = kinova_api_.getCartesianPosition(kinova_cartesian_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get Cartesian finger position", result);
    }

    if (num_fingers_ == 2)
    {
        kinova_cartesian_position.Fingers.Finger3 = 0.0;
    }

    fingers = FingerAngles(kinova_cartesian_position.Fingers);
}

/*!
 * \brief Set the cartesian inertia and damping parameters for force control.
 */
void KinovaComm::setCartesianInertiaDamping(const CartesianInfo &inertia, const CartesianInfo& damping)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = kinova_api_.setCartesianInertiaDamping(inertia, damping);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set cartesian inertia and damping", result);
    }
}

/*!
 * \brief Set the cartesian min and max force parameters for force control.
 */
void KinovaComm::setCartesianForceMinMax(const CartesianInfo &min, const CartesianInfo& max)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = kinova_api_.setCartesianForceMinMax(min, max);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set cartesian min/max force.", result);
    }
}

/*!
 * \brief Start cartesian force control.
 */
void KinovaComm::startForceControl()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = kinova_api_.startForceControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not start force control.", result);
    }
}

/*!
 * \brief Stop cartesian force control.
 */
void KinovaComm::stopForceControl()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = kinova_api_.stopForceControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not stop force control.", result);
    }
}

/*!
 * \brief Set the end effector offset
 */
void KinovaComm::setEndEffectorOffset(float x, float y, float z)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    // Recopy the current status:
    float tx, ty, tz;
    unsigned int status;
    int result = kinova_api_.getEndEffectorOffset(status, tx, ty, tz);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get current end effector offset.", result);
    }

    result = kinova_api_.setEndEffectorOffset(status, x, y, z);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set end effector offset.", result);
    }
}

/*!
 * \brief API call to obtain the current client configuration.
 */
void KinovaComm::getConfig(ClientConfigurations &config)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&config, 0, sizeof(config));  // zero structure

    int result = kinova_api_.getClientConfigurations(config);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get client configuration", result);
    }
}


/*!
 * \brief API call to obtain the current "quick status".
 */
void KinovaComm::getQuickStatus(QuickStatus &quick_status)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&quick_status, 0, sizeof(quick_status));  // zero structure
    int result = kinova_api_.getQuickStatus(quick_status);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get quick status", result);
    }
}


void KinovaComm::stopAPI()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    is_software_stop_ = true;

    int result = kinova_api_.stopControlAPI();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not stop the control API", result);
    }

    result = kinova_api_.eraseAllTrajectories();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not erase all trajectories", result);
    }
}


void KinovaComm::startAPI()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    if (is_software_stop_)
    {
        is_software_stop_ = false;
        kinova_api_.stopControlAPI();
        ros::Duration(0.05).sleep();
    }

    int result = kinova_api_.startControlAPI();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not start the control API", result);
    }
}


int KinovaComm::numFingers() const
{
    return num_fingers_;
}

int KinovaComm::robotType() const
{
    return robot_type_;
}

/*!
 * \brief Dumps the current joint angles onto the screen.
 */
void KinovaComm::printAngles(const KinovaAngles &angles)
{
    ROS_INFO("Joint angles (deg) -- J1: %f, J2: %f J3: %f, J4: %f, J5: %f, J6: %f",
             angles.Actuator1, angles.Actuator2, angles.Actuator3,
             angles.Actuator4, angles.Actuator5, angles.Actuator6);
}


/*!
 * \brief Dumps the current cartesian positions onto the screen.
 */
void KinovaComm::printPosition(const KinovaPose &position)
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
void KinovaComm::printFingers(const FingersPosition &fingers)
{
    ROS_INFO("Finger positions -- F1: %f, F2: %f, F3: %f",
             fingers.Finger1, fingers.Finger2, fingers.Finger3);
}


/*!
 * \brief Dumps the client configuration onto the screen.
 */
void KinovaComm::printConfig(const ClientConfigurations &config)
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


bool KinovaComm::isStopped()
{
    return is_software_stop_;
}

//double KinovaComm::j6o() const
//{
//    // J6 offset is 260 for Kinova R1 (type 0), and 270 for Mico and Kinova R2.
//    return robotType() == 0 ? 260.0 : 270.0;
//}

}  // namespace kinova
