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
#include <KinovaTypes.h>

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


/**
 * @brief KinovaComm::startAPI API gains the control of robot
 */
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


/**
 * @brief KinovaComm::stopAPI API ceases the control of robot
 */
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


/**
 * @brief KinovaComm::isStopped check if API ceases the control of robot.
 * @return true if stopAPI() was called.
 */
bool KinovaComm::isStopped()
{
    return is_software_stop_;
}


/**
 * @brief KinovaComm::startForceControl Start cartesian force control.
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


/**
 * @brief KinovaComm::stopForceControl Stop cartesian force control.
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


/**
 * @brief KinovaComm::robotType get robotType
 *
 * Index for robot type:
 *  JACOV1_ASSISTIVE = 0,
 *  MICO_6DOF_SERVICE = 1,
 *  MICO_4DOF_SERVICE = 2,
 * 	JACOV2_6DOF_SERVICE = 3,
 * 	JACOV2_4DOF_SERVICE = 4,
 * 	MICO_6DOF_ASSISTIVE = 5,
 * 	JACOV2_6DOF_ASSISTIVE = 6,
 *
 * @return index of robot type
 */
int KinovaComm::robotType() const
{
    return robot_type_;
}


/**
 * @brief KinovaComm::getQuickStatus obtain the current "quick status".
 * @param quick_status This structure holds various informations but mostly it is flag status, such as robotype, retractType, forceControlStatus,
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


/**
 * @brief Obtains the current arm configuration.
 *
 * This is the configuration which are stored on the arm itself. Many of these configurations may be set using the Windows interface.
 *
 * @param config This structure holds informations relative to the client, including serial number, robot model, limits for position, velocity, acceleration and force, etc.
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


/**
 * @brief KinovaComm::getConfig obtain the current client configuration.
 * @param config This structure holds informations relative to the client, including serial number, robot model, limits for position, velocity, acceleration and force, etc.
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


/**
 * @brief KinovaComm::printConfig Dumps the client configuration onto the screen.
 * @param config This structure holds informations relative to the client, including serial number, robot model, limits for position, velocity, acceleration and force, etc.
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


/**
 * @brief KinovaComm::getControlType get current control type
 * The control in ROS is independent to Joystick control type. For example: even set robot in joint control though api in ROS, robot may still controlled in joint level by joystick.
 * @param controlType Cartesian control[0] or joint control[1]
 */
void KinovaComm::getControlType(int &controlType)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&controlType, 0, sizeof(controlType)); //zero structure
    int result = kinova_api_.getControlType(controlType);
    if (result!=NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get control type", result);
    }
}


/**
 * @brief KinovaComm::getGeneralInformations get almost all information of the robotical arm.
 * @param general_info includes: power statistics, sensor infos, robot position and command, etc.
 */
void KinovaComm::getGeneralInformations(GeneralInformations &general_info)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&general_info, 0, sizeof(general_info));
    int result = kinova_api_.getGeneralInformations(general_info);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get general information about the device", result);
    }
}


/**
 * @brief KinovaComm::getSensorsInfo get feedback from equipped sensors.
 * @param sensor_Info temperate and accelerometer info. voltage and current are included as well.
 */
void KinovaComm::getSensorsInfo(SensorsInfo &sensor_Info)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&sensor_Info, 0, sizeof(sensor_Info));
    int result = kinova_api_.getSensorsInfo(sensor_Info);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get sensors information", result);
    }
}


/**
 * @brief KinovaComm::getForcesInfo
 * @param force_Info joint torque and end-effector wrench in Nm and N.
 */
void KinovaComm::getForcesInfo(ForcesInfo &force_Info)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&force_Info, 0, sizeof(force_Info));
    int result = kinova_api_.getForcesInfo(force_Info);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get force information", result);
    }
}


/**
 * @brief KinovaComm::getGripperStatus
 * @param gripper_status most complete information of each fingers, including model, motion, force, limits, etc.
 */
void KinovaComm::getGripperStatus(Gripper &gripper_status)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&gripper_status, 0, sizeof(gripper_status));
    int result = kinova_api_.getGripperStatus(gripper_status);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the gripper status", result);
    }
}




/**
 * @brief KinovaComm::setAngularControl
 * If robot is not in motion, change control model to Angular control
 */
void KinovaComm::setAngularControl()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    TrajectoryFIFO trajectory_fifo;
    memset(&trajectory_fifo, 0, sizeof(trajectory_fifo));
    getGlobalTrajectoryInfo(trajectory_fifo);
    if(trajectory_fifo.TrajectoryCount > 0)
    {
        ROS_WARN("Current tranjectory count is %d, Please wait the trajectory to finish to swich to Angular control.", trajectory_fifo.TrajectoryCount);
        return;
    }
    int result = kinova_api_.setAngularControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set angular control", result);
    }
}


/**
 * @brief KinovaComm::getAngularCommand
 * @param angular_command {AngularInfo, FingersPosition}
 */
void KinovaComm::getAngularCommand(AngularPosition &angular_command)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&angular_command, 0, sizeof(angular_command));
    int result = kinova_api_.getAngularCommand(angular_command);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the angular command", result);
    }
}


/**
 * @brief Obtain current joint angles
 * @param angles in degrees
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


/**
 * @brief Sends a joint angle command to the Kinova arm.
 *
 * Waits until the arm has stopped moving before releasing control of
 *  the API. sendAdvanceTrajectory() is called in api to complete the motion.
 *
 * @param angles target joint angle to set, type float, unit in degree
 * @param timeout default value 0.0, not used.
 * @param push default true, errase all trajectory after motion.
 */
void KinovaComm::setJointAngles(const KinovaAngles &angles, int timeout, bool push)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_WARN_STREAM("In class [" << typeid(*this).name() << "], function ["<< __FUNCTION__ << "]: The angles could not be set because the arm is stopped" << std::endl);
        return;
    }

    int result = NO_ERROR_KINOVA;
    TrajectoryPoint kinova_joint;
    kinova_joint.InitStruct();
    memset(&kinova_joint, 0, sizeof(kinova_joint));  // zero structure

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

    kinova_joint.Position.Delay = 0.0;
    kinova_joint.Position.Type = ANGULAR_POSITION;
    kinova_joint.Position.Actuators = angles;

    result = kinova_api_.sendAdvanceTrajectory(kinova_joint);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send advanced joint angle trajectory", result);
    }

}


/**
 * @brief obtain the current angular velocities of all the joints.
 * @param vels joint velocity in degrees/second
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


/**
 * @brief joint velocity Control
 *
 * set TrajectoryPoint type as ANGULAR_VELOCITY, and send it with sendAdvanceTrajectory().
 *
 * @param joint_vel joint velocity in degree/second
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


/**
 * @brief KinovaComm::getJointAccelerations get joint acceleration
 * @param joint_acc unit in degree per second^2
 */
void KinovaComm::getJointAccelerations(AngularAcceleration &joint_acc)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&joint_acc, 0, sizeof(joint_acc));
    int result = kinova_api_.getAngularAcceleration(joint_acc);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get joint acceleration.", result);
    }
}


/**
 * @brief obtain the current torque of all the joints.
 * @param tqs joint torques in Nm
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


/**
 * @brief KinovaComm::getJointCurrent
 * @param anguler_current current in joints, unit in A.
 */
void KinovaComm::getJointCurrent(AngularPosition &anguler_current)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&anguler_current, 0, sizeof(anguler_current));
    int result = kinova_api_.getAngularCurrent(anguler_current);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the current in joints", result);
    }
}


/**
 * @brief KinovaComm::setJointTorqueMinMax
 * @param min 6 actuators in Nm
 * @param max 6 actuators in Nm
 */
void KinovaComm::setJointTorqueMinMax(AngularInfo &min, AngularInfo &max)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&min, 0, sizeof(min));
    memset(&max, 0, sizeof(max));
    int result = kinova_api_.setAngularTorqueMinMax(min, max);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set the limits for joint torques", result);
    }
}


/**
 * @brief KinovaComm::printAngles Dumps the current joint angles onto the screen.
 * @param angles input in degrees
 */
void KinovaComm::printAngles(const KinovaAngles &angles)
{
    ROS_INFO("Joint angles (deg) -- J1: %f, J2: %f J3: %f, J4: %f, J5: %f, J6: %f \n",
             angles.Actuator1, angles.Actuator2, angles.Actuator3,
             angles.Actuator4, angles.Actuator5, angles.Actuator6);

    ROS_INFO("Joint angles (rad) -- J1: %f, J2: %f J3: %f, J4: %f, J5: %f, J6: %f",
             angles.Actuator1/180.0*M_PI, angles.Actuator2/180.0*M_PI, angles.Actuator3/180.0*M_PI,
             angles.Actuator4/180.0*M_PI, angles.Actuator5/180.0*M_PI, angles.Actuator6/180.0*M_PI);
}




/**
 * @brief KinovaComm::setCartesianControl
 * If robot is not in motion, change control model to Angular control
 */
void KinovaComm::setCartesianControl()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    TrajectoryFIFO trajectory_fifo;
    memset(&trajectory_fifo, 0, sizeof(trajectory_fifo));
    getGlobalTrajectoryInfo(trajectory_fifo);
    if(trajectory_fifo.TrajectoryCount > 0)
    {
        ROS_WARN("Current tranjectory count is %d, Please wait the trajectory to finish to swich to Cartesian control.", trajectory_fifo.TrajectoryCount);
        return;
    }
    int result = kinova_api_.setCartesianControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set Cartesian control", result);
    }
}


/**
 * @brief KinovaComm::getCartesianCommand
 * @param cartesian_command {CartesianInfo, FingersPosition}
 */
void KinovaComm::getCartesianCommand(CartesianPosition &cartesian_command)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&cartesian_command, 0, sizeof(cartesian_command));
    int result = kinova_api_.getCartesianCommand(cartesian_command);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the Cartesian command", result);
    }
}


/**
 * @brief obtain the current cartesian pose of the arm.
 *
 * In KinovaPose, orientation is expressed in Euler-ZYX convention, so that tf::Matrix3x3 EulerYPR = Rz(tz)*Ry(ty)*Rx(tx)
 *
 * @param position pose in [X,Y,Z,ThetaX,ThetaY,ThetaZ]
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

    ROS_INFO_STREAM_ONCE("Cartesian pose in [X,Y,Z, ThetaX, ThetaY, ThetaZ] is : " << kinova_cartesian_position.Coordinates.X << ", "
                    << kinova_cartesian_position.Coordinates.Y << ", "
                    << kinova_cartesian_position.Coordinates.Z << ", "
                    << kinova_cartesian_position.Coordinates.ThetaX << ", "
                    << kinova_cartesian_position.Coordinates.ThetaY << ", "
                    << kinova_cartesian_position.Coordinates.ThetaZ << std::endl);

    position = KinovaPose(kinova_cartesian_position.Coordinates);
}


/**
* @brief Sends a cartesian coordinate trajectory to the Kinova arm.
*
* Waits until the arm has stopped moving before releasing control of the API. sendBasicTrajectory() is called in api to complete the motion.
*
* @param pose target pose of robot [X,Y,Z, ThetaX, ThetaY, ThetaZ], unit in meter and radians.
* @param timeout default 0.0, not used.
* @param push default true, errase all trajectory after motion.
*/
void KinovaComm::setCartesianPosition(const KinovaPose &pose, int timeout, bool push)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_WARN_STREAM("In class [" << typeid(*this).name() << "], function ["<< __FUNCTION__ << "]: The pose could not be set because the arm is stopped" << std::endl);
        return;
    }

    int result = NO_ERROR_KINOVA;
    TrajectoryPoint kinova_pose;
    kinova_pose.InitStruct();
    memset(&kinova_pose, 0, sizeof(kinova_pose));  // zero structure

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

    kinova_pose.Position.Delay = 0.0;
    kinova_pose.Position.Type = CARTESIAN_POSITION;
//    kinova_pose.Position.HandMode = HAND_NOMOVEMENT;
    kinova_pose.Position.CartesianPosition = pose;

    result = kinova_api_.sendBasicTrajectory(kinova_pose);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send basic trajectory", result);
    }
}


/**
 * @brief Linear and angular velocity control in Cartesian space
 * Definition of angular velocity "Omega" is based on the skew-symmetric matrices "S = R*R^(-1)", where "R" is the rotation matrix. angular velocity vector "Omega = [S(3,2); S(1,3); S(2,1)]".
 * @param velocities unit are meter/second for linear velocity and radians/second for "Omega".
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


/**
 * @brief KinovaComm::getMaxTranslationVelocity
 *
 * max translation(X, Y and Z) velocity of the robot's end effector in ClientConfigurations
 *
 * @return MaxTranslationVelocity in meter per second
 */
float KinovaComm::getMaxTranslationVelocity()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    ClientConfigurations configuration;
    getConfig(configuration);
    return configuration.MaxTranslationVelocity;
}


/**
 * @brief KinovaComm::setMaxTranslationVelocity
 *
 * max translation(X, Y and Z) velocity of the robot's end effector in ClientConfigurations
 *
 * @param max_trans_vel in meter per second
 */
void KinovaComm::setMaxTranslationVelocity(const float &max_trans_vel)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    ClientConfigurations configuration;
    getConfig(configuration);
    usleep(100000);
    configuration.MaxTranslationVelocity = max_trans_vel;
    setConfig(configuration);
}


/**
 * @brief KinovaComm::getMaxOrientationVelocity
 * max orientation(ThetaX, ThetaY and ThetaZ) velocity of the robot's end effector.
 * @return in
 */
float KinovaComm::getMaxOrientationVelocity()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    ClientConfigurations configuration;
    getConfig(configuration);
    return configuration.MaxOrientationVelocity;
}


/**
 * @brief KinovaComm::setMaxOrientationVelocity
 * max orientation(ThetaX, ThetaY and ThetaZ) velocity of the robot's end effector.
 * @param max_orient_vel
 */
void KinovaComm::setMaxOrientationVelocity(const float &max_orient_vel)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    ClientConfigurations configuration;
    getConfig(configuration);
    usleep(100000);
    configuration.MaxOrientationVelocity = max_orient_vel;
    setConfig(configuration);
}


/**
 * @brief obtain the current cartesian force of the arm.
 * @param cart_force wrench in end-effector frame
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


/**
 * @brief Set the cartesian min and max force parameters for force control.
 * @param min in Newton
 * @param max in Newton
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


/**
 * @brief Set the cartesian inertia and damping parameters for force control.
 * Do not use it unless you know well with force control.
 * @param inertia
 * @param damping
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


/**
 * @brief KinovaComm::printPosition Dumps the current cartesian pose onto the screen.
 * @param position in [X,Y,Z,ThetaX,ThetaY,ThetaZ], where orientation is using Euler-ZYX convention.
 */
void KinovaComm::printPosition(const KinovaPose &position)
{
    ROS_INFO("Arm position\n"
             "\tposition (m) -- x: %f, y: %f z: %f\n"
             "\trotation (rad) -- theta_x: %f, theta_y: %f, theta_z: %f",
             position.X, position.Y, position.Z,
             position.ThetaX, position.ThetaY, position.ThetaZ);
}


/**
 * @brief KinovaComm::getUserCommand
 * get UserPosition from trajectory.
 * @param user_position contains POSITION_TYPE(joint or cartesian position/velocity, etc), CartesianInfo and AngularInfo, finger positions etc.
 */
void KinovaComm::getUserCommand(UserPosition &user_position)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&user_position, 0, sizeof(user_position));
    TrajectoryPoint trajecory_point;
    int result = kinova_api_.getActualTrajectoryInfo(trajecory_point);
    if(result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get trajecory information", result);
    }
    user_position = trajecory_point.Position;
}


/**
 * @brief KinovaComm::getGlobalTrajectoryInfo
 * provide information of number of trajecoty point which are still stored in robot. Detail of trajectory point is not stored in trajectoryFIFO.
 * @param trajectoryFIFO {TrajectoryCount; UsedPercentage; MaxSize} informations regarding the robot's trajectory's FIFO.
 */
void KinovaComm::getGlobalTrajectoryInfo(TrajectoryFIFO &trajectoryFIFO)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&trajectoryFIFO, 0, sizeof(trajectoryFIFO));
    int result = kinova_api_.getGlobalTrajectoryInfo(trajectoryFIFO);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get trjectoryFIFO.", result);
    }
}


/**
 * @brief KinovaComm::eraseAllTrajectories
 * All trajectory will be cleared including angular, cartesian and fingers.
 */
void KinovaComm::eraseAllTrajectories()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = kinova_api_.eraseAllTrajectories();
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not errase all trajectories.", result);
    }
}




/**
 * @brief KinovaComm::numFingers get number of fingers.
 *
 * number of fingers determined by robotType. 3 fingers for robotType(0,3,4,6) and 2 fingers for robotType(1,2,5)
 *
 * @return returns number of fingers.
 */
int KinovaComm::numFingers() const
{
    return num_fingers_;
}


/**
 * @brief obtain the current finger positions.
 * @param fingers fingers in degrees, range from 0 to 6800
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


/**
 * @brief Sets the finger positions
 * @param fingers in degrees from 0 to about 6800
 * @param timeout timeout default 0.0, not used.
 * @param push default true, errase all trajectory after
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
    TrajectoryPoint kinova_pose;
    kinova_pose.InitStruct();
    memset(&kinova_pose, 0, sizeof(kinova_pose));  // zero structure

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
    kinova_pose.Position.HandMode = POSITION_MODE;
    kinova_pose.Position.Type = ANGULAR_POSITION;
    kinova_pose.Position.Fingers = fingers;
    kinova_pose.Position.Delay = 0.0;
    kinova_pose.LimitationsActive = 0;

    AngularPosition kinova_angles;
    memset(&kinova_angles, 0, sizeof(kinova_angles));  // zero structure

    result = kinova_api_.getAngularPosition(kinova_angles);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get the angular position", result);
    }


    kinova_pose.Position.Actuators = kinova_angles.Actuators;

    // When loading a cartesian position for the fingers, values are required for the arm joints
    // as well or the arm goes nuts.  Grab the current position and feed it back to the arm.
    KinovaPose pose;
    getCartesianPosition(pose);
    kinova_pose.Position.CartesianPosition = pose;

    result = kinova_api_.sendAdvanceTrajectory(kinova_pose);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not send advanced finger trajectory", result);
    }
}


/**
 * @brief KinovaComm::printFingers Dumps the current finger agnles onto the screen.
 * @param fingers
 */
void KinovaComm::printFingers(const FingersPosition &fingers)
{
    ROS_INFO("Finger positions -- F1: %f, F2: %f, F3: %f",
             fingers.Finger1, fingers.Finger2, fingers.Finger3);
}


/**
 * @brief Send the arm to the "home" position.
 *
 * The code replicates the function of the "home" button on the user
 *  controller by "pressing" the home button long enough for the arm
 *  to return to the home position.
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


/**
 * @brief Determines whether the arm has returned to its "Home" state.
 *
 * Checks the current joint angles, then compares them to the known
 *  "Home" joint angles.
 *
 * @return true is robot is already in predefined "Home"
 *  configuration.
 */
bool KinovaComm::isHomed(void)
{
    QuickStatus quick_status;
    getQuickStatus(quick_status);

    if (quick_status.RetractType == RETRACT_TYPE_READY_STANDBY)
    {
        return true;
    }
    else
    {
        return false;
    }
}


/**
 * @brief Initialize finger actuators.
 *
 * Move fingers to the full-open position to initialize them for use.
 * Note, The this routine requires firmware version 5.05.x (or higher).
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


/**
 * @brief KinovaComm::setEndEffectorOffset Set the end effector offset.
 * @param x in meter
 * @param y in meter
 * @param z in meter
 */
void KinovaComm::setEndEffectorOffset(unsigned int status, float x, float y, float z)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    int result = kinova_api_.setEndEffectorOffset(status, x, y, z);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not set end effector offset.", result);
    }
}


/**
 * @brief KinovaComm::getEndEffectorOffset
 * @param x in meter
 * @param y in meter
 * @param z in meter
 */
void KinovaComm::getEndEffectorOffset(unsigned int &status, float &x, float &y, float &z)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&status, 0, sizeof(status));
    memset(&x, 0, sizeof(x));
    memset(&y, 0, sizeof(y));
    memset(&z, 0, sizeof(z));
    int result = kinova_api_.getEndEffectorOffset(status, x, y, z);
    if (result != NO_ERROR_KINOVA)
    {
        throw KinovaCommException("Could not get current end effector offset.", result);
    }
}


}  // namespace kinova
