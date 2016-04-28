/*
 * jacolib.h
 *
 *  Created on: Feb 16, 2013
 *  Modified on: June 25, 2013
 *      Author: mdedonato, Clearpath Robotics
 */

#ifndef JACO_DRIVER_JACO_API_H
#define JACO_DRIVER_JACO_API_H

#include <dlfcn.h>
#include <ros/ros.h>

#include <iostream>
#include <vector>

#include "kinova/Kinova.API.UsbCommandLayerUbuntu.h"
#include "kinova/KinovaTypes.h"


namespace jaco
{

#define JACO_USB_LIBRARY "Kinova.API.USBCommandLayerUbuntu.so"

class JacoAPI
{
 public:
    JacoAPI(const std::string & lib = JACO_USB_LIBRARY);

    int (*initAPI)(void);
    int (*closeAPI)(void);
    int (*getAPIVersion)(std::vector<int> &);
    int (*getDevices)(std::vector<KinovaDevice> &, int &);
    int (*setActiveDevice)(KinovaDevice);

    int (*getGeneralInformations)(GeneralInformations &);
    int (*getQuickStatus)(QuickStatus &);
    // int (*GetForcesInfo)(ForcesInfo &);

    int (*getCodeVersion)(std::vector<int> &);
    int (*startControlAPI)();
    int (*stopControlAPI)();
    int (*initFingers)();

    int (*moveHome)();

    int (*getCartesianPosition)(CartesianPosition &);
    int (*getAngularPosition)(AngularPosition &);
    int (*getAngularVelocity)(AngularPosition &);
    int (*getCartesianForce)(CartesianPosition &);
    int (*setCartesianForceMinMax)(CartesianInfo, CartesianInfo);
    int (*setCartesianInertiaDamping)(CartesianInfo, CartesianInfo);
    int (*startForceControl)();
    int (*stopForceControl)();
    int (*getAngularForce)(AngularPosition &);
    int (*getAngularCurrent)(AngularPosition &);
    int (*getControlType)(int &);
    int (*getActualTrajectoryInfo)(TrajectoryPoint &);
    int (*getGlobalTrajectoryInfo)(TrajectoryFIFO &);
    int (*getSensorsInfo)(SensorsInfo &);
    int (*setAngularControl)();
    int (*setCartesianControl)();
    int (*restoreFactoryDefault)();
    int (*sendJoystickCommand)(JoystickCommand);
    int (*sendAdvanceTrajectory)(TrajectoryPoint);
    int (*sendBasicTrajectory)(TrajectoryPoint);
    int (*getClientConfigurations)(ClientConfigurations &);
    int (*setClientConfigurations)(ClientConfigurations);
    int (*eraseAllTrajectories)();
    int (*getPositionCurrentActuators)(std::vector<float> &);
    int (*setActuatorPID)(unsigned int, float, float, float);
};

}  // namespace jaco
#endif  // JACO_DRIVER_JACO_API_H
