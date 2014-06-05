/*
 * jacolib.h
 *
 *  Created on: Feb 16, 2013
 *  Modified on: June 25, 2013
 *      Author: mdedonato, Clearpath Robotics
 */

#ifndef JACOLIB_H_
#define JACOLIB_H_

#include "jaco_driver/Kinova.API.UsbCommandLayerUbuntu.h"
#include "jaco_driver/KinovaTypes.h"
#include <dlfcn.h>
#include <iostream>
#include <ros/ros.h>


namespace jaco {
#define JACO_USB_LIBRARY "Kinova.API.USBCommandLayerUbuntu.so"

class JacoAPI {
 public:
    JacoAPI(void);
    int (*initAPI)(void);
    int (*closeAPI)(void);
    int (*getQuickStatus)(QuickStatus &);
    // int (*GetForcesInfo)(ForcesInfo &);
    int (*getCodeVersion)(std::vector<int> &);
    int (*getCartesianPosition)(CartesianPosition &);
    int (*getAngularPosition)(AngularPosition &);
    int (*getCartesianForce)(CartesianPosition &);
    int (*getAngularForce)(AngularPosition &);
    int (*getAngularCurrent)(AngularPosition &);
    int (*getControlOperator)(int &);
    int (*getActualTrajectoryInfo)(TrajectoryPoint &);
    int (*getGlobalTrajectoryInfo)(TrajectoryFIFO &);
    int (*getSensorsInfo)(SensorsInfo &);
    int (*getSingularityVector)(SingularityVector &);
    int (*setAngularControl)();
    int (*setCartesianControl)();
    int (*startControlAPI)();
    int (*stopControlAPI)();
    int (*moveHome)();
    int (*initFingers)();
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
}

#endif  // JACOLIB_H_
