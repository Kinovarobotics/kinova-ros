/*
 * jacolib.h
 *
 *  Created on: Feb 16, 2013
 *  Modified on: June 25, 2013
 *      Author: mdedonato, Clearpath Robotics
 */

#ifndef JACOLIB_H_
#define JACOLIB_H_

#include "kinova/Kinova.API.UsbCommandLayerUbuntu.h"
#include "kinova/KinovaTypes.h"
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
    int (*getCartesianForce)(CartesianPosition &);
    int (*getAngularForce)(AngularPosition &);
    int (*getAngularCurrent)(AngularPosition &);
    int (*getControlType)(int &);
    // int (*getControlOperator)(int &);
    int (*getActualTrajectoryInfo)(TrajectoryPoint &);
    int (*getGlobalTrajectoryInfo)(TrajectoryFIFO &);
    int (*getSensorsInfo)(SensorsInfo &);
//    int (*getSingularityVector)(SingularityVector &);
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

std::ostream& operator<< (std::ostream& stream, const QuickStatus& quickStatus);

}  // namespace jaco

#endif  // JACOLIB_H_
