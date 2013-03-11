/*
 * jacolib.h
 *
 *  Created on: Feb 16, 2013
 *      Author: mdedonato
 */

#ifndef JACOLIB_H_
#define JACOLIB_H_

#include "jaco_driver/Kinova.API.UsbCommandLayerUbuntu.h"
#include "jaco_driver/KinovaTypes.h"
#include <dlfcn.h>
#include <iostream>
#include <ros/ros.h>

namespace jaco {
#define JACO_USB_LIBRARY 			"Kinova.API.USBCommandLayerUbuntu.so"

class JacoAPI {

public:
	JacoAPI(void);
	int (*InitAPI)(void);
	int (*CloseAPI)(void);
	int (*GetCodeVersion)(std::vector<int> &);
	int (*GetCartesianPosition)(CartesianPosition &);
	int (*GetAngularPosition)(AngularPosition &);
	int (*GetCartesianForce)(CartesianPosition &);
	int (*GetAngularForce)(AngularPosition &);
	int (*GetAngularCurrent)(AngularPosition &);
	int (*GetControlOperator)(int &);
	int (*GetActualTrajectoryInfo)(TrajectoryPoint &);
	int (*GetGlobalTrajectoryInfo)(TrajectoryFIFO &);
	int (*GetSensorsInfo)(SensorsInfo &);
	int (*GetSingularityVector)(SingularityVector &);
	int (*SetAngularControl)();
	int (*SetCartesianControl)();
	int (*StartControlAPI)();
	int (*StopControlAPI)();
	int (*RestoreFactoryDefault)();
	int (*SendJoystickCommand)(JoystickCommand);
	int (*SendAdvanceTrajectory)(TrajectoryPoint);
	int (*SendBasicTrajectory)(TrajectoryPoint);
	int (*GetClientConfigurations)(ClientConfigurations &);
	int (*SetClientConfigurations)(ClientConfigurations);
	int (*EraseAllTrajectories)();
	int (*GetPositionCurrentActuators)(std::vector<float> &);
	int (*SetActuatorPID)(unsigned int, float, float, float);

	};}

#endif /* JACOLIB_H_ */
