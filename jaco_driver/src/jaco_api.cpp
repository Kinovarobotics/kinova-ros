/*
 * jacolib.cpp
 *
 *  Created on: Mar 10, 2013
 *  Modified on: June 25, 2013
 *      Author: mdedonato, Clearpath Robotics
 */

#include <jaco_driver/jaco_api.h>

namespace jaco {

JacoAPI::JacoAPI(void) {

	void *USBLib = dlopen(JACO_USB_LIBRARY, RTLD_NOW | RTLD_GLOBAL);
	if (USBLib == NULL) {
		ROS_WARN("%s", dlerror());
	}

	InitAPI = (int (*)())dlsym(USBLib, "InitAPI");

	CloseAPI = (int (*)())dlsym(USBLib, "CloseAPI");

	GetCodeVersion = (int (*)(
			std::vector<int> &))dlsym(USBLib, "GetCodeVersion");

	GetCartesianPosition = (int (*)(
			CartesianPosition &))dlsym(USBLib, "GetCartesianPosition");

	GetAngularPosition = (int (*)(
			AngularPosition &))dlsym(USBLib, "GetAngularPosition");

	GetCartesianForce = (int (*)(
			CartesianPosition &))dlsym(USBLib, "GetCartesianForce");

	GetAngularForce = (int (*)(
			AngularPosition &))dlsym(USBLib, "GetAngularForce");

	GetAngularCurrent = (int (*)(
			AngularPosition &))dlsym(USBLib, "GetAngularCurrent");

	GetControlOperator = (int (*)(int &))dlsym(USBLib, "GetControlOperator");

	GetActualTrajectoryInfo = (int (*)(
			TrajectoryPoint &))dlsym(USBLib, "GetActualTrajectoryInfo");

	GetGlobalTrajectoryInfo = (int (*)(
			TrajectoryFIFO &))dlsym(USBLib, "GetGlobalTrajectoryInfo");

	GetSensorsInfo = (int (*)(SensorsInfo &))dlsym(USBLib, "GetSensorsInfo");

	GetSingularityVector = (int (*)(
			SingularityVector &))dlsym(USBLib, "GetSingularityVector");

	SetAngularControl = (int (*)())dlsym(USBLib, "SetAngularControl");

	SetCartesianControl = (int (*)())dlsym(USBLib, "SetCartesianControl");

	StartControlAPI = (int (*)())dlsym(USBLib, "StartControlAPI");

	StopControlAPI = (int (*)())dlsym(USBLib, "StopControlAPI");

	MoveHome = (int (*)())dlsym(USBLib, "MoveHome");

	InitFingers = (int (*)())dlsym(USBLib, "InitFingers");

	RestoreFactoryDefault = (int (*)())dlsym(USBLib, "RestoreFactoryDefault");

	SendJoystickCommand = (int (*)(
			JoystickCommand))dlsym(USBLib, "SendJoystickCommand");

	SendAdvanceTrajectory = (int (*)(
			TrajectoryPoint))dlsym(USBLib, "SendAdvanceTrajectory");

	SendBasicTrajectory = (int (*)(
			TrajectoryPoint))dlsym(USBLib, "SendBasicTrajectory");

	GetQuickStatus = (int (*)(
			QuickStatus &))dlsym(USBLib, "GetQuickStatus");

	GetClientConfigurations = (int (*)(
			ClientConfigurations &))dlsym(USBLib, "GetClientConfigurations");

	SetClientConfigurations = (int (*)(
			ClientConfigurations))dlsym(USBLib, "SetClientConfigurations");

	EraseAllTrajectories = (int (*)())dlsym(USBLib, "EraseAllTrajectories");

	GetPositionCurrentActuators = (int (*)(
			std::vector<float> &))dlsym(USBLib, "GetPositionCurrentActuators");

	SetActuatorPID = (int (*)(unsigned int, float, float,
			float))dlsym(USBLib, "SetActuatorPID");
}

}
