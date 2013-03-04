/*
 * jacolib.cpp
 *
 *  Created on: Feb 16, 2013
 *      Author: mdedonato
 */
#include "jaco_arm/jacolib.h"

#define JACO_USB_LIBRARY 			"Kinova.API.USBCommandLayerUbuntu.so"

int (*Jaco_InitAPI)();

int (*Jaco_CloseAPI)();

int (*Jaco_GetCodeVersion)(std::vector<int> &);

int (*Jaco_GetCartesianPosition)(CartesianPosition &);

int (*Jaco_GetAngularPosition)(AngularPosition &);

int (*Jaco_GetCartesianForce)(CartesianPosition &);

int (*Jaco_GetAngularForce)(AngularPosition &);

int (*Jaco_GetAngularCurrent)(AngularPosition &);

int (*Jaco_GetControlOperator)(int &);

int (*Jaco_GetActualTrajectoryInfo)(TrajectoryPoint &);

int (*Jaco_GetGlobalTrajectoryInfo)(TrajectoryFIFO &);

int (*Jaco_GetSensorsInfo)(SensorsInfo &);

int (*Jaco_GetSingularityVector)(SingularityVector &);

int (*Jaco_SetAngularControl)();

int (*Jaco_SetCartesianControl)();

int (*Jaco_StartControlAPI)();

int (*Jaco_StopControlAPI)();

int (*Jaco_RestoreFactoryDefault)();

int (*Jaco_SendJoystickCommand)(JoystickCommand);

int (*Jaco_SendAdvanceTrajectory)(TrajectoryPoint);

int (*Jaco_SendBasicTrajectory)(TrajectoryPoint);

int (*Jaco_GetClientConfigurations)(ClientConfigurations &);

int (*Jaco_SetClientConfigurations)(ClientConfigurations);

int (*Jaco_EraseAllTrajectories)();

int (*Jaco_GetPositionCurrentActuators)(std::vector<float> &);

int (*Jaco_SetActuatorPID)(unsigned int, float, float, float);


using namespace std;

void Jacolib_Init(void) {

	void *USBLib = dlopen(JACO_USB_LIBRARY, RTLD_NOW | RTLD_GLOBAL);
	if (USBLib == NULL) {
		cout << dlerror() << endl;
	}

	Jaco_InitAPI = (int (*)())dlsym(USBLib, "InitAPI");

Jaco_CloseAPI	= (int (*)())dlsym(USBLib, "CloseAPI");

Jaco_GetCodeVersion	= (int (*)(std::vector<int> &))dlsym(USBLib, "GetCodeVersion");

Jaco_GetCartesianPosition	= (int (*)(CartesianPosition &))dlsym(USBLib, "GetCartesianPosition");

Jaco_GetAngularPosition	= (int (*)(AngularPosition &))dlsym(USBLib, "GetAngularPosition");

Jaco_GetCartesianForce	= (int (*)(CartesianPosition &))dlsym(USBLib, "GetCartesianForce");

Jaco_GetAngularForce	= (int (*)(AngularPosition &))dlsym(USBLib, "GetAngularForce");

Jaco_GetAngularCurrent	= (int (*)(AngularPosition &))dlsym(USBLib, "GetAngularCurrent");

Jaco_GetControlOperator	= (int (*)(int &))dlsym(USBLib, "GetControlOperator");

Jaco_GetActualTrajectoryInfo	= (int (*)(TrajectoryPoint &))dlsym(USBLib, "GetActualTrajectoryInfo");

Jaco_GetGlobalTrajectoryInfo	= (int (*)(TrajectoryFIFO &))dlsym(USBLib, "GetGlobalTrajectoryInfo");

Jaco_GetSensorsInfo	= (int (*)(SensorsInfo &))dlsym(USBLib, "GetSensorsInfo");

Jaco_GetSingularityVector	= (int (*)(SingularityVector &))dlsym(USBLib, "GetSingularityVector");

Jaco_SetAngularControl	= (int (*)())dlsym(USBLib, "SetAngularControl");

Jaco_SetCartesianControl	= (int (*)())dlsym(USBLib, "SetCartesianControl");

Jaco_StartControlAPI	= (int (*)())dlsym(USBLib, "StartControlAPI");

Jaco_StopControlAPI	= (int (*)())dlsym(USBLib, "StopControlAPI");

Jaco_RestoreFactoryDefault	= (int (*)())dlsym(USBLib, "RestoreFactoryDefault");

Jaco_SendJoystickCommand	= (int (*)(JoystickCommand))dlsym(USBLib, "SendJoystickCommand");

Jaco_SendAdvanceTrajectory	= (int (*)(TrajectoryPoint))dlsym(USBLib, "SendAdvanceTrajectory");

Jaco_SendBasicTrajectory	= (int (*)(TrajectoryPoint))dlsym(USBLib, "SendBasicTrajectory");

Jaco_GetClientConfigurations	= (int (*)(ClientConfigurations &))dlsym(USBLib, "GetClientConfigurations");

Jaco_SetClientConfigurations	= (int (*)(ClientConfigurations))dlsym(USBLib, "SetClientConfigurations");

Jaco_EraseAllTrajectories	= (int (*)())dlsym(USBLib, "EraseAllTrajectories");

Jaco_GetPositionCurrentActuators	= (int (*)(
			std::vector<float> &))dlsym(USBLib, "GetPositionCurrentActuators");

Jaco_SetActuatorPID	= (int (*)(unsigned int, float, float,
			float))dlsym(USBLib, "SetActuatorPID");



}
