#ifdef KINOVAAPIUSBCOMMANDLAYER_EXPORTS
#define KINOVAAPIUSBCOMMANDLAYER_API __attribute__ ((visibility ("default")))
#else
#define KINOVAAPIUSBCOMMANDLAYER_API __attribute__ ((visibility ("default")))
#endif

/**
 * @file Kinova.API.UsbCommandLayerUbuntu.h
 * @brief This file contains header of all available functions of this API.
 */

#include <vector>
#include "KinovaTypes.h"
#include "Kinova.API.EthCommLayerUbuntu.h"
#include <stdio.h>

//This defines the the location of the communication layer.(Kinova.API.CommLayerUbuntu.so)
//#define COMM_LAYER_PATH "./CommLayerUbuntu.so"
//#define COMM_LAYER_ETHERNET_PATH "./EthCommLayerUbuntu.so"

// ***** E R R O R   C O D E S ******
#define ERROR_INIT_API 2001      // Error while initializing the API
#define ERROR_LOAD_COMM_DLL 2002 // Error while loading the communication layer

//Those 3 codes are mostly for internal use
#define JACO_NACK_FIRST 2003
#define JACO_COMM_FAILED 2004
#define JACO_NACK_NORMAL 2005

//Unable to initialize the communication layer.
#define ERROR_INIT_COMM_METHOD  2006

//Unable to load the Close() function from the communication layer.
#define ERROR_CLOSE_METHOD  2007

//Unable to load the GetDeviceCount() function from the communication layer.
#define ERROR_GET_DEVICE_COUNT_METHOD  2008

//Unable to load the SendPacket() function from the communication layer.
#define ERROR_SEND_PACKET_METHOD  2009

//Unable to load the SetActiveDevice() function from the communication layer.
#define ERROR_SET_ACTIVE_DEVICE_METHOD 2010

//Unable to load the GetDeviceList() function from the communication layer.
#define ERROR_GET_DEVICES_LIST_METHOD 2011

//Unable to initialized the system semaphore.
#define ERROR_SEMAPHORE_FAILED 2012

//Unable to load the ScanForNewDevice() function from the communication layer.
#define ERROR_SCAN_FOR_NEW_DEVICE 2013

//Unable to load the GetActiveDevice function from the communication layer.
#define ERROR_GET_ACTIVE_DEVICE_METHOD 2014

//Unable to load the OpenRS485_Activate() function from the communication layer.
#define ERROR_OPEN_RS485_ACTIVATE 2015

//A function's parameter is not valid.
#define ERROR_INVALID_PARAM 2100

//The API is not initialized.
#define ERROR_API_NOT_INITIALIZED 2101

//Unable to load the InitDataStructure() function from the communication layer.
#define ERROR_INIT_DATA_STRUCTURES_METHOD 2102

// ***** E N D  O F  E R R O R   C O D E S ******


//This represents the size of an array containing Cartesian values.
#define CARTESIAN_SIZE 6

//This represents the max actuator count in our context.
#define MAX_ACTUATORS 7

//This represents the max actuator count in our context.
#define MAX_INVENTORY 15

//This represents the size of the array returned by the function GetCodeVersion.
#define CODE_VERSION_COUNT 42

//This represents the size of the array returned by the function GetAPIVersion.
#define API_VERSION_COUNT 3

//This represents the size of the array returned by the function GetPositionCurrentActuators.
#define POSITION_CURRENT_COUNT 12

#define POSITION_CURRENT_COUNT_7DOF 14

//This represents the size of the array returned by the function GetSpasmFilterValues and sent to SetSpasmFilterValues.
#define SPASM_FILTER_COUNT 1

//Version of the API 5.03.00
#define COMMAND_LAYER_VERSION 50300

#define COMMAND_SIZE 70

#define OPTIMAL_Z_PARAM_SIZE 16

#define OPTIMAL_Z_PARAM_SIZE_7DOF 19

#define GRAVITY_VECTOR_SIZE 3

#define GRAVITY_PARAM_SIZE 42

#define GRAVITY_PAYLOAD_SIZE 4

//This represents the size of the buffer for the IP address.
#define IP_ADDRESS_LENGTH 4
#define MAC_ADDRESS_LENGTH 6

// ***** API'S FUNCTIONAL CORE *****

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetDevices(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetActiveDevice(KinovaDevice device);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetActiveDeviceEthernet(KinovaDevice device, unsigned long ipAddress);
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_RefresDevicesList(void);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_InitAPI(void);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_InitEthernetAPI(EthernetCommConfig & config);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_CloseAPI(void);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetCodeVersion(int Response[CODE_VERSION_COUNT]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetAPIVersion(int Response[API_VERSION_COUNT]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetCartesianPosition(CartesianPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetAngularPosition(AngularPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetCartesianForce(CartesianPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetAngularForce(AngularPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetAngularCurrent(AngularPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetActualTrajectoryInfo(TrajectoryPoint &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetGlobalTrajectoryInfo(TrajectoryFIFO &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetSensorsInfo(SensorsInfo &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetSingularityVector(SingularityVector &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetAngularControl();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetCartesianControl();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_StartControlAPI();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_StopControlAPI();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_RestoreFactoryDefault();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SendJoystickCommand(JoystickCommand joystickCommand);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SendAdvanceTrajectory(TrajectoryPoint trajectory);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SendBasicTrajectory(TrajectoryPoint trajectory);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetClientConfigurations(ClientConfigurations &config);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API  int Ethernet_GetAllRobotIdentity(RobotIdentity robotIdentity[MAX_KINOVA_DEVICE], int & count);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API  int Ethernet_GetRobotIdentity(RobotIdentity &robotIdentity);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetClientConfigurations(ClientConfigurations config);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_EraseAllTrajectories();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetPositionCurrentActuators(float Response[POSITION_CURRENT_COUNT]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetActuatorPID(unsigned int address, float P, float I, float D);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetAngularCommand(AngularPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetCartesianCommand(CartesianPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetAngularCurrentMotor(AngularPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetAngularVelocity(AngularPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetControlType(int &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_StartForceControl();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_StopForceControl();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_StartRedundantJointNullSpaceMotion();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_StopRedundantJointNullSpaceMotion();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_ActivateExtraProtectionPinchingWrist(int state);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_ActivateCollisionAutomaticAvoidance(int state); //not available on Jaco, Jaco Spherical 6 DOF and Mico models.

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_ActivateSingularityAutomaticAvoidance(int state); //not available on Jaco, Jaco Spherical 6 DOF and Mico models.

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_StartCurrentLimitation();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_StopCurrentLimitation();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetSystemErrorCount(unsigned int &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetSystemError(unsigned int indexError, SystemError &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_ClearErrorLog();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_EraseAllProtectionZones();

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetSerialNumber(char Command[STRING_LENGTH], char temp[STRING_LENGTH]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetDefaultGravityParam(float Command[GRAVITY_PARAM_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetControlMapping(ControlMappingCharts &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetProtectionZone(ZoneList &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetProtectionZone(ZoneList Command);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetGripperStatus(Gripper &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetQuickStatus(QuickStatus &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetForcesInfo(ForcesInfo &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetControlMapping(ControlMappingCharts Command);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_ProgramFlash(const char * filename);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetJointZero(int ActuatorAdress);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueZero(int ActuatorAdress);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueGain(int ActuatorAdress, int Gain);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetActuatorPIDFilter(int ActuatorAdress, float filterP, float filterI, float filterD);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetActuatorAddress(int ActuatorAdress, int newAddress);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetGeneralInformations(GeneralInformations &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetFrameType(int frameType);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetCartesianForceMinMax(CartesianInfo min, CartesianInfo max);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetCartesianInertiaDamping(CartesianInfo inertia, CartesianInfo damping);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetAngularTorqueMinMax(AngularInfo min, AngularInfo max);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetAngularInertiaDamping(AngularInfo inertia, AngularInfo damping);

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetDevValue(std::vector<float> command);

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetDevValue(std::vector<float> &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetSpasmFilterValues(float Command[SPASM_FILTER_COUNT], int activationStatus);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetSpasmFilterValues(float Response[SPASM_FILTER_COUNT], int &activationStatus);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_MoveHome();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetAngularForceGravityFree(AngularPosition &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetActuatorAcceleration(AngularAcceleration &Response);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_InitFingers();

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetPeripheralInventory(PeripheralInfo list[MAX_INVENTORY] );

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetModel(char Command[STRING_LENGTH], char temp[STRING_LENGTH]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetJoystickValue(JoystickCommand &joystickCommand);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetRobotConfiguration(int ConfigID);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetCommandVelocity(float cartesianVelocity[CARTESIAN_SIZE], float angularVelocity[MAX_ACTUATORS]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetEndEffectorOffset(unsigned int &status, float &x, float &y, float &z);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetEndEffectorOffset(unsigned int status, float x, float y, float z);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SendAngularTorqueCommand(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SendCartesianForceCommand(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueActuatorGain(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueActuatorDamping(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SwitchTrajectoryTorque(GENERALCONTROL_TYPE type);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueCommandMax(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueSafetyFactor(float factor);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueGainMax(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueRateLimiter(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueFeedCurrent(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueFeedVelocity(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorquePositionLimitDampingGain(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorquePositionLimitDampingMax(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorquePositionLimitRepulsGain(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorquePositionLimitRepulsMax(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueFilterVelocity(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueFilterMeasuredTorque(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueFilterError(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueFilterControlEffort(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetGravityType(GRAVITY_TYPE type);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetGravityVector(float gravityVector[GRAVITY_VECTOR_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetGravityOptimalZParam(float Command[GRAVITY_PARAM_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetGravityManualInputParam(float Command[GRAVITY_PARAM_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetAngularTorqueCommand(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetAngularTorqueGravityEstimation(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetActuatorMaxVelocity(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetSwitchThreshold(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetPositionLimitDistance(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueControlType(TORQUECONTROL_TYPE type);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetGravityPayload(float Command[GRAVITY_PAYLOAD_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueVibrationController(float activationStatus);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueRobotProtection(int protectionLevel);

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueVelocityLimitFilter(float Command[COMMAND_SIZE]);

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueFeedFilter(float Command[COMMAND_SIZE]);

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueStaticFriction(float Command[COMMAND_SIZE]);

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueErrorDeadband(float Command[COMMAND_SIZE]);

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueBrake(float Command[COMMAND_SIZE]);

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueInactivityTimeActuator(float Command[COMMAND_SIZE]);

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueInactivityTimeMainController(int time);

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueDampingMax(float Command[COMMAND_SIZE]);

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueFeedVelocityUnderGain(float Command[COMMAND_SIZE]);

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueFeedCurrentVoltage(float Command[COMMAND_SIZE]);

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueStaticFrictionMax(float Command[COMMAND_SIZE]);

//Internal use only
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueErrorResend(float Command[COMMAND_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_RunGravityZEstimationSequence(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_RunGravityZEstimationSequence7DOF(ROBOT_TYPE type, float OptimalzParam[OPTIMAL_Z_PARAM_SIZE_7DOF]);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetTrajectoryTorqueMode(int&);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetTorqueInactivityType(int);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API  int Ethernet_GetActuatorsPosition(float *positionList);

//NEW ETHERNET EXPORTED FUNCTIONS
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetEthernetConfiguration(EthernetConfiguration * config);

extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_GetEthernetConfiguration(EthernetConfiguration * config);

//DO NOT USE only for Kinova
extern "C" KINOVAAPIUSBCOMMANDLAYER_API int Ethernet_SetLocalMACAddress(unsigned char mac[MAC_ADDRESS_LENGTH], char temp[STRING_LENGTH]);
