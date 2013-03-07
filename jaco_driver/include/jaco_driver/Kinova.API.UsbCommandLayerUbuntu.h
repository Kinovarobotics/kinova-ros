#ifdef KINOVAAPIUSBCOMMANDLAYER_EXPORTS
#define KINOVAAPIUSBCOMMANDLAYER_API __declspec(dllexport)
#else
#define KINOVAAPIUSBCOMMANDLAYER_API __declspec(dllimport)
#endif

#include <vector>
#include <jaco_driver/KinovaTypes.h>

//namespace jaco_arm {
// ***** E R R O R   C O D E S ******

#define ERROR_INIT_API 2001      // Error while initializing the API
#define ERROR_LOAD_COMM_DLL 2002 // Error while loading the communication layer

//Those 3 codes are mostly for internal use
#define JACO_NACK_FIRST 2003
#define JACO_COMM_FAILED 2004
#define JACO_NACK_NORMAL 2005

#define COMMAND_LAYER_VERSION 10001

// ***** API'S FUNCTIONAL CORE *****
extern "C" __attribute__ ((visibility ("default"))) int InitAPI(void);

extern "C" __attribute__ ((visibility ("default"))) int CloseAPI(void);

extern "C" __attribute__ ((visibility ("default"))) int GetCodeVersion(std::vector<int> &Response);

extern "C" __attribute__ ((visibility ("default"))) int GetCartesianPosition(CartesianPosition &Response);

extern "C" __attribute__ ((visibility ("default"))) int GetAngularPosition(AngularPosition &Response);

extern "C" __attribute__ ((visibility ("default"))) int GetCartesianForce(CartesianPosition &Response);

extern "C" __attribute__ ((visibility ("default"))) int GetAngularForce(AngularPosition &Response);

extern "C" __attribute__ ((visibility ("default"))) int GetAngularCurrent(AngularPosition &Response);

extern "C" __attribute__ ((visibility ("default"))) int GetControlOperator(int &Response);

extern "C" __attribute__ ((visibility ("default"))) int GetActualTrajectoryInfo(TrajectoryPoint &Response);

extern "C" __attribute__ ((visibility ("default"))) int GetGlobalTrajectoryInfo(TrajectoryFIFO &Response);

extern "C" __attribute__ ((visibility ("default"))) int GetSensorsInfo(SensorsInfo &Response);

extern "C" __attribute__ ((visibility ("default"))) int GetSingularityVector(SingularityVector &Response);

extern "C" __attribute__ ((visibility ("default"))) int SetAngularControl();

extern "C" __attribute__ ((visibility ("default"))) int SetCartesianControl();

extern "C" __attribute__ ((visibility ("default"))) int StartControlAPI();

extern "C" __attribute__ ((visibility ("default"))) int StopControlAPI();

extern "C" __attribute__ ((visibility ("default"))) int RestoreFactoryDefault();

extern "C" __attribute__ ((visibility ("default"))) int SendJoystickCommand(JoystickCommand joystickCommand);

extern "C" __attribute__ ((visibility ("default"))) int SendAdvanceTrajectory(TrajectoryPoint trajectory);

extern "C" __attribute__ ((visibility ("default"))) int SendBasicTrajectory(TrajectoryPoint trajectory);

extern "C" __attribute__ ((visibility ("default"))) int GetClientConfigurations(ClientConfigurations &config);

extern "C" __attribute__ ((visibility ("default"))) int SetClientConfigurations(ClientConfigurations config);

extern "C" __attribute__ ((visibility ("default"))) int EraseAllTrajectories();

extern "C" __attribute__ ((visibility ("default"))) int GetPositionCurrentActuators(std::vector<float> &Response);

extern "C" __attribute__ ((visibility ("default"))) int SetActuatorPID(unsigned int address, float P, float I, float D);
//}
