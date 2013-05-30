#ifdef KINOVAAPIUSBCOMMANDLAYER_EXPORTS
#define KINOVAAPIUSBCOMMANDLAYER_API __declspec(dllexport)
#else
#define KINOVAAPIUSBCOMMANDLAYER_API __declspec(dllimport)
#endif

#include <vector>
#include "KinovaTypes.h"
#include <windows.h>

#define WIN32_LEAN_AND_MEAN

#define ERROR_INIT_API 2001
#define ERROR_LOAD_COMM_DLL 2002
#define JACO_NACK_FIRST 2003
#define JACO_COMM_FAILED 2004
#define JACO_NACK_NORMAL 2005

#define COMMAND_LAYER_VERSION 10001

/// <summary>
/// Get the number of connected arms.
/// </summary>
/// <param name="A DWORD">The number connected arms</param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetJacoCount(DWORD &);

/// <summary>
/// Init the API, must be called before any API session.
/// </summary>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD InitAPI(void);

/// <summary>
/// Stop the API, must be called at the end of an API session.
/// </summary>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD CloseAPI(void);

/// <summary>
/// Get the code version.
/// </summary>
/// <param name="Response">A vector to be filled with all version numbers</param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetCodeVersion(std::vector<int> &Response);

/// <summary>
/// Get absolute cartesian position (m, rad).
/// </summary>
/// <param name="Response">A CartesianPosition struture to be filled with absolute position in meters and rad </param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetCartesianPosition(CartesianPosition &Response);

/// <summary>
/// Get absolute angular position (degrees).
/// </summary>
/// <param name="Response">A AngularPosition struture to be filled with absolute angular positions in degrees </param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetAngularPosition(AngularPosition &Response);

/// <summary>
/// Get cartesian force (N).
/// </summary>
/// <param name="Response">A CartesianPosition struture to be filled with cartesian forces in Newton</param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetCartesianForce(CartesianPosition &Response);


/// <summary>
/// Get actuators applied force (Nm).
/// </summary>
/// <param name="Response">A AngularPosition structure to be filled with applied force in Newton-meters </param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetAngularForce(AngularPosition &Response);

/// <summary>
/// Get actuator current consumption (A).
/// </summary>
/// <param name="Response">A AngularPosition structure to be filled with electrical currents in Ampers </param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetAngularCurrent(AngularPosition &Response);

/// <summary>
/// Get info about actual ongoing trajectory.
/// </summary>
/// <param name="Response">A TrajectoryPoint struture to be filled </param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetActualTrajectoryInfo(TrajectoryPoint &Response);

/// <summary>
/// Get information about trajectories FIFO.
/// </summary>
/// <param name="Response">A TrajectoryFIFO structure to be filled containing information about the FIFO</param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetGlobalTrajectoryInfo(TrajectoryFIFO &Response);

/// <summary>
/// Get sensors values (voltage(V), current(A), [X,Y,Z] accelerations(m/s^2 , Actuators temperature(deg. C) and finger temperature(deg. C) ).
/// </summary>
/// <param name="Response">A SensorsInfo structure to be filled</param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetSensorsInfo(SensorsInfo &Response);

/// <summary>
/// <br>Get information about singularities from the actual hand position.
/// <br>Singularities concern the hand translation and it's rotation (changement or orientation)
/// </summary>
/// <param name="Response">A SingularityVector structure to be filled</param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetSingularityVector(SingularityVector &Response);

/// <summary>
/// Set the joystick (virtual or real one) control to angular mode (moves actuators one by one). 
/// </summary>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD SetAngularControl();

/// <summary>
/// Set the joystick (virtual or real one) control to cartesian (X,Y,Z). 
/// </summary>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD SetCartesianControl();

/// <summary>
/// Allow the API to control the arm. This function must be called before any control over the arm using API (Ex.: SendBasicTrajectory() )
/// If the joystick is used while API control, it's desactivated and need to call the function again to recover control.
/// This method has only effect on the arm movements, other kind of methods work without calling a StartControlAPI() before.
/// </summary>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD StartControlAPI();

/// <summary>
/// Disable API control over the arm.
/// Control functions has no more effects on the arm (Ex.: SendBasicTrajectory() ).
/// </summary>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD StopControlAPI();

/// <summary>
/// Restore the arm ClientConfigurations to its factory default settings (Clear all current settings).
/// </summary>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD RestoreFactoryDefault();

/// <summary>
/// Send a virtual joystick command (This has the same behavior than the hard-wired joystick).
/// The control has to be refresh in a control loop around each 10ms.
/// </summary>
/// <param name="JoystickCommand">A JoystickCommand filled structure with desired joystick controller values</param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD SendJoystickCommand(JoystickCommand joystickCommand);

/// <summary>
/// Send all trajectory infos to the arm.
/// The trajectory is push into the FIFO.
/// If it's a velocity trajectory, refresh the command each 10ms to keep movement.
/// </summary>
/// <param name="trajectory">A TrajectoryPoint structure to send </param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD SendAdvanceTrajectory(TrajectoryPoint trajectory);

/// <summary>
/// Send only essential TrajectoryPoint info (Positions for moving) to the arm (faster than SendAdvanceTrajectory)
/// The trajectory is push into the FIFO.
/// If it's a velocity trajectory, refresh the command each 10ms to keep movement.
/// </summary>
/// <param name="trajectory">A TrajectoryPoint structure to send </param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD SendBasicTrajectory(TrajectoryPoint trajectory);

/// <summary>
/// Get a ClientConfigurations from the arm.
/// </summary>
/// <param name="config">A ClientConfigurations structure to be filled</param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetClientConfigurations(ClientConfigurations &config);

/// <summary>
/// Set arm client configuration
/// </summary>
/// <param name="config">A defined ClientConfigurations structure to send and set</param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD SetClientConfigurations(ClientConfigurations config);

/// <summary>
/// Erase all trajectories in the FIFO (The arm stops immediatly)
/// </summary>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD EraseAllTrajectories();

/// <summary>
/// Get current position of actuators (degrees)
/// </summary>
/// <param name="Response">A vector to be filled containing position of all activated actuators in degrees</param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD GetPositionCurrentActuators(std::vector<float> &Response);

/// <summary>
/// Set an actuator PID
/// </summary>
/// <param name="address">Actuator address, from 16 (first actuator) to 21</param>
/// <param name="P">Kp have to be between 0 and 3(high vibrations), tune with increments of 0.1</param>
/// <param name="I">Ki have to be around 0.01, tune with increments of 0.01</param>
/// <param name="D">Kd set it to 0 , useless for now</param>
/// <returns>Error code</returns>
extern "C"
KINOVAAPIUSBCOMMANDLAYER_API DWORD SetActuatorPID(unsigned int address, float P, float I, float D);
