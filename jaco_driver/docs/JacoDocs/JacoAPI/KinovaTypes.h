#pragma once

#define JOYSTICK_BUTTON_COUNT 16
#define NB_ADVANCE_RETRACT_POSITION		20

/// <summary>
/// <br>Type of position handled by a UserPosition.
/// <br>The types can be set or get.
/// <br>Set: this type can be loaded into the arm
/// <br>Get: this type can be received from the arm
/// <br>If a time delay is wish between trajectories, it's necessary to send a time delay POSITION_TYPE between tow trajectories.
/// </summary>
enum POSITION_TYPE
{
	CARTESIAN_POSITION = 1, /*!< A cartesian absolute position (X,Y,Z,thetaX, thetaY, thetaZ) [set,get]*/  
	ANGULAR_POSITION = 2, /*!< An actuator angular absolute position (deg) [set,get]*/  
	RETRACTED_POSITION = 3,  /*!<The retracted absolute cartesian position (X,Y,Z,thetaX, thetaY, thetaZ)  [get]*/  
	PREDEFINED_POSITION_1 = 4, /*!< A predefined absolute cartesian position (X,Y,Z,thetaX, thetaY, thetaZ) [get]*/ 
	PREDEFINED_POSITION_2 = 5, /*!< A predefined absolute cartesian position (X,Y,Z,thetaX, thetaY, thetaZ) [get]*/ 
	PREDEFINED_POSITION_3 = 6, /*!< A predefined absolute cartesian position  (X,Y,Z,thetaX, thetaY, thetaZ) [get]*/ 
	CARTESIAN_VELOCITY = 7, /*!< A cartesian velocity(X,Y,Z,thetaX, thetaY, thetaZ) [set,get]*/  
	ANGULAR_VELOCITY = 8,  /*!< An actuator angular velocity (deg/s) [set,get]*/  
	PREDEFINED_POSITION_4 = 9, /*!< A predefined absolute cartesian position  (X,Y,Z,thetaX, thetaY, thetaZ) [get]*/ 
	PREDEFINED_POSITION_5 = 10, /*!< A predefined absolute cartesian position  (X,Y,Z,thetaX, thetaY, thetaZ) [get]*/ 
	TIME_DELAY = 12, /*!< A time delay (s) [set,get]*/  
};

enum HAND_MODE
{
	POSITION_MODE = 1,
	VELOCITY_MODE = 2,
    NO_FINGER = 3,
};

enum ARM_LATERALITY
{
	RIGHT_HAND,
	LEFT_HAND,
};




/// <summary>
/// <br>Contains some info of each actuator.
/// <br>Values could represent a velocity(rad/s), an absolute position(deg.), a current(A) or a force(Nm). It depends of the called function.
/// <br><b>WARNING: If you send this structure, always define all variables, there is no initial values. Otherwise, there will be random information sent to the arm.</b>
/// <br>Suggestion: use the function memset(&yourStructure, 0, sizeof(yourStructure)) from the std lib and set the memory.
/// </summary>
struct AngularInfo
{
	float Actuator1;
	float Actuator2;
	float Actuator3;
	float Actuator4;
	float Actuator5;
	float Actuator6;
};

/// <summary>
/// <br>Contains some cartesian info of the hand.
/// <br>Values could represent a velocity(m/s), an absolute position(m), a current(A) or a force(N). It depends of the called function.
/// <br>WARNING: If you send this structure, always define all variables, there is no initial values. Otherwise, there will be random informaition sent to the arm.
/// <br>Suggestion: use the function memset(&yourStructure, 0, sizeof(yourStructure)) from the std lib and set the memory
/// </summary>
struct CartesianInfo
{
	float X; /*!< Cartesian X value */  
	float Y; /*!< Cartesian Y value */  
	float Z; /*!< Cartesian Z value */  
	float ThetaX; /*!< Euler angle X axe value in rad */  
	float ThetaY; /*!< Euler angle Y axe value in rad */ 
	float ThetaZ; /*!< Euler angleZ axe value in rad */ 
};

/// <summary>
/// <br>Contains all the sensor values
/// </summary>
struct SensorsInfo
{
	float Voltage; /*!< Main voltage in V*/ 
	float Current; /*!< Main current in A*/ 
	float AccelerationX; /*!< X acceleration in m/s^2 */ 
	float AccelerationY; /*!< Y acceleration in m/s^2*/ 
	float AccelerationZ; /*!< Z acceleration in m/s^2*/ 
	float ActuatorTemp1; /*!< Actuator1 temp in deg. C*/ 
	float ActuatorTemp2; /*!< Actuator2 temp in deg. C*/ 
	float ActuatorTemp3; /*!< Actuator3 temp in deg. C*/ 
	float ActuatorTemp4; /*!< Actuator4 temp in deg. C*/ 
	float ActuatorTemp5; /*!< Actuator5 temp in deg. C*/ 
	float ActuatorTemp6; /*!< Actuator6 temp in deg. C*/ 
	float FingerTemp1;  /*!< Finger1 temp in deg. C*/ 
	float FingerTemp2;  /*!< Finger2 temp in deg. C*/ 
	float FingerTemp3;  /*!< Finger3 temp in deg. C*/ 
};

/// <summary>
/// <br>Contains info about fingers position 
/// <br>It could be an absolute position (0 to ~53(closed)) or a velocity (0 to 200(really fast) )
/// <br><b>WARNING: If you send this structure, always define all variables, there is no initial values. Otherwise, there will be random information sent to the arm.</b>
/// <br>Suggestion: use the function memset(&yourStructure, 0, sizeof(yourStructure)) from the std lib and set the memory
/// </summary>
struct FingersPosition
{
	float Finger1; /*!< Finger1 position value*/ 
	float Finger2; /*!< Finger2 position value*/ 
	float Finger3; /*!< Finger3 position value*/ 
};

/// <summary>
/// <br>Contains all cartesian positions of the hand and their fingers
/// <br>The CartesianPosition is only a mix of two position structures.
//  <br>Note: The finger position is always the same however it is angular or cartesian
/// <br><b>WARNING: If you send this structure, always define all variables, there is no initial values. Otherwise, there will be random information sent to the arm.</b>
/// <br>Suggestion: use the function memset(&yourStructure, 0, sizeof(yourStructure)) from the std lib and set the memory.
/// </summary>
struct CartesianPosition
{
	CartesianInfo Coordinates;
	FingersPosition Fingers;
};

/// <summary>
/// <br>Contains all angular positions of the hand and their fingers
/// <br>The AngularPosition is only a mix of two angular structures.
/// <br>Note: The finger position is always the same however it is angular or cartesian
/// <br><b>WARNING: If you send this structure, always define all variables, there is no initial values. Otherwise, there will be random information sent to the arm.</b>
/// <br>Suggestion: use the function memset(&yourStructure, 0, sizeof(yourStructure)) from the std lib and set the memory
/// </summary>
struct AngularPosition
{
	AngularInfo Actuators;
	FingersPosition Fingers;
};

//	________________________________________________________________________________________
//	|							|							|								|
//	|							|		Cartesian mode		|		Angular mode			|
//	|___________________________|___________________________|_______________________________|
//	|	speedParameter1 		|	Linear Speed			|	Speed of 3 first joints		|
//	|	speedParameter2 		|	Rotation Speed			|	Speed of 3 last joints		|
//	|	speedParameter3 		|	Finger Speed			|	Finger speed				|
//	|							|							|								|
//	|	forceParameter1 		|	Linear force			|	3 first joints force		|
//	|	forceParameter2 		|	Rotation Force			|	3 last joints force			|
//	|	forceParameter3 		|	finger force			|	Finger force				|
//	|							|							|								|
//	|	accelerationParameter1 	|	Linear acceleration		|	3 first joints acceleration	|
//	|	accelerationParameter2 	|	Rotation acceleration	|	3 last joints acceleration	|
//	|	accelerationParameter3 	|	finger acceleration		|	Finger force acceleration	|
//	|___________________________|___________________________|_______________________________|


/// <summary>
/// <br>Absolute limitations of the robotic arm. The limitations are different for cartesian or angular modes
/// <br><b>WARNING: If you send this structure, always define all variables, there is no initial values. Otherwise, there will be random information sent to the arm.</b>
/// <br>Suggestion: use the function memset(&yourStructure, 0, sizeof(yourStructure)) from the std lib and set the memory
/// </summary>
struct Limitation
{
	float speedParameter1; /*!< Cartesian mode: Max linear speed (m/s) || Angular mode: Speed of 3 first joints	(deg/s)*/ 
	float speedParameter2;  /*!< Cartesian mode: Max rotation speed (rad/s) || Angular mode: Speed of 3 last joints (deg/s)*/ 
	float speedParameter3; /*!< Cartesian mode: Max finger speed || Angular mode: Max finger speed */ 
	float forceParameter1;  /*!< Cartesian mode: Max linear force (N) || Angular mode: 3 first joints force (Nm)*/ 
	float forceParameter2; /*!< Cartesian mode: Max angular force (Nm) || Angular mode: 3 last joints force (Nm)*/
	float forceParameter3; /*!< Cartesian mode: Max fingers force || Angular mode: Finger force */
	float accelerationParameter1; /*!< Cartesian mode: Max linear acceleration (m/s^2) || Angular mode: 3 first joints acceleration (deg/s^2)*/ 
	float accelerationParameter2;  /*!< Cartesian mode: Max angular acceleration (rad/s^2) || Angular mode: 3 last joints acceleration (deg/s^2)*/ 
	float accelerationParameter3; /*!< Cartesian mode: Max finger acceleration || Angular mode: Finger force acceleration */ 
};

/// <summary>
/// <br>Contains all the information about a position.
/// <br>Values could represent an absolute position, a velocity or a delay of time. It depends of the called function.
/// <br><b>WARNING: If you send this structure, always define all variables, there is no initial values. Otherwise, there will be random information sent to the arm.</b>
/// <br>If you don't need a variable, set it to 0 (It's the case in many situations).</br>
/// <br>Suggestion: use the function memset(&yourStructure, 0, sizeof(yourStructure)) from the std lib and set the memory.
/// <br><b>WARNING: Be sure that the UserPosition is well defined, otherwise the arm could try to reach undesired position</b>
/// </summary>
struct UserPosition
{
	POSITION_TYPE Type; /*!< Type of the position, can be an absolute position, a velocity or a time delay*/ 
	float Delay; /*!< Delay of time to execute another trajectory after this one (s).*/ 
    CartesianInfo CartesianPosition; /*!< Cartesian position values, depends of the type*/ 
	AngularInfo Actuators; /*!< Actuator values, depends of the type*/

	HAND_MODE HandMode; /*!< Type of fingers positioning*/ 
	FingersPosition Fingers; /*!< Finger position value, depends of the type*/      
};

/// <summary>
/// <br>Contains all the information about a trajectory point
/// <br>Values could represent a velocity, an absolute position or a delay of time. It depends of the called function.
/// <br><b>WARNING: If you send this structure, always define all variables, there is no initial values. Otherwise, there will be random information sent to the arm.</b>
/// <br>If you don't need a variable, set it to 0 (It's the case in many situations)
/// <br>Suggestion: use the function memset(&yourStructure, 0, sizeof(yourStructure)) from the std lib and set the memory
/// <br><b>WARNING: Be sure that the UserPosition is well defined, otherwise the arm could try to reach undesired position</b></br>
/// </summary>
struct TrajectoryPoint
{
	UserPosition Position; /*!<  Position related to the trajectory */
	int LimitationsActive; /*!< Number of active limitations, set it to 0 if not needed*/  
	Limitation Limitations;  /*!< Number of active limitations, set each related variables to 0 if not needed*/  
};

/// <summary>
/// <br>Contains all the information about the trajectory FIFO
/// </summary>
struct TrajectoryFIFO
{
	unsigned int TrajectoryCount; /*!<  Number of trajectory to proceed in the FIFO */
	float UsedPercentage; /*!<  Percentage of use of the FIFO (100% = max capacity) */
	unsigned int MaxSize;  /*!<  Maximal number of trajectories that can be in the FIFO */
};

/// <summary>
/// <br>Contains all the information about singularities form the actual hand position
/// </summary>
struct SingularityVector
{
	int TranslationSingularityCount; /*!< Number of near singularities from the current hand position for a translation (X,Y,Z)*/
	int OrientationSingularityCount; /*!< Number of near singularities from the current hand position for a rotation (Euler angles: theta X, theta Y and theta Z)*/
	float TranslationSingularityDistance; /*!<Relative distance of the nearest singularity point from the current hand position*/
	float OrientationSingularityDistance; /*!<Relative angle of the nearest singularity point from the current hand orientation*/
	CartesianInfo RepulsionVector; /*!<Cartesian direction to follow for avoiding the nearest singularity */
};

/// <summary>
/// <br>Contains all the information about a virtual joystick command (same behavior as an hard-wired).
/// </summary>
struct JoystickCommand
{
	short ButtonValue[JOYSTICK_BUTTON_COUNT]; /*! Array of button values, 0=not pushed, 1 = pushed */

	float InclineLeftRight; /*!< Value is a ratio of movement velocity between -1 and 1, 0 = no movement. 1 = 100% of max speed*/ 
    float InclineForwardBackward;  /*!< Value is a ratio of movement velocity between -1 and 1, 0 = no movement. 1 = 100% of max speed*/ 
    float Rotate; /*!< Value is a ratio of movement velocity between -1 and 1, 0 = no movement. 1 = 100% of max speed*/ 

    float MoveLeftRight; /*!< Value is a ratio of movement velocity between -1 and 1, 0 = no movement. 1 = 100% of max speed*/  
    float MoveForwardBackward;/*!< Value is a ratio of movement velocity between -1 and 1, 0 = no movement. 1 = 100% of max speed*/ 
	float PushPull; /*!< Value is a ratio of movement velocity between -1 and 1, 0 = no movement. 1 = 100% of max speed*/ 
};


/// <summary>
/// <br>Contains all the information about a client configuration
/// <br><b>WARNING: If you send this structure, always define all variables, there is no initial values. Otherwise, there will be random information sent to the arm.</b>
/// <br>If you don't need a variable, set it to 0 (It's the case in many situations)
/// <br>Suggestion: use the function memset(&yourStructure, 0, sizeof(yourStructure)) from the std lib and set the memory
/// </summary>
struct ClientConfigurations
{
    char ClientID[20]; /*!< Client ID, ex: MAX232*/ 
    char ClientName[20]; /*!< Client Name, ex: Maxime Tremblay*/  
	char Organization[20]; /*!< Organization, ex: Kinova*/  

    char Serial[20]; /*!< Serial number, ex: PJ 0000 0000 0000*/
    char Model[20]; /*!< Arm model, ex: 3.0.1*/

	ARM_LATERALITY Laterality; /*!<For REHAB: Laterality of the arm: can be right handed or left handed*/ 

    float MaxLinearSpeed; /*!< Absolute arm linear speed (m/s), range 0 to 0.15 */
	float MaxAngularSpeed;  /*!< Absolute arm angular speed (rad/s) (thetaX, thetaY and thetaZ), range 0 to 0.6 */
    float MaxLinearAcceleration; /*!< Absolute arm linear acceleration (m/s^2), range 0 to 0.8*/
	float MaxAngularAcceleration;  /*!< Absolute arm angular acceleration (rad/s^2), range 0 to 2 */
	float MaxForce;  /*!< Absolute linear force (X,Y,Z), 0 to 20N. 
					      THE LINEAR FORCE IS REALLY APPROXIMATIVE.
					 */
    float Sensibility; /*!< Sensibility of the joystick(%)(virtual or hard-wired), range 0 to 100*/
	float DrinkingHeight; /*!< X distance of the center of rotation from hand center (m), has a purpose for rehab*/ 

	int ComplexRetractActive; /*!<Is the arm in complex retract mode (a custum position)*/ 
	float RetractedPositionAngle; /*!<Angle of the joint 2 while using normal retract*/ 
	int RetractedPositionCount;/*!<Number of recorded retract positions*/ 
	UserPosition RetractPositions[NB_ADVANCE_RETRACT_POSITION]; /*!< Array of recorded retract positions*/ 

	float DrinkingDistance;  /*!<Y distance of the center of rotation from hand center (m), has a purpose for rehab*/ 
	int Fingers2and3Inverted; /*!< Is the fingers 2 and 3 control is inverted for left handed mode*/ 
	float DrinkingLenght; /*!<Z distance of the center of rotation from hand center (m), has a purpose for rehab*/ 

	int DeletePreProgrammedPositionsAtRetract; /*!<Are the preprogrammed retract positions deleted*/ 

	int EnableFlashErrorLog; /*!<Enable logging of flash memory errors*/ 
	int EnableFlashPositionLog;  /*!<Enable logging of positions stored in flash memory*/ 
};			