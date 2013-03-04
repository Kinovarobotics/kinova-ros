
#ifndef KINOVA_TYPES_H_
#define KINOVA_TYPES_H_


#define JOYSTICK_BUTTON_COUNT 16
#define NB_ADVANCE_RETRACT_POSITION		20

enum POSITION_TYPE
{
	NOMOVEMENT_POSITION,
	CARTESIAN_POSITION,
	ANGULAR_POSITION,
	RETRACTED_POSITION,
	PREDEFINED_POSITION_1,
	PREDEFINED_POSITION_2,
	PREDEFINED_POSITION_3,
	CARTESIAN_VELOCITY,
	ANGULAR_VELOCITY,
	PREDEFINED_POSITION_4,
	PREDEFINED_POSITION_5,
	ANY_TRAJECTORY,
	TIME_DELAY,
};

enum HAND_MODE
{
	HAND_NOMOVEMENT,
	POSITION_MODE,
	VELOCITY_MODE,
    NO_FINGER,
    ONE_FINGER,
    TWO_FINGER,
    THREEFINGER
};

enum ArmLaterality
{
	RIGHTHAND,
	LEFTHAND,
};

struct AngularInfo
{
	float Actuator1;
	float Actuator2;
	float Actuator3;
	float Actuator4;
	float Actuator5;
	float Actuator6;
};

struct CartesianInfo
{
	float X;
	float Y;
	float Z;
	float ThetaX;
	float ThetaY;
	float ThetaZ;
};

struct SensorsInfo
{
	float Voltage;
	float Current;
	float AccelerationX;
	float AccelerationY;
	float AccelerationZ;
	float ActuatorTemp1;
	float ActuatorTemp2;
	float ActuatorTemp3;
	float ActuatorTemp4;
	float ActuatorTemp5;
	float ActuatorTemp6;
	float FingerTemp1;
	float FingerTemp2;
	float FingerTemp3;
};

struct FingersPosition
{
	float Finger1;
	float Finger2;
	float Finger3;
};

struct CartesianPosition
{
	CartesianInfo Coordinates;
	FingersPosition Fingers;
};

struct AngularPosition
{
	AngularInfo Actuators;
	FingersPosition Fingers;
};

struct Limitation
{
	float speedParameter1;
	float speedParameter2;
	float speedParameter3;
	float forceParameter1;
	float forceParameter2;
	float forceParameter3;
	float accelerationParameter1;
	float accelerationParameter2;
	float accelerationParameter3;
};

struct UserPosition
{
	POSITION_TYPE Type;
	float Delay;
    CartesianInfo CartesianPosition;
	AngularInfo Actuators;

	HAND_MODE HandMode;
	FingersPosition Fingers;    
};

struct TrajectoryPoint
{
	UserPosition Position;
	int LimitationsActive;
	Limitation Limitations;
};

struct TrajectoryFIFO
{
	unsigned int TrajectoryCount;
	float UsedPercentage;
	unsigned int MaxSize;
};

struct SingularityVector
{
	int TranslationSingularityCount;
	int OrientationSingularityCount;
	float TranslationSingularityDistance;
	float OrientationSingularityDistance;
	CartesianInfo RepulsionVector;
};

struct JoystickCommand
{
	short ButtonValue[JOYSTICK_BUTTON_COUNT];

	float InclineLeftRight;
    float InclineForwardBackward;
    float Rotate;

    float MoveLeftRight;
    float MoveForwardBackward;
	float PushPull;
};



struct ClientConfigurations
{
    char ClientID[20];
    char ClientName[20];
	char Organization[20];

    char Serial[20];
    char Model[20];

	ArmLaterality Laterality;

    float MaxLinearSpeed;
	float MaxAngularSpeed;
    float MaxLinearAcceleration;
	float MaxAngularAcceleration;
	float MaxForce;
    float Sensibility;
	float DrinkingHeight;

	int ComplexRetractActive;
	float RetractedPositionAngle;
	int RetractedPositionCount;
	UserPosition RetractPositions[NB_ADVANCE_RETRACT_POSITION];

	float DrinkingDistance;
	int Fingers2and3Inverted;
	float DrinkingLenght;

	int DeletePreProgrammedPositionsAtRetract;

	int EnableFlashErrorLog;
	int EnableFlashPositionLog;
};			

#endif /* KINOVA_TYPES_H_ */

