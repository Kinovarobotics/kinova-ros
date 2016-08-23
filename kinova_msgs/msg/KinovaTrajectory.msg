#TrajectoryPoint used in Kinova API
#UserPosition
#Position Type
#Define used here are the same as in KinovaTypes.h

int32 PositionType
int32 PositionTypeNOMOVEMENT_POSITION=0
int32 PositionTypeCARTESIAN_POSITION=1
int32 PositionTypeANGULAR_POSITION=2
int32 PositionTypeRETRACTED=3
int32 PositionTypePREDEFINED1=4
int32 PositionTypePREDEFINED2=5
int32 PositionTypePREDEFINED3=6
int32 PositionTypeCARTESIAN_VELOCITY=7
int32 PositionTypeANGULAR_VELOCITY=8
int32 PositionTypePREDEFINED4=9
int32 PositionTypePREDEFINED5=10
int32 PositionTypeANY_TRAJECTORY=11
int32 PositionTypeTIME_DELAY=12

#delay (in case of PositionType=TIME_DELAY)
float32 Delay

#Cartesian Position in meter or Velocity in meter/s
geometry_msgs/Pose pose

#Angular Position in Degree or velocity in radian/s (Not used
kinova_msgs/JointAngles Actuators

#Hand mode define if the hand is in position or velocity mode
int32 HandMode
int32 HandModeHAND_NOMOVEMENT=0
int32 HandModePOSITION_MODE=1
int32 HandModeVELOCITY_MODE=2

#finger position or velocity
kinova_msgs/FingerPosition Fingers

#Flag to indicate if the limitation are active or not (1 is active, 0 is not)
int32 LimitationsActive

#Synchronization action (1) or inactice (0), Angular Control only
int32 SynchroType

#Limitation to Apply to the robot if LimitationsActive=1
float32 speedParameter1
float32 speedParameter2
