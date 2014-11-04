#include "../include/jaco_driver/joystick_command_translator.h"
#include <kinova/KinovaTypes.h>
#include "jaco_driver/jaco_types.h"
#include <string>

namespace jaco {

JoystickCommandTranslator::JoystickCommandTranslator(JacoComm &arm_comm, ros::NodeHandle &node_handle)
    : arm_comm(arm_comm),
      node_handle(node_handle, "joystick_translator")
{
    std::string joy_topic("/jaco_arm_driver/in/joystick_command");
    subscriber = node_handle.subscribe(joy_topic, 10, &JoystickCommandTranslator::commandReceived, this);
}

void JoystickCommandTranslator::commandReceived(const jaco_msgs::JoystickCommandConstPtr& msg)
{
    arm_comm.sendJoystickCommand(this->translateCommand(msg));
}

JoystickCommand JoystickCommandTranslator::translateCommand(const jaco_msgs::JoystickCommandConstPtr& command)
{
    JoystickCommand result;

    for(int i = 0; i < JOYSTICK_BUTTON_COUNT; i++)
    {
        result.ButtonValue[i] = command->ButtonValue[i];
    }

    result.InclineLeftRight = command->InclineLeftRight;
    result.InclineForwardBackward = command->InclineForwardBackward;
    result.Rotate = command->Rotate;
    result.MoveLeftRight = command->MoveLeftRight;
    result.MoveForwardBackward = command->MoveForwardBackward;
    result.PushPull = command->PushPull;

    return result;
}

}
