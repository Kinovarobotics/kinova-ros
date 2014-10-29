#ifndef JOYSTICK_COMMAND_TRANSLATOR_H
#define JOYSTICK_COMMAND_TRANSLATOR_H

#include <ros/ros.h>

#include <jaco_msgs/JoystickCommand.h>

#include <string>
#include "jaco_driver/jaco_comm.h"

namespace jaco {

class JoystickCommandTranslator
{
public:
    JoystickCommandTranslator(JacoComm &arm_comm, ros::NodeHandle &node_handle);

private:
   void commandReceived(const jaco_msgs::JoystickCommandConstPtr& msg);
   JoystickCommand translateCommand(const jaco_msgs::JoystickCommandConstPtr& command);

   ros::NodeHandle node_handle;
   JacoComm &arm_comm;
   ros::Subscriber subscriber;
};

}

#endif // JOYSTICK_COMMAND_TRANSLATOR_H
