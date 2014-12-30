#include <kinova_ros_ctrl/kinova_hardware_interface.hpp>
#include <ros/ros.h>

using namespace kinova_ros_ctrl;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinova_node");

    ros::NodeHandle n, np("~");

    KinovaHardwareInterface hwi(n, np);

    ros::spin();
}


