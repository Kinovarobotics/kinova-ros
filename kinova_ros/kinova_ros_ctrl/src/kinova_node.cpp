#include <kinova_ros_ctrl/kinova_hardware_interface.hpp>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

using namespace kinova_ros_ctrl;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinova_node");

    ros::NodeHandle n, np("~");

    double p;
    np.param("period", p, 0.01);
    ros::Duration period(p);

    KinovaHardwareInterface               hwi(n, np);
    controller_manager::ControllerManager cm(&hwi);

    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        ros::Time end = now + period;

        hwi.read();
        cm.update(now, period);
        hwi.write();

        ros::spinOnce();

        ros::Duration left(end - ros::Time::now());
        left.sleep();
    }
}


