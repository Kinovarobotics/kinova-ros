#include <kinova_ros_ctrl/kinova_hardware_interface.hpp>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

using namespace kinova_ros_ctrl;

namespace {
    void loop()
    {
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinova_node");

    ros::NodeHandle n, np("~");

    double p;
    np.param("period", p, 0.1);
    ros::Duration period(p);

    KinovaHardwareInterface               hwi(n, np);
    controller_manager::ControllerManager cm(&hwi);

    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control
    // loop.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate rate(period);
    while (ros::ok()) {

        hwi.read();
        cm.update(ros::Time::now(), period);
        hwi.write();

        rate.sleep();
    }

    spinner.stop();

    return 0;
}


