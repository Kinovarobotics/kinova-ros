/*
 * kinovalib.cpp
 *
 *  Created on: Mar 10, 2013
 *      Author: mdedonato
 */

#include <kinova_driver/kinova_arm_kinematics.h>
#include <kinova_driver/kinova_ros_types.h>
#include <string>


namespace kinova
{

std::string concatTfName(const std::string& prefix, const std::string name)
{
    std::stringstream ss;
    ss << prefix << name;
    return ss.str();
}


KinovaKinematics::KinovaKinematics(const ros::NodeHandle &node_handle, std::string& kinova_robotType)
    : kinova_robotType_(kinova_robotType)
{
    if (valid_kinovaRobotType(kinova_robotType_) == false)
    {
        ROS_WARN("Invalid kinova_robotType error! Obtained: %s.", kinova_robotType_.c_str());
        return;
    }

//    tf_prefix_ = kinova_robotType_ + "_" + boost::lexical_cast<string>(same_type_index); // in case of multiple same_type robots
    tf_prefix_ = kinova_robotType_ + "_";

    // Maximum number of joints on Kinova-like robots:
    robot_category_ = kinova_robotType_[0];
    robot_category_version_ = kinova_robotType_[1]-'0';
    wrist_type_ = kinova_robotType_[2];
    arm_joint_number_ = kinova_robotType_[3]-'0';
    robot_mode_ = kinova_robotType_[4];
    finger_number_ = kinova_robotType_[5]-'0';
    int joint_total_number_ = arm_joint_number_ + finger_number_;

    if (robot_category_=='j') // jaco robot
    {
        // special parameters for jaco
    }
    else if (robot_category_ == 'm') // mico robot
    {
        // special parameters for mico
    }
    else if (robot_category_ == 'r') // roco robot
    {
        // special parameters for roco
    }
    else
    {
        // special parameters for custom robot or other cases
    }

    // parameters stored in DSP chip
    node_handle.param<double>("D1", D1_, 0.2755);
    node_handle.param<double>("D2", D2_, 0.41);
    node_handle.param<double>("D3", D3_, 0.2073);
    node_handle.param<double>("D4", D4_, 0.0741);
    node_handle.param<double>("D5", D5_, 0.0741);
    node_handle.param<double>("D6", D6_, 0.160);
    node_handle.param<double>("e2", e2_, -0.0098);
    node_handle.param<double>("wrist_deg", wrist_deg_, 60.0);

}

tf::Transform KinovaKinematics::DHParam2Transform(float d, float theta, float a, float alpha)
{
    tf::Transform transform;
    tf::Quaternion rotation_q(0, 0, 0, 1);
    tf::Matrix3x3 rot_matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
    tf::Vector3 translation_v(0, 0, 0);

    rot_matrix.setValue(cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),
                        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),
                        0,           sin(alpha),             cos(alpha));
    rot_matrix.getRotation(rotation_q);
    transform.setRotation(rotation_q.normalize());

    translation_v.setValue(a*cos(theta), a*sin(theta), d);
    transform.setOrigin(translation_v);
    return transform;
}

// IMPORTANT!!!
// In the robot DSP chip, the classical D-H parameters are used to define the robot. Therefore, the frame definition is different comparing with the frames in URDF model.
void KinovaKinematics::updateForward(float q1, float q2, float q3, float q4, float q5, float q6)
{
    tf::Transform transform;

    // parameters for classical D-H convention
    double aa = wrist_deg_/2 * M_PI/180.0, sa = sin(aa), s2a = sin(2*aa);
    double a1 = 0, a2 = D2_, a3 = 0, a4 = 0, a5 = 0, a6 = 0;
    double d1 = D1_, d2 = 0, d3 = e2_, d4 = -(D3_ + sa/s2a * D4_),
           d5 = -(sa/s2a * D4_ + sa/s2a * D5_), d6 = -(sa/s2a * D5_ + D6_);
    double alpha1 = M_PI/2, alpha2 = M_PI, alpha3 = M_PI/2, alpha4 = 2*aa, alpha5 = 2*aa, alpha6 = M_PI;

    double theta1 = -q1, theta2 = q2-M_PI/2, theta3 = q3+M_PI/2, theta4 = q4, theta5 = q5-M_PI, theta6 = q6+M_PI/2;

    /**************************************/
    /*                                    */
    /*  Computer and publish transform    */
    /*                                    */
    /**************************************/
    transform = DHParam2Transform(0, 0, 0, 0);
    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "root"),
                                                    concatTfName(tf_prefix_, "link_base")));


    transform = DHParam2Transform(d1, theta1, a1, alpha1);
    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "link_base"),
                                                    concatTfName(tf_prefix_, "link_1")));

    transform = DHParam2Transform(d2, theta2, a2, alpha2);
    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "link_1"),
                                                    concatTfName(tf_prefix_, "link_2")));

    transform = DHParam2Transform(d3, theta3, a3, alpha3);
    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "link_2"),
                                                    concatTfName(tf_prefix_, "link_3")));

    transform = DHParam2Transform(d4, theta4, a4, alpha4);
    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "link_3"),
                                                    concatTfName(tf_prefix_, "link_4")));

    transform = DHParam2Transform(d5, theta5, a5, alpha5);
    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "link_4"),
                                                    concatTfName(tf_prefix_, "link_5")));

    transform = DHParam2Transform(d6, theta6, a6, alpha6);
    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "link_5"),
                                                    concatTfName(tf_prefix_, "link_6")));
}

}  // namespace kinova
