/*
 * kinovalib.cpp
 *
 *  Created on: Mar 10, 2013
 *      Author: mdedonato
 */

#include <kinova_driver/kinova_arm_kinematics.h>
#include <kinova_driver/kinova_ros_types.h>
#include <boost/lexical_cast.hpp>


namespace kinova
{

std::string concatTfName(const std::string& prefix, const std::string name)
{
    std::stringstream ss;
    ss << prefix << name;
    return ss.str();
}
std::string concatTfName(const std::string& prefix, const std::string name, const int index)
{
    std::stringstream ss;
    ss << prefix << name << index;
    return ss.str();
}
std::vector<double> array2vector(double* array, int length)
{
    std::vector<double> result;
    for (int i=0; i<length; i++)
    {
        result.push_back(array[i]);
    }
    return result;
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

    if (kinova_robotType_.substr(0,4) == "j2n4" || kinova_robotType_.substr(0,4) == "m1n4")
        {
            // parameters stored in DSP chip
            node_handle.param<double>("D1", D1_, 0.2755);
            if(robot_category_ == 'j') {
                node_handle.param<double>("D2", D2_, 0.41);
                node_handle.param<double>("D3", D3_, 0.2073);
                node_handle.param<double>("e2", e2_, 0.0098);
            }
            else
            {
                node_handle.param<double>("D2", D2_, 0.29);
                node_handle.param<double>("D3", D3_, 0.1233);
                node_handle.param<double>("e2", e2_, 0.0070);
            }
            node_handle.param<double>("D4", D4_, 0.160);
            node_handle.param<double>("wrist_deg", wrist_deg_, 60.0);

            double aa = wrist_deg_/2 * M_PI/180.0, sa = sin(aa), s2a = sin(2*aa); // temp parameters

            // avoid use dynamic array DH_a[arm_joint_number_], but vector for DH parameters.
            double DH_a[4] = {0,  D2_, 0, 0};
            double DH_d[4] = {D1_, 0, -e2_, -(D3_ + D4_)};
            double DH_alpha[4] = {M_PI/2, M_PI, M_PI/2, M_PI};
            // DH_theta = DH_theta_sign*Q + DH_theta_offset
            double DH_theta_sign[4] = {-1, 1, 1, 1};
            double DH_theta_offset[4] = {0, -M_PI/2, M_PI/2, 3/2*M_PI};

            // copy local array values to class-scope vector.
            DH_a_ = array2vector(DH_a, arm_joint_number_);
            DH_d_ = array2vector(DH_d, arm_joint_number_);
            DH_alpha_ = array2vector(DH_alpha, arm_joint_number_);
            DH_theta_sign_ = array2vector(DH_theta_sign, arm_joint_number_);
            DH_theta_offset_ = array2vector(DH_theta_offset, arm_joint_number_);
        }
    else if (kinova_robotType_.substr(0,4) == "j2n6" || kinova_robotType_.substr(0,4) == "m1n6" )
    {
        // parameters stored in DSP chip
        node_handle.param<double>("D1", D1_, 0.2755);
        if(robot_category_ == 'j') {
            node_handle.param<double>("D2", D2_, 0.41);
            node_handle.param<double>("D3", D3_, 0.2073);
            node_handle.param<double>("e2", e2_, 0.0098);
        }
        else
        {
            node_handle.param<double>("D2", D2_, 0.29);
            node_handle.param<double>("D3", D3_, 0.1233);
            node_handle.param<double>("e2", e2_, 0.0070);
        }
        node_handle.param<double>("D4", D4_, 0.0741);
        node_handle.param<double>("D5", D5_, 0.0741);
        node_handle.param<double>("D6", D6_, 0.160);
        node_handle.param<double>("wrist_deg", wrist_deg_, 60.0);

        double aa = wrist_deg_/2 * M_PI/180.0, sa = sin(aa), s2a = sin(2*aa); // temp parameters

        // avoid use dynamic array DH_a[arm_joint_number_], but vector for DH parameters.
        double DH_a[6] = {0,  D2_, 0, 0, 0, 0};
        double DH_d[6] = {D1_, 0, -e2_, -(D3_ + sa/s2a * D4_), -(sa/s2a * D4_ + sa/s2a * D5_), -(sa/s2a * D5_ + D6_)};
        double DH_alpha[6] = {M_PI/2, M_PI, M_PI/2, 2*aa, 2*aa, M_PI};
        // DH_theta = DH_theta_sign*Q + DH_theta_offset
        double DH_theta_sign[6] = {-1, 1, 1, 1, 1, 1};
        double DH_theta_offset[6] = {0, -M_PI/2, +M_PI/2, 0, -M_PI, +M_PI/2};

        // copy local array values to class-scope vector.
        DH_a_ = array2vector(DH_a, arm_joint_number_);
        DH_d_ = array2vector(DH_d, arm_joint_number_);
        DH_alpha_ = array2vector(DH_alpha, arm_joint_number_);
        DH_theta_sign_ = array2vector(DH_theta_sign, arm_joint_number_);
        DH_theta_offset_ = array2vector(DH_theta_offset, arm_joint_number_);
    }
    else if (kinova_robotType_.substr(0,4) == "j2s6")
    {
        // parameters stored in DSP chip
        node_handle.param<double>("D1", D1_, 0.2755);
        node_handle.param<double>("D2", D2_, 0.41);
        node_handle.param<double>("D3", D3_, 0.2073);
        node_handle.param<double>("D4", D4_, 0.1038);
        node_handle.param<double>("D5", D5_, 0.1038);
        node_handle.param<double>("D6", D6_, 0.160);
        node_handle.param<double>("e2", e2_, 0.0098);

        // avoid use dynamic array DH_a[arm_joint_number_], but vector for DH parameters.
        double DH_a[6] = {0,  D2_, 0, 0, 0, 0};
        double DH_d[6] = {-D1_, 0, -e2_, -(D3_ + D4_), 0, -(D5_ + D6_)};
        double DH_alpha[6] = {M_PI/2, M_PI, M_PI/2, M_PI/2, M_PI/2, M_PI};
        // DH_theta = DH_theta_sign*Q + DH_theta_offset
        double DH_theta_sign[6] = {1, 1, 1, 1, 1, 1};
        double DH_theta_offset[6] = {-M_PI, M_PI/2, M_PI/2, 0, 0, -M_PI/2};

        // copy local array values to class-scope vector.
        DH_a_ = array2vector(DH_a, arm_joint_number_);
        DH_d_ = array2vector(DH_d, arm_joint_number_);
        DH_alpha_ = array2vector(DH_alpha, arm_joint_number_);
        DH_theta_sign_ = array2vector(DH_theta_sign, arm_joint_number_);
        DH_theta_offset_ = array2vector(DH_theta_offset, arm_joint_number_);
    }
    else if (kinova_robotType_.substr(0,4) == "j2s7")
    {
        // parameters stored in DSP chip
        node_handle.param<double>("D1", D1_, 0.2755);
        node_handle.param<double>("D2", D2_, 0.205);
        node_handle.param<double>("D3", D3_, 0.205);
        node_handle.param<double>("D4", D4_, 0.2073);
        node_handle.param<double>("D5", D5_, 0.1038);
        node_handle.param<double>("D6", D6_, 0.1038);
        node_handle.param<double>("D7", D7_, 0.160);
        node_handle.param<double>("e2", e2_, 0.0098);

        // avoid use dynamic array DH_a[arm_joint_number_], but vector for DH parameters.
        double DH_a[7] = {0,  0, 0, 0, 0, 0, 0};
        double DH_d[7] = {-D1_, 0, -(D2_+ D3_), -e2_, -(D4_ + D5_), 0, -(D6_ + D7_)};
        double DH_alpha[7] = {M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI};
        // DH_theta = DH_theta_sign*Q + DH_theta_offset
        double DH_theta_sign[7] = {-1, 1, 1, 1, 1, 1, 1};
        double DH_theta_offset[7] = {0, 0, 0, 0, 0, 0, 0};

        // copy local array values to class-scope vector.
        DH_a_ = array2vector(DH_a, arm_joint_number_);
        DH_d_ = array2vector(DH_d, arm_joint_number_);
        DH_alpha_ = array2vector(DH_alpha, arm_joint_number_);
        DH_theta_sign_ = array2vector(DH_theta_sign, arm_joint_number_);
        DH_theta_offset_ = array2vector(DH_theta_offset, arm_joint_number_);
    }
    else
    {
        // special parameters for custom robot or other cases
        printf("Please specify the kinematic of robots other than jaco and mico!\n");
    }

    node_handle.param<std::string>("base_frame", baseFrame, "root");
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

/**************************************/
/*                                    */
/*  Computer and publish transform    */
/*                                    */
/**************************************/
// IMPORTANT!!! In the robot DSP chip, the classical D-H parameters are used to define the robot. Therefore, the frame definition is different comparing with the frames in URDF model.
void KinovaKinematics::updateForward(float* Q)
{
    tf::Transform transform;
    // the orientation of frame0 is differently defined starting from Jaco 6 spherical robot.
    if(kinova_robotType_.substr(0,4) == "j2s6" || kinova_robotType_.substr(0,4) == "j2s7")
    {
        transform = DHParam2Transform(0, 0, 0, M_PI);
    }
    else
    {
        transform = DHParam2Transform(0, 0, 0, 0);
    }
    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    baseFrame,
                                                    concatTfName(tf_prefix_, "link_base")));
    double DH_theta_i;
    for (int i = 0; i<arm_joint_number_; i++)
    {
        DH_theta_i = DH_theta_sign_[i]*Q[i] + DH_theta_offset_[i];
        transform = DHParam2Transform(DH_d_[i], DH_theta_i, DH_a_[i], DH_alpha_[i]);
        if(i==0)
        {
            broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), concatTfName(tf_prefix_, "link_base"), concatTfName(tf_prefix_, "link_1")));
        }
        else
        {
            broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),                                                            concatTfName(tf_prefix_, "link_", i), concatTfName(tf_prefix_, "link_", i+1)));
        }
    }

}

}  // namespace kinova
