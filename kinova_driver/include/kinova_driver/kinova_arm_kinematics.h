/*
 * kinova_arm.h
 *
 *  Created on: Mar 6, 2013
 *      Author: mdedonato
 */

#ifndef KINOVA_DRIVER_KINOVA_ARM_KINEMATICS_H
#define KINOVA_DRIVER_KINOVA_ARM_KINEMATICS_H

#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <vector>

/******************************************/
/**************DH Parameters***************/
/******************************************/
/* i * alpha(i-1) * a(i-1) * di  * theta1 */
/******************************************/
/* 1 * 0          * 0      * D1  * q1     */
/* 2 * -pi/2      * 0      * 0   * q2     */
/* 3 * 0          * D2     * 0   * q3     */
/* 4 * -pi/2      * 0      * d4b * q4     */
/* 5 * 2*aa       * 0      * d5b * q5     */
/* 6 * 2*aa       * 0      * d6b * q6     */
/******************************************/

namespace kinova
{

class KinovaKinematics
{
 public:
    explicit KinovaKinematics(const ros::NodeHandle& node_handle, std::string& kinova_robotType);

    void updateForward(float* Q);

    inline float degToRad(float degrees)
    {
        return (degrees * (M_PI / 180));
    }

 private:
    tf::TransformBroadcaster broadcaster_;
    tf::Transform DHParam2Transform(float d, float theta, float a, float alpha);

    // Parameters
    std::string kinova_robotType_;
    std::string tf_prefix_;

    char robot_category_;
    int robot_category_version_;
    char wrist_type_;
    int arm_joint_number_;
    char robot_mode_;
    int finger_number_;
    int joint_total_number_;

    //base frame for robot
    std::string baseFrame;

    /* Robot Length Values (Meters) */
    double D1_;         // base to elbow
    double D2_;         // arm length
    double D3_;         // front arm length
    double D4_;         // frist wrist length
    double D5_;         // second wrist length
    double D6_;         // wrist to center of hand
    double D7_;
    double e2_;         // offset of joint
    double wrist_deg_;  // wrist bend degree

    /* classic DH table parameters */
    std::vector<double> DH_a_;
    std::vector<double> DH_d_;
    std::vector<double> DH_alpha_;
    std::vector<double> DH_theta_sign_;
    std::vector<double> DH_theta_offset_;

};

}  // namespace kinova
#endif  // KINOVA_DRIVER_KINOVA_ARM_KINEMATICS_H
