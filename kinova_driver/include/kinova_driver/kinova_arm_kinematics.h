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

    void updateForward(float q1, float q2, float q3, float q4, float q5, float q6);

    inline float degToRad(float degrees)
    {
        return (degrees * (M_PI / 180));
    }

 private:
    tf::TransformBroadcaster broadcaster_;

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


    /* Robot Length Values (Meters) */
    double base_to_api_;
    double base_to_j1_;       // base to joint1 (Meters)
    double j1_to_j2_;         // joint1 to joint2 (Meters)
    double j2_to_j3_;         // arm length (Meters)
    double j3_offset_;        // Arm Length (Meters)
    double j3_to_j4_;         // Front Arm Length (Meters)
    double j4_to_j5_;         // First Wrist Length (Meters)
    double j5_to_j6_;         // Second Wrist Length (Meters)
    double j6_to_end_;        // Wrist to Center of Hand(Meters)
    double j5_bend_degrees_;  //
    double j6_bend_degrees_;



};

}  // namespace kinova
#endif  // KINOVA_DRIVER_KINOVA_ARM_KINEMATICS_H
