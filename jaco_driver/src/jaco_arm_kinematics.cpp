/*
 * jacolib.cpp
 *
 *  Created on: June 4, 2015
 *      Author: Xiaolong Wu xwu@georgiatech-metz.fr
 */

#include <jaco_driver/jaco_arm_kinematics.h>
#include <string>


namespace jaco
{

std::string concatTfName(const std::string& prefix, const std::string name)
{
    std::stringstream ss;
    ss << prefix << name;
    return ss.str();
}


JacoKinematics::JacoKinematics(const ros::NodeHandle &node_handle)
{
    node_handle.param<std::string>("tf_prefix", tf_prefix_, "jaco_");

    node_handle.param<double>("base_to_api", base_to_api_, 0.028);
    node_handle.param<double>("base_to_j1", base_to_j1_, 0.2755);
    node_handle.param<double>("j2_to_j3", j2_to_j3_, 0.4100);
    node_handle.param<double>("j3_offset", j3_offset_, 0.0098);
    node_handle.param<double>("j3_to_j4", j3_to_j4_, 0.2501);
    node_handle.param<double>("j4_to_j5", j4_to_j5_, 0.0856);
    node_handle.param<double>("j5_to_j6", j5_to_j6_, 0.2028);
    node_handle.param<double>("j5_bend_degrees", j5_bend_degrees_, 60.0);
    node_handle.param<double>("j6_bend_degrees", j6_bend_degrees_, 60.0);
}


void JacoKinematics::updateForward(float q1, float q2, float q3, float q4, float q5, float q6)
{
    tf::Transform transform;
    tf::Quaternion rotation_q(0, 0, 0, 1);
    tf::Matrix3x3 rot_matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
    tf::Vector3 translation_v(0, 0, 0);

    /**********************Base**********************/
    /* Rotation Matrix */
    /*******               *******
    * cos(PI/2)  -sin(PI/2)    0 *
    * sin(PI/2)   cos(PI/2)    0 *
    * 0              0         1 *
    *******               *******/
    rot_matrix.setValue(1,  0,  0,
                        0,  1,  0,
                        0,  0,  1);
    rot_matrix.getRotation(rotation_q);
    transform.setRotation(rotation_q);

    /* Translation Vector */
    translation_v.setValue(0, 0, 0);
    transform.setOrigin(translation_v);

    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "arm_base"),
                                                    concatTfName(tf_prefix_, "base")));

#ifdef PRINT_DEBUG_INFO
    ROS_INFO("API Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation
             _q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());
#endif

    /**********************API**********************/
    /* Rotation Matrix */
    rot_matrix.setValue(1, 0, 0,
                        0, 1, 0,
                        0, 0, 1);
    rot_matrix.getRotation(rotation_q);
    transform.setRotation(rotation_q);

    /* API Translation Vector */
    translation_v.setValue(0, 0, base_to_api_);
    transform.setOrigin(translation_v);

    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "base"),
                                                    concatTfName(tf_prefix_, "api_origin")));

#ifdef PRINT_DEBUG_INFO
    ROS_INFO("API Translation: X = %f, Y = %f, Z = %f",
             translation_v.getX(), translation_v.getY(), translation_v.getZ());
#endif

    /**********************Joint_1**********************/
    /* Joint 1 Rotation Matrix */
    /*******                 *******
     * cos(q1)    -sin(q1)      0  *
     * sin(q1)     cos(q1)      0  *
     * 0            0           1  *
     *******                *******/
    rot_matrix.setValue(cos(q1), -sin(q1), 0,
                        sin(q1),  cos(q1), 0,
                        0,        0,       1);
    rot_matrix.getRotation(rotation_q);
    transform.setRotation(rotation_q);

    /* Joint 1 Translation Vector */
    translation_v.setValue(0, 0, base_to_j1_);
    transform.setOrigin(translation_v);

    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "link_base"),
                                                    concatTfName(tf_prefix_, "link_1")));

#ifdef PRINT_DEBUG_INFO
    ROS_INFO("Joint 1 Rotation: X = %f, Y = %f, Z = %f, W = %f",
             rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());
    ROS_INFO("Joint 1 Translation: X = %f, Y = %f, Z = %f",
             translation_v.getX(), translation_v.getY(), translation_v.getZ());
#endif

    /**********Joint_2*************/
    /* Joint 2 Rotation Matrix */
    /*******                *******
     *  cos(q2)   -sin(q2)     0  *
     *  0            0         1  *
     * -sin(q2)   -cos(q2)     0  *
     *******                *******/
    rot_matrix.setValue(cos(q2), -sin(q2), 0,
                        0,        0,       1,
                       -sin(q2), -cos(q2), 0);
    rot_matrix.getRotation(rotation_q);
    transform.setRotation(rotation_q);

    /* Joint 2 Translation Vector */
    translation_v.setValue(0, 0, 0);
    transform.setOrigin(translation_v);

    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "link_1"),
                                                    concatTfName(tf_prefix_, "link_2")));

#ifdef PRINT_DEBUG_INFO
    ROS_INFO("Joint 2 Rotation: X = %f, Y = %f, Z = %f, W = %f",
             rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());
    ROS_INFO("Joint 2 Translation: X = %f, Y = %f, Z = %f",
             translation_v.getX(), translation_v.getY(), translation_v.getZ());
#endif

    /**********************Joint_3**********************/
    /* Joint 3 Rotation Matrix */
    /*******                   *******
     *  cos(q3)    -sin(q3)        0 *
     * -sin(q3)    -cos(q3)        0 *
     *    0           0           -1 *
     *******                  *******/
    rot_matrix.setValue( cos(q3), -sin(q3), 0,
                        -sin(q3), -cos(q3), 0,
                         0,        0,      -1);
    rot_matrix.getRotation(rotation_q);
    transform.setRotation(rotation_q);

    /* Joint 3 Translation Vector */
    translation_v.setValue(j2_to_j3_, 0, j3_offset_);
    transform.setOrigin(translation_v);

    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "link_2"),
                                                    concatTfName(tf_prefix_, "link_3")));

#ifdef PRINT_DEBUG_INFO
    ROS_INFO("Joint 3 Rotation: X = %f, Y = %f, Z = %f, W = %f",
             rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());
    ROS_INFO("Joint 3 Translation: X = %f, Y = %f, Z = %f",
             translation_v.getX(), translation_v.getY(), translation_v.getZ());
#endif

    /**********************Joint_4**********************/
    /* Joint 4 Rotation Matrix */
    /*******                  *******
     *  cos(q4)    -sin(q4)       0 *
     *     0          0           1 *
     * -sin(q4)    -cos(q4)       0 *
     *******                 *******/
    rot_matrix.setValue(cos(q4), -sin(q4), 0,
                        0,        0,       1,
                       -sin(q4), -cos(q4), 0);
    rot_matrix.getRotation(rotation_q);
    transform.setRotation(rotation_q);

    /* Joint 4 Translation Vector */
    translation_v.setValue(0, -j3_to_j4_, 0);
    transform.setOrigin(translation_v);

    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "link_3"),
                                                    concatTfName(tf_prefix_, "link_4")));

#ifdef PRINT_DEBUG_INFO
    ROS_INFO("Joint 4 Rotation: X = %f, Y = %f, Z = %f, W = %f",
             rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());
    ROS_INFO("Joint 4 Translation: X = %f, Y = %f, Z = %f",
             translation_v.getX(), translation_v.getY(), translation_v.getZ());
#endif

    /************************Joint_5************************/
    /* Joint 5 Rotation Matrix */
    /*******                                         *******
     * cos(q5)             -sin(q5)                      0 *
     * sin(q5)*cos(60)      cos(q5)*cos(60)       -sin(60) *
     * sin(q5)*sin(60)      cos(q5)*sin(60)        cos(60) *
     *******                                         *******/
    rot_matrix.setValue(cos(q5), -sin(q5), 0, 
                        sin(q5)*cos((degToRad(j5_bend_degrees_))),
                        cos(q5)*cos((degToRad(j5_bend_degrees_))),
                        -sin((degToRad(j5_bend_degrees_))),
                        sin(q5)*sin((degToRad(j5_bend_degrees_))),
                        cos(q5)*sin((degToRad(j5_bend_degrees_))),
                        cos((degToRad(j5_bend_degrees_))));
    rot_matrix.getRotation(rotation_q);
    transform.setRotation(rotation_q);

    /* Joint 5 Translation Vector */
    /****             ****
     *         0         *
     *  sin(60)*j4_to_j5 *
     * -cos(60)*j4_to_j5 *
     ****             ****/
    translation_v.setValue(0,
                           sin(degToRad(j5_bend_degrees_)) * j4_to_j5_,
                          -cos(degToRad(j5_bend_degrees_)) * j4_to_j5_);
    transform.setOrigin(translation_v);

    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "link_4"),
                                                    concatTfName(tf_prefix_, "link_5")));

#ifdef PRINT_DEBUG_INFO
    ROS_INFO("Joint 5 Rotation: X = %f, Y = %f, Z = %f, W = %f",
             rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());
    ROS_INFO("Joint 5 Translation: X = %f, Y = %f, Z = %f",
             translation_v.getX(), translation_v.getY(), translation_v.getZ());
#endif

    /**********************Joint_6**********************/
    /* Joint 6 Rotation Matrix */
    /*******                                       *******
     * cos(q6)             -sin(q6)                      0 *
     * sin(q6)*cos(60)      cos(q6)*cos(60)       -sin(60) *
     * sin(q6)*sin(60)      cos(q6)*sin(60)        cos(60) *
     *******                                       *******/
    rot_matrix.setValue(cos(q6), -sin(q6), 0, 
                        sin(q6)*cos((degToRad(j6_bend_degrees_))),
                        cos(q6)*cos((degToRad(j6_bend_degrees_))),
                        -sin((degToRad(j6_bend_degrees_))),
                        sin(q6)*sin((degToRad(j6_bend_degrees_))),
                        cos(q6)*sin((degToRad(j6_bend_degrees_))),
                        cos((degToRad(j6_bend_degrees_))));
    rot_matrix.getRotation(rotation_q);
    transform.setRotation(rotation_q);
    /* Joint 6 Translation Vector */
    /****              ****
     *          0         *
     *  sin(60)*j5_to_j6 *
     * -cos(60)*j5_to_j6 *
     ****              ****/
    translation_v.setValue(0,
                           sin(degToRad(j6_bend_degrees_)) * j5_to_j6_,
                          -cos(degToRad(j6_bend_degrees_)) * j5_to_j6_);
    transform.setOrigin(translation_v);

    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "link_5"),
                                                    concatTfName(tf_prefix_, "link_6")));

#ifdef PRINT_DEBUG_INFO
    ROS_INFO("Joint 6 Rotation: X = %f, Y = %f, Z = %f, W = %f",
             rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());
    ROS_INFO("Joint 6 Translation: X = %f, Y = %f, Z = %f",
             translation_v.getX(), translation_v.getY(), translation_v.getZ());
#endif

    /*****************End-Effector*****************/
    /* E-E Rotation Matrix */
    /*******              *******
     * 1         0        0     *
     * 0     cos(180) -sin(180) *
     * 0     sin(180)  cos(180) *
     *******              *******/
    rot_matrix.setValue(1,  0,  0,
                        0, -1,  0,
                        0,  0, -1);
    rot_matrix.getRotation(rotation_q);
    transform.setRotation(rotation_q);

    /* API Translation Vector */
    translation_v.setValue(0, 0, 0);
    transform.setOrigin(translation_v);

    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "link_6"),
                                                    concatTfName(tf_prefix_, "link_hand")));

#ifdef PRINT_DEBUG_INFO
    ROS_INFO("API Translation: X = %f, Y = %f, Z = %f",
             translation_v.getX(), translation_v.getY(), translation_v.getZ());
#endif
    /**********************finger_1**********************/
    rot_matrix.setRPY(-1.7983, 1.117, 3.1416);
    rot_matrix.getRotation(rotation_q);
    transform.setRotation(rotation_q);

    translation_v.setValue(-0.03978, 0, -0.10071);
    transform.setOrigin(translation_v);

    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "link_hand"),
                                                    concatTfName(tf_prefix_, "link_finger_1")));


    /**********************finger_2**********************/
    rot_matrix.setRPY(-1.6222, 1.117, -0.23615);
    rot_matrix.getRotation(rotation_q);
    transform.setRotation(rotation_q);

    translation_v.setValue(0.03569, -0.0216, -0.10071);
    transform.setOrigin(translation_v);

    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "link_hand"),
                                                    concatTfName(tf_prefix_, "link_finger_2")));


    /**********************finger_3**********************/
    rot_matrix.setRPY(-1.5161, 1.1459, 0.23978);
    rot_matrix.getRotation(rotation_q);
    transform.setRotation(rotation_q);

    translation_v.setValue(0.03569, 0.0216, -0.10071);
    transform.setOrigin(translation_v);

    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    concatTfName(tf_prefix_, "link_hand"),
                                                    concatTfName(tf_prefix_, "link_finger_3")));
}

}  // namespace jaco
