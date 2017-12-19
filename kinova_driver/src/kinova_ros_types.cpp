/**
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™
 *
 *  File: kinova_ros_types.cpp
 *  Desc: Wrappers around Kinova structs to facilitate easier conversion to ROS
 *		  types.
 *  Auth: Alex Bencz
 *
 *  Copyright (c) 2013, Clearpath Robotics, Inc.
 *  All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com
 *
 */

#include <math.h>
#include <angles/angles.h>
#include <kinova_driver/kinova_ros_types.h>

namespace kinova
{

// %Tag(converntion functions)%


// rads 0 to 2*M_PI
float normalizePositiveInRads(float rads)
{
    return static_cast<float>(angles::normalize_angle_positive(rads));
}

// rads -M_PI to M_PI
float normalizeInRads(float rads)
{
    return static_cast<float>(angles::normalize_angle(rads));
}

// degrees 0 to 360
float normalizePositiveInDegrees(float degrees)
{
    return angles::to_degrees(angles::normalize_angle_positive(angles::from_degrees(degrees)));
}

// degrees -180 to 180
float normalizeInDegrees(float degrees)
{
    return angles::to_degrees(angles::normalize_angle(angles::from_degrees(degrees)));
}

// %EndTag(converntion functions)%


bool areValuesClose(float first, float second, float tolerance)
{
    return ((first <= second + tolerance) && (first >= second - tolerance));
}

/**
 * @brief EulerXYZ2Quaternion
 * @param tx input Euler angle tx
 * @param ty input Euler angle ty
 * @param tz input Euler angle tz
 * @return output Quaternion
 * @warning DSP(KinovaPose) use Euler-XYZ convention, while ROS use Euler-ZYX by default.
 */
tf::Quaternion EulerXYZ2Quaternion(float tx, float ty, float tz)
{
    float sx = sin(0.5*tx);
    float cx = cos(0.5*tx);
    float sy = sin(0.5*ty);
    float cy = cos(0.5*ty);
    float sz = sin(0.5*tz);
    float cz = cos(0.5*tz);

    float qx, qy, qz, qw;
    qx =  sx*cy*cz + cx*sy*sz;
    qy = -sx*cy*sz + cx*sy*cz;
    qz =  sx*sy*cz + cx*cy*sz;
    qw = -sx*sy*sz + cx*cy*cz;

    tf::Quaternion q;
    q.setX(qx);
    q.setY(qy);
    q.setZ(qz);
    q.setW(qw);
    return q;
}


/**
 * @brief EulerXYZ2Matrix3x3
 * Build rotation matrix using Euler-XYZ angles from KinovaPose
 * @param tx input Euler angle tx
 * @param ty input Euler angle ty
 * @param tz input Euler angle tz
 * @return output rotation matrix
 * @warning DSP(KinovaPose) use Euler-XYZ convention, while ROS use Euler-ZYX by default.
 */
tf::Matrix3x3 EulerXYZ2Matrix3x3(float tx, float ty, float tz)
{
    tf::Matrix3x3 Rot_m;
    float sx = sin(tx);
    float cx = cos(tx);
    float sy = sin(ty);
    float cy = cos(ty);
    float sz = sin(tz);
    float cz = cos(tz);
    Rot_m.setValue(cz*cy, -sz*cy, sy,
                   cz*sy*sx+sz*cx, -sz*sy*sx+cz*cx, -cy*sx,
                   -cz*sy*cx+sz*sx, sz*sy*cx+cz*sx, cy*cx);
    return Rot_m;
}


/**
 * @brief getEulerXYZ
 * getEulerXYZ get Euler-XYZ convention for KinovaPose orientation from Rotation matrix convention.
 * @param Rot_matrix input rotation matrix
 * @param tx output Euler angle tx
 * @param ty output Euler angle ty
 * @param tz output Euler angle tz
 * @warning DSP(KinovaPose) use Euler-XYZ convention, while ROS use Euler-ZYX by default.
 */
void getEulerXYZ(tf::Matrix3x3 &Rot_matrix, float &tx, float &ty, float &tz)
{
    float a11 = Rot_matrix.getRow(0).getX();
    float a12 = Rot_matrix.getRow(0).getY();
    float a13 = Rot_matrix.getRow(0).getZ();
    float a23 = Rot_matrix.getRow(1).getZ();
    float a33 = Rot_matrix.getRow(2).getZ();

    tx = atan2(-a23, a33);
    ty = atan2(a13, sqrt(1-a13*a13));
    tz = atan2(-a12, a11);

}


/**
 * @brief getEulerXYZ get Euler-XYZ convention for KinovaPose orientation from Quaternion convention.
 * @param q input Quaternion
 * @param tx output Euler angle tx
 * @param ty output Euler angle ty
 * @param tz output Euler angle tz
 * @warning DSP(KinovaPose) use Euler-XYZ convention, while ROS use Euler-ZYX by default.
 */
void getEulerXYZ(tf::Quaternion &q, float &tx, float &ty, float &tz)
{
    float qx = q.getX();
    float qy = q.getY();
    float qz = q.getZ();
    float qw = q.getW();

    tx = atan2((2*qw*qx-2*qy*qz),(qw*qw-qx*qx-qy*qy+qz*qz));
    ty = asin(2*qw*qy+2*qx*qz);
    tz = atan2((2*qw*qz-2*qx*qy),(qw*qw+qx*qx-qy*qy-qz*qz));
}

bool valid_kinovaRobotType(const std::string &kinova_RobotType)
{
    if(kinova_RobotType.size()!=8)
    {
        ROS_ERROR("The kinova_RobotType should be 8 characters, but get %lu instead. kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2s7a300 refers to jaco v2 7DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.", (unsigned long)(kinova_RobotType.size()));
        return false;
    }

    // kinova_RobotType = [robot_categary, robot_version, wrist, dof, model, fingerNum, res7, res8];

    // check categary
    if (kinova_RobotType[0]=='j')
    {
        // check version
        if ((kinova_RobotType[1]!='1') && (kinova_RobotType[1]!='2'))
        {
            ROS_ERROR("version error.");
            return false;
        }

        // check wrist
        if ((kinova_RobotType[2]!='s') && (kinova_RobotType[2]!='n'))
        {
            ROS_ERROR("wrist type error.");
            return false;
        }

        // check dof
        if ((kinova_RobotType[3]!='4') && (kinova_RobotType[3]!='6') && (kinova_RobotType[3]!='7'))
        {
            ROS_WARN("Number of expected joints for standard robot configurations is 4, 6, 7");
            return false;
        }

        // check model
        if ((kinova_RobotType[4]!='s') && (kinova_RobotType[4]!='a'))
        {
            ROS_ERROR("model type error.");
            return false;
        }

        // check number of fingers
        if ((kinova_RobotType[5]!='2') && (kinova_RobotType[5]!='3'))
        {
            ROS_ERROR("finger number error.");
            return false;
        }
    }
    else if (kinova_RobotType[0]=='m')
    {
        // check version
        if ((kinova_RobotType[1]!='1')) return false;
        // check wrist
        if ((kinova_RobotType[2]!='s') && (kinova_RobotType[2]!='n')) return false;
        // check dof
        if ((kinova_RobotType[3]!='4') && (kinova_RobotType[3]!='6')) return false;
        // check model
        if ((kinova_RobotType[4]!='s') && (kinova_RobotType[4]!='a')) return false;
        // check number of fingers
        if ((kinova_RobotType[5]!='2') && (kinova_RobotType[5]!='3')) return false;
    }
    else if (kinova_RobotType[0]=='r')
    {
        // roco property check
    }
    else if (kinova_RobotType[0]=='c')
    {
        // customized robot property check
    }
    else
    {
        ROS_ERROR("The categary(first charactor of kinova_RobotType) should be one of the set [0, j, m, r, c], but obtained %c", kinova_RobotType[0]);
        return false;
    }

    return true;

}


// Exceptions
// ----------
KinovaCommException::KinovaCommException(const std::string& message, const int error_code)
{
    std::stringstream ss;
        ss << "KinovaCommException: " << message << " (return code: " << error_code << ")" << std::endl;
    desc_ = ss.str();
}


const char* KinovaCommException::what() const throw()
{
    return desc_.c_str();
}


// Class definitions
// -----------------
/**
 * @brief KinovaPose::KinovaPose
 * obtain KinovaPose from standard ROS geometry_msg.
 * @param pose ROS geometry_msgs[x,y,z,qx,qy,qz,qw], in meters and quaterion.
 * @warning KinovaPose use Euler-XYZ convention, while ROS use Euler-ZYX convention by default.
 */
KinovaPose::KinovaPose(const geometry_msgs::Pose &pose)
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.orientation, q);

    X = static_cast<float>(pose.position.x);
    Y = static_cast<float>(pose.position.y);
    Z = static_cast<float>(pose.position.z);

    getEulerXYZ(q,ThetaX, ThetaY,ThetaZ);
}


/**
 * @brief KinovaPose::KinovaPose
 * Copy from CartesianInfo to KinovaPose, and change degree to radians.
 * @param pose units are in meters and degrees
 */
KinovaPose::KinovaPose(const CartesianInfo &pose)
{
    X = pose.X;
    Y = pose.Y;
    Z = pose.Z;

    ThetaX = normalizeInRads(pose.ThetaX);
    ThetaY = normalizeInRads(pose.ThetaY);
    ThetaZ = normalizeInRads(pose.ThetaZ);
}

/**
 * @brief construct geometric::Pose message from KinovaPose
 * @return geometry_msgs::Pose[x,y,z,qx,qy,qz,qw] position in meters, orientation is in Quaternion.
 */
geometry_msgs::Pose KinovaPose::constructPoseMsg()
{
    geometry_msgs::Pose pose;
    tf::Quaternion position_quaternion;

    // However, DSP using Euler-XYZ, while ROS using Euler-ZYX.
    KinovaPose::getQuaternion(position_quaternion);
    tf::quaternionTFToMsg(position_quaternion, pose.orientation);

    pose.position.x = X;
    pose.position.y = Y;
    pose.position.z = Z;

    return pose;
}


kinova_msgs::KinovaPose KinovaPose::constructKinovaPoseMsg()
{
    kinova_msgs::KinovaPose pose;

    pose.X = X;
    pose.Y = Y;
    pose.Z = Z;
    pose.ThetaX = ThetaX;
    pose.ThetaY = ThetaY;
    pose.ThetaZ = ThetaZ;

    return pose;
}

tf::Quaternion KinovaPose::getQuaternion(tf::Quaternion &q)
{
    q = EulerXYZ2Quaternion(ThetaX, ThetaY, ThetaZ);
}


/**
 * @brief construct geometric::Wrench message from KinovaPose data structure
 *
 * KinovaPose[X,Y,Z,ThetaX,ThetaY,ThetaZ] doesn't store pose information in this case. Instead, it stores force and torque correspondingly.
 *
 * @return geometry_msgs [Fx,Fy,Fz,Tx,Ty,Tz] in Newton and Nm.
 */
geometry_msgs::Wrench KinovaPose::constructWrenchMsg()
{
    geometry_msgs::Wrench wrench;

    wrench.force.x  = X;
    wrench.force.y  = Y;
    wrench.force.z  = Z;
    wrench.torque.x = ThetaX;
    wrench.torque.y = ThetaY;
    wrench.torque.z = ThetaZ;

    return wrench;
}


/**
 * @brief check all the position and orientation values, to determine if current KinovaPose reach "other"
 *
 * @param other the pose to be compared with, normally the motion goal. In meters and radians.
 * @param position_tolerance  threshold to be considered as identical between two position values in the same axis.
 * @param EulerAngle_tolerance threshold to be considered as identical between two EulerAngle values along the same axis.
 * @return true if both position and orientation can be considerred identical.
 */
bool KinovaPose::isCloseToOther(const KinovaPose &other, float position_tolerance, float EulerAngle_tolerance) const
{
    bool status = true;
    status = status && areValuesClose(X, other.X, position_tolerance);
    status = status && areValuesClose(Y, other.Y, position_tolerance);
    status = status && areValuesClose(Z, other.Z, position_tolerance);
    status = status && (areValuesClose(fmod(ThetaX, 2*M_PI), fmod(other.ThetaX, 2*M_PI), EulerAngle_tolerance) ||
                        areValuesClose(fmod(ThetaX, 2*M_PI) + 2*M_PI, fmod(other.ThetaX, 2*M_PI), EulerAngle_tolerance) ||
                        areValuesClose(fmod(ThetaX, 2*M_PI), fmod(other.ThetaX, 2*M_PI) + 2*M_PI, EulerAngle_tolerance));
    status = status && (areValuesClose(fmod(ThetaY, 2*M_PI), fmod(other.ThetaY, 2*M_PI), EulerAngle_tolerance) ||
                        areValuesClose(fmod(ThetaY, 2*M_PI) + 2*M_PI, fmod(other.ThetaY, 2*M_PI), EulerAngle_tolerance) ||
                        areValuesClose(fmod(ThetaY, 2*M_PI), fmod(other.ThetaY, 2*M_PI) + 2*M_PI, EulerAngle_tolerance));
    status = status && (areValuesClose(fmod(ThetaZ, 2*M_PI), fmod(other.ThetaZ, 2*M_PI), EulerAngle_tolerance) ||
                        areValuesClose(fmod(ThetaZ, 2*M_PI) + 2*M_PI, fmod(other.ThetaZ, 2*M_PI), EulerAngle_tolerance) ||
                        areValuesClose(fmod(ThetaZ, 2*M_PI), fmod(other.ThetaZ, 2*M_PI) + 2*M_PI, EulerAngle_tolerance));
    return status;
}

/**
 * @brief KinovaAngles[Actuator1 ... Actuator7] stores 7 joint values in degrees
 *
 * KinovaAngles[Actuator1 ... Actuator7] can go beyond the range of [0 360]
 *
 * @param angles in degrees
 */
KinovaAngles::KinovaAngles(const kinova_msgs::JointAngles &angles)
{

    Actuator1 = angles.joint1;
    Actuator2 = angles.joint2;
    Actuator3 = angles.joint3;
    Actuator4 = angles.joint4;
    Actuator5 = angles.joint5;
    Actuator6 = angles.joint6;
    Actuator7 = angles.joint7;
}

/**
 * @brief KinovaAngles[Actuator1 ... Actuator7] stores 7 joint values in degrees
 * @param angles in degrees
 */
KinovaAngles::KinovaAngles(const AngularInfo &angles)
{
    Actuator1 = angles.Actuator1;
    Actuator2 = angles.Actuator2;
    Actuator3 = angles.Actuator3;
    Actuator4 = angles.Actuator4;
    Actuator5 = angles.Actuator5;
    Actuator6 = angles.Actuator6;
    Actuator7 = angles.Actuator7;
}

/**
 * @brief KinovaAngles::constructAnglesMsg
 * @return kinova_msgs::JointAngles is used as messanges for topic in ROS, in degrees.
 */
kinova_msgs::JointAngles KinovaAngles::constructAnglesMsg()
{
    kinova_msgs::JointAngles angles;
    angles.joint1 = Actuator1;
    angles.joint2 = Actuator2;
    angles.joint3 = Actuator3;
    angles.joint4 = Actuator4;
    angles.joint5 = Actuator5;
    angles.joint6 = Actuator6;
    angles.joint7 = Actuator7;
    return angles;
}


/**
 * @brief check all the joint values, to determine if current KinovaAngles reach "other"
 * @param other joint angles to be compared with, in degrees.
 * @param tolerance threshold to be considered as identical between two joints.
 * @return true if all joint values can be considerred identical.
 */
bool KinovaAngles::isCloseToOther(const KinovaAngles &other, float tolerance) const
{
    bool status = true;

    status = status && (areValuesClose(fmod(Actuator1, 360.0), fmod(other.Actuator1, 360.0), tolerance) ||
                        areValuesClose(fmod(Actuator1, 360.0) + 360, fmod(other.Actuator1, 360.0), tolerance) ||
                        areValuesClose(fmod(Actuator1, 360.0), fmod(other.Actuator1, 360.0) + 360, tolerance));
    status = status && (areValuesClose(fmod(Actuator2, 360.0), fmod(other.Actuator2, 360.0), tolerance) ||
                        areValuesClose(fmod(Actuator2, 360.0) + 360, fmod(other.Actuator2, 360.0), tolerance) ||
                        areValuesClose(fmod(Actuator2, 360.0), fmod(other.Actuator2, 360.0) + 360, tolerance));
    status = status && (areValuesClose(fmod(Actuator3, 360.0), fmod(other.Actuator3, 360.0), tolerance) ||
                        areValuesClose(fmod(Actuator3, 360.0) + 360, fmod(other.Actuator3, 360.0), tolerance) ||
                        areValuesClose(fmod(Actuator3, 360.0), fmod(other.Actuator3, 360.0) + 360, tolerance));
    status = status && (areValuesClose(fmod(Actuator4, 360.0), fmod(other.Actuator4, 360.0), tolerance) ||
                        areValuesClose(fmod(Actuator4, 360.0) + 360, fmod(other.Actuator4, 360.0), tolerance) ||
                        areValuesClose(fmod(Actuator4, 360.0), fmod(other.Actuator4, 360.0) + 360, tolerance));
    status = status && (areValuesClose(fmod(Actuator5, 360.0), fmod(other.Actuator5, 360.0), tolerance) ||
                        areValuesClose(fmod(Actuator5, 360.0) + 360, fmod(other.Actuator5, 360.0), tolerance) ||
                        areValuesClose(fmod(Actuator5, 360.0), fmod(other.Actuator5, 360.0) + 360, tolerance));
    status = status && (areValuesClose(fmod(Actuator6, 360.0), fmod(other.Actuator6, 360.0), tolerance) ||
                        areValuesClose(fmod(Actuator6, 360.0) + 360, fmod(other.Actuator6, 360.0), tolerance) ||
                        areValuesClose(fmod(Actuator6, 360.0), fmod(other.Actuator7, 360.0) + 360, tolerance));
    status = status && (areValuesClose(fmod(Actuator7, 360.0), fmod(other.Actuator7, 360.0), tolerance) ||
                        areValuesClose(fmod(Actuator7, 360.0) + 360, fmod(other.Actuator6, 360.0), tolerance) ||
                        areValuesClose(fmod(Actuator7, 360.0), fmod(other.Actuator7, 360.0) + 360, tolerance));

    return status;
}


FingerAngles::FingerAngles(const kinova_msgs::FingerPosition &position)
{
    Finger1 = position.finger1;
    Finger2 = position.finger2;
    Finger3 = position.finger3;
}


FingerAngles::FingerAngles(const FingersPosition &angle)
{
    Finger1 = angle.Finger1;
    Finger2 = angle.Finger2;
    Finger3 = angle.Finger3;
}


kinova_msgs::FingerPosition FingerAngles::constructFingersMsg()
{
    kinova_msgs::FingerPosition angles;
    angles.finger1 = Finger1;
    angles.finger2 = Finger2;
    angles.finger3 = Finger3;
    return angles;
}


bool FingerAngles::isCloseToOther(const FingerAngles &other, float tolerance) const
{
    bool status = true;
    status = status && areValuesClose(Finger1, other.Finger1, tolerance);
    status = status && areValuesClose(Finger2, other.Finger2, tolerance);
    status = status && areValuesClose(Finger3, other.Finger3, tolerance);
    return status;
}

}  // namespace kinova
