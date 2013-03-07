/*
 * jaco_arm.h
 *
 *  Created on: Mar 6, 2013
 *      Author: mdedonato
 */

#ifndef JACO_ARM_H_
#define JACO_ARM_H_

#include <math.h>

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

namespace jaco_arm {
class jaco_kinematics {

private:
	/* Robot Length Values (Meters) */
	static inline double D1(void) {
		return 0.2102; 			//Base to elbow (Meters)
	}

	static inline double D2(void) {
		return 0.4100;			//Arm Length (Meters)
	}

	static inline double D3(void) {
		return 0.2070;			//Front Arm Length (Meters)
	}

	static inline double D4(void) {
		return 0.0750;			//First Wrist Length (Meters)
	}

	static inline double D5(void) {
		return 0.0750;			//Second Wrist Length (Meters)
	}

	static inline double D6(void) {
		return 0.1850;			//Wrist to Center of Hand(Meters)
	}

	/* Alternate Parameters */

	static inline double aa(void) {
		return ((11.0 * M_PI) / 72);
	}

	static inline double ca(void) {
		return cos(aa());
	}

	static inline double sa(void) {
		return sin(aa());
	}

	static inline double c2a(void) {
		return cos(2 * aa());
	}

	static inline double s2a(void) {
		return sin(2 * aa());
	}

	static inline double d4b(void) {
		return (D3() + (ca() - c2a() / s2a() * sa()) * D4());
	}

	static inline double d5b(void) {
		return (sa() / s2a() * D4() + (ca() - c2a() / s2a() * sa()) * D5());
	}

	static inline double d6b(void) {
		return (sa() / s2a() * D5() + D6());
	}

public:

	inline float deg_to_rad(float degrees) {
		return (degrees * (M_PI / 180));
	}

	/* Joint 1 Rotation Matrix */
	/*******									  *******
	 * cos(q1)		-sin(q1)*cos(0)		sin(q1)*sin(0)	*
	 * sin(q1)		cos(q1)*cos(0)		-cos(q1)*sin(0) *
	 * 0			sin(0)				cos(0)          *
	 *******									  *******/
	inline void J1_Rotation(float q1, tf::Quaternion& rotation_q) {
		tf::Matrix3x3 rot_matrix(cos(q1), -sin(q1) * cos(0), sin(q1) * sin(0),
				sin(q1), cos(q1) * cos(0), -cos(q1) * sin(0), 0, sin(0),
				cos(0));
		rot_matrix.getRotation(rotation_q);
	}

	/* Joint 1 Translation Vector */
	/****     ****
	 * 0*cos(q1) *
	 * 0*sin(q1) *
	 * D1        *
	 ****     ****/
	inline void J1_Translation(float q1, tf::Vector3& translation_v) {
		translation_v.setValue(0*cos(q1),0*sin(q1), this->D1());
	}



	/* Joint 2 Rotation Matrix */
	/*******									 		  *******
	 * cos(q2)		-sin(q2)*cos(-pi/2)		sin(q2)*sin(-pi/2)	*
	 * sin(q2)		cos(q2)*cos(-pi/2)		-cos(q2)*sin(-pi/2) *
	 * 0			sin(-pi/2)				cos(-pi/2)          *
	 *******											  *******/
	inline void J2_Rotation(float q2, tf::Quaternion& rotation_q) {
			tf::Matrix3x3 rot_matrix(cos(q2), -sin(q2)*cos(-1*M_PI_2), sin(q2)*sin(-1*M_PI_2),
					sin(q2), cos(q2) * cos(-1*M_PI_2), -cos(q2) * sin(-1*M_PI_2), 0, sin(-1*M_PI_2),
					cos(-1*M_PI_2));
			rot_matrix.getRotation(rotation_q);
		}

	/* Joint 2 Translation Vector */
	/****     ****
	 * 0*cos(q2) *
	 * 0*sin(q2) *
	 * 0         *
	 ****     ****/
	inline void J2_Translation(float q2, tf::Vector3& translation_v) {
		translation_v.setValue(0*cos(q2),0*sin(q2), 0);
	}




	/* Joint 3 Rotation Matrix */
	/*******									  *******
	 * cos(q3)		-sin(q3)*cos(0)		sin(q3)*sin(0)	*
	 * sin(q3)		cos(q3)*cos(0)		-cos(q3)*sin(0) *
	 * 0			sin(0)				cos(0)          *
	 *******									  *******/
	inline void J3_Rotation(float q3, tf::Quaternion& rotation_q) {
		tf::Matrix3x3 rot_matrix(cos(q3), -sin(q3) * cos(0), sin(q3) * sin(0),
				sin(q3), cos(q3) * cos(0), -cos(q3) * sin(0), 0, sin(0),
				cos(0));
		rot_matrix.getRotation(rotation_q);
	}

	/* Joint 3 Translation Vector */
	/****      ****
	 * D2*cos(q3) *
	 * D2*sin(q3) *
	 * 0          *
	 ****      ****/
	inline void J3_Translation(float q3, tf::Vector3& translation_v) {
		translation_v.setValue(this->D2()*cos(q3),this->D2()*sin(q3), 0);
	}



	/* Joint 4 Rotation Matrix */
	/*******									 		  *******
	 * cos(q4)		-sin(q4)*cos(-pi/2)		sin(q4)*sin(-pi/2)	*
	 * sin(q4)		cos(q4)*cos(-pi/2)		-cos(q4)*sin(-pi/2) *
	 * 0			sin(-pi/2)				cos(-pi/2)          *
	 *******											  *******/
	inline void J4_Rotation(float q4, tf::Quaternion& rotation_q) {
			tf::Matrix3x3 rot_matrix(cos(q4), -sin(q4)*cos(-1*M_PI_2), sin(q4)*sin(-1*M_PI_2),
					sin(q4), cos(q4) * cos(-1*M_PI_2), -cos(q4) * sin(-1*M_PI_2), 0, sin(-1*M_PI_2),
					cos(-1*M_PI_2));
			rot_matrix.getRotation(rotation_q);
		}


	/* Joint 4 Translation Vector */
	/****     ****
	 * 0*cos(q4) *
	 * 0*sin(q4) *
	 * d4b       *
	 ****     ****/
	inline void J4_Translation(float q4, tf::Vector3& translation_v) {
		translation_v.setValue(0*cos(q4),0*sin(q4), this->d4b());
	}



	/* Joint 5 Rotation Matrix */
	/*******									 		 *******
	 * cos(q5)		-sin(q5)*cos(2*aa)		sin(q5)*sin(2*aa)  *
	 * sin(q5)		cos(q5)*cos(2*aa)		-cos(q5)*sin(2*aa) *
	 * 0			sin(2*aa)				cos(2*aa)          *
	 *******										     *******/
	inline void J5_Rotation(float q5, tf::Quaternion& rotation_q) {
			tf::Matrix3x3 rot_matrix(cos(q5), -sin(q5)*cos(2*this->aa()), sin(q5)*sin(2*this->aa()),
					sin(q5), cos(q5) * cos(2*this->aa()), -cos(q5) * sin(2*this->aa()), 0, sin(2*this->aa()),
					cos(2*this->aa()));
			rot_matrix.getRotation(rotation_q);
		}

	/* Joint 5 Translation Vector */
	/****     ****
	 * 0*cos(q5) *
	 * 0*sin(q5) *
	 * d5b       *
	 ****     ****/
	inline void J5_Translation(float q5, tf::Vector3& translation_v) {
		translation_v.setValue(0*cos(q5),0*sin(q5), this->d5b());
	}




	/* Joint 6 Rotation Matrix */
	/*******									 		 *******
	 * cos(q6)		-sin(q6)*cos(2*aa)		sin(q6)*sin(2*aa)  *
	 * sin(q6)		cos(q6)*cos(2*aa)		-cos(q6)*sin(2*aa) *
	 * 0			sin(2*aa)				cos(2*aa)          *
	 *******										     *******/
	inline void J6_Rotation(float q6, tf::Quaternion& rotation_q) {
			tf::Matrix3x3 rot_matrix(cos(q6), -sin(q6)*cos(2*this->aa()), sin(q6)*sin(2*this->aa()),
					sin(q6), cos(q6) * cos(2*this->aa()), -cos(q6) * sin(2*this->aa()), 0, sin(2*this->aa()),
					cos(2*this->aa()));
			rot_matrix.getRotation(rotation_q);
		}

	/* Joint 6 Translation Vector */
	/****     ****
	 * 0*cos(q6) *
	 * 0*sin(q6) *
	 * d6b       *
	 ****     ****/
	inline void J6_Translation(float q6, tf::Vector3& translation_v) {
		translation_v.setValue(0*cos(q6),0*sin(q6), this->d6b());
	}

};
}

#endif /* JACO_ARM_H_ */
