/*
 * jaco_arm.h
 *
 *  Created on: Mar 6, 2013
 *      Author: mdedonato
 */

#ifndef JACO_ARM_H_
#define JACO_ARM_H_

#include <math.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
//#include <jaco_driver/jaco_arm_controller.h>

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

namespace jaco {
class JacoKinematics {

public:
	JacoKinematics(void);

	static inline float Q1(float joint_angle) {
		return (-1 * (joint_angle - 180));
	}
	static inline float Q2(float joint_angle) {
		return (joint_angle - 270);
	}
	static inline float Q3(float joint_angle) {
		return (-1 * (joint_angle - 90));
	}
	static inline float Q4(float joint_angle) {
		return (-1 * (joint_angle - 180));
	}
	static inline float Q5(float joint_angle) {
		return (-1 * (joint_angle - 180));
	}
	static inline float Q6(float joint_angle) {
		return (-1 * (joint_angle - (180 + 80)));
	}

	/* Robot Length Values (Meters) */
	static inline double BaseToJ1(void) {
		return 0.1370; 			//Base to J1 (Meters)
	}

	/* Robot Length Values (Meters) */
	static inline double J1ToJ2(void) {
		return 0.1181; 			//J1 to J2 (Meters)
	}


	static inline double J2ToJ3(void) {
		return 0.4100;			//Arm Length (Meters)
	}
	static inline double J3Offset(void) {
			return 0.0113;			//Arm Length (Meters)
		}

	static inline double J3ToJ4(void) {
		return 0.2070;			//Front Arm Length (Meters)
	}

	static inline double J4ToJ5(void) {
		return 0.0750;			//First Wrist Length (Meters)
	}

	static inline double J5ToJ6(void) {
		return 0.0750;			//Second Wrist Length (Meters)
	}

	static inline double J6ToEnd(void) {
		return 0.1850;			//Wrist to Center of Hand(Meters)
	}



public:

	inline float deg_to_rad(float degrees) {
		return (degrees * (M_PI / 180));
	}


	void UpdateForward(float q1,float q2,float q3,float q4,float q5,float q6);
private:
	tf::TransformBroadcaster br;

};
}

#endif /* JACO_ARM_H_ */
