/*
 * jacolib.cpp
 *
 *  Created on: Mar 10, 2013
 *      Author: mdedonato
 */
#include <jaco_driver/jaco_arm_kinematics.h>

namespace jaco {
JacoKinematics::JacoKinematics(void) {

}
void JacoKinematics::UpdateForward(float q1, float q2, float q3, float q4,
		float q5, float q6) {
	tf::Transform transform;
	tf::Quaternion rotation_q(0, 0, 0, 0);
	tf::Matrix3x3 rot_matrix(0, 0, 0, 0, 0, 0, 0, 0, 0);
	tf::Vector3 translation_v(0, 0, 0);

	/**********************Base**********************/
		/* API Rotation */
		rotation_q.setValue(0, 0, 0, 0); //zero rotation
		rot_matrix.setValue(0, 0, 0, 0, 0, 0, 0, 0, 0);


		/* API Rotation Matrix */
		/*******						    	 *******
		 * cos(PI/2)	    	-sin(PI/2)	       	 0 *
		 * sin(PI/2)		    cos(PI/2)     	     0 *
		 * 0					0					 1 *
		 *******				 		  	     *******/
		rot_matrix.setValue(cos(M_PI_2), -sin(M_PI_2), 0, sin(M_PI_2), cos(M_PI_2), 0, 0, 0, 1);
		rot_matrix.getRotation(rotation_q);

	#ifdef PRINT_DEBUG_INFO
		/* Display Results */
		ROS_INFO(
				"API Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());
	#endif
		transform.setRotation(rotation_q); //Set Rotation

		/* API Translation */
		translation_v.setValue(0, 0, 0); //zero translation
		//get API translation vector

		/* API Translation Vector */
		/****     ****
		 * 0 		 *
		 * 0		 *
		 * 0        *
		 ****     ****/
		translation_v.setValue(0, 0, 0);

	#ifdef PRINT_DEBUG_INFO

		/* Display Results */
		ROS_INFO(
				"API Translation: X = %f, Y = %f, Z = %f", translation_v.getX(), translation_v.getY(), translation_v.getZ());
	#endif

		transform.setOrigin(translation_v);	//Set Translation

		/* Broadcast Transform */
		br.sendTransform(
				tf::StampedTransform(transform, ros::Time::now(), "arm_base",
						"jaco_base"));
		/***************************************************/
		/**********************API**********************/
				/* API Rotation */
				rotation_q.setValue(0, 0, 0, 0); //zero rotation
				rot_matrix.setValue(0, 0, 0, 0, 0, 0, 0, 0, 0);


				/* API Rotation Matrix */
				/*******						    	            *******
				 * cos(deg_to_rad(15))		-sin(deg_to_rad(15))		0 *
				 * sin(deg_to_rad(15))		cos(deg_to_rad(15))     	0 *
				 * 0					    0					        1 *
				 *******				 			                *******/
				//rot_matrix.setValue(cos(deg_to_rad(16.5)), -sin(deg_to_rad(16.5)), 0, sin(deg_to_rad(16.5)), cos(deg_to_rad(16.5)), 0, 0, 0, 1);
				rot_matrix.setValue(1,0,0,0,1,0,0,0,1);

				rot_matrix.getRotation(rotation_q);

			#ifdef PRINT_DEBUG_INFO
				/* Display Results */
				ROS_INFO(
						"API Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());
			#endif
				transform.setRotation(rotation_q); //Set Rotation

				/* API Translation */
				translation_v.setValue(0, 0, 0); //zero translation
				//get API translation vector

				/* API Translation Vector */
				/****     ****
				 * 0 		 *
				 * 0		 *
				 * 0        *
				 ****     ****/
				translation_v.setValue(0, 0, 0.028);

			#ifdef PRINT_DEBUG_INFO

				/* Display Results */
				ROS_INFO(
						"API Translation: X = %f, Y = %f, Z = %f", translation_v.getX(), translation_v.getY(), translation_v.getZ());
			#endif

				transform.setOrigin(translation_v);	//Set Translation

				/* Broadcast Transform */
				br.sendTransform(
						tf::StampedTransform(transform, ros::Time::now(), "jaco_base",
								"jaco_api_origin"));
				/***************************************************/


	/**********************Joint_1**********************/
	/* Joint 1 Rotation */
	rotation_q.setValue(0, 0, 0, 0); //zero rotation
	rot_matrix.setValue(0, 0, 0, 0, 0, 0, 0, 0, 0);

	/* Joint 1 Rotation Matrix */
	/*******					 *******
	 * cos(q1)		-sin(q1)		    0  *
	 * -sin(q1)		-cos(q1)     	0  *
	 * 0			0				-1 *
	 *******				     *******/

	rot_matrix.setValue(cos(q1), -sin(q1), 0, -sin(q1), -cos(q1), 0, 0, 0, -1);
	rot_matrix.getRotation(rotation_q);

#ifdef PRINT_DEBUG_INFO
	/* Display Results */
	ROS_INFO(
			"Joint 1 Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());
#endif
	transform.setRotation(rotation_q); //Set Rotation

	/* Joint 1 Translation */
	translation_v.setValue(0, 0, 0); //zero translation
	//get joint 1 translation vector

	/* Joint 1 Translation Vector */
	/****     ****
	 * 0 		 *
	 * 0		 *
	 * D1        *
	 ****     ****/
	translation_v.setValue(0, 0, this->BaseToJ1());

#ifdef PRINT_DEBUG_INFO

	/* Display Results */
	ROS_INFO(
			"Joint 1 Translation: X = %f, Y = %f, Z = %f", translation_v.getX(), translation_v.getY(), translation_v.getZ());
#endif

	transform.setOrigin(translation_v);	//Set Translation

	/* Broadcast Transform */
	br.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(), "jaco_base",
					"jaco_joint_1"));
	/***************************************************/

	/**********************Joint_2**********************/
	/* Joint 2 Rotation */
	rotation_q.setValue(0, 0, 0, 0); //zero rotation
	rot_matrix.setValue(0, 0, 0, 0, 0, 0, 0, 0, 0);
	/* Joint 2 Rotation Matrix */
	/*******					 *******
	 * sin(q2)		cos(q2)		0  *
	 * 0			0				1 *
	 * cos(q2)		-sin(q2)     	0  *
	 *******				     *******/
	rot_matrix.setValue(sin(q2), cos(q2), 0, 0, 0, 1, cos(q2), -sin(q2), 0);

	rot_matrix.getRotation(rotation_q);

#ifdef PRINT_DEBUG_INFO
	/* Display Results */
	ROS_INFO(
			"Joint 2 Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());
#endif
	transform.setRotation(rotation_q); //Set Rotation

	/* Joint 2 Translation */
	translation_v.setValue(0, 0, 0); //zero translation
	//get joint 2 translation vector

	/* Joint 2 Translation Vector */
	/****     ****
	 * 0 		 *
	 * 0		 *
	 * -D1        *
	 ****     ****/
	translation_v.setValue(0, 0 , -this->J1ToJ2());

#ifdef PRINT_DEBUG_INFO

	/* Display Results */
	ROS_INFO(
			"Joint 2 Translation: X = %f, Y = %f, Z = %f", translation_v.getX(), translation_v.getY(), translation_v.getZ());
#endif

	transform.setOrigin(translation_v);	//Set Translation

	/* Broadcast Transform */
	br.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(), "jaco_joint_1",
					"jaco_joint_2"));
	/***************************************************/

	/**********************Joint_3**********************/
	/* Joint 3 Rotation */
	rotation_q.setValue(0, 0, 0, 0); //zero rotation
	rot_matrix.setValue(0, 0, 0, 0, 0, 0, 0, 0, 0);
	/* Joint 3 Rotation Matrix */
	/*******					*******
	 * -cos(q3)		sin(q3)		0 *
	 * sin(q2)		cos(q3)		0 *
	 * 0			0		    -1 *
	 *******				    *******/
	rot_matrix.setValue(-cos(q3), sin(q3), 0, sin(q3), cos(q3), 0, 0, 0, -1);

	rot_matrix.getRotation(rotation_q);

#ifdef PRINT_DEBUG_INFO
	/* Display Results */
	ROS_INFO(
			"Joint 3 Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());
#endif
	transform.setRotation(rotation_q); //Set Rotation

	/* Joint 3 Translation */
	translation_v.setValue(0, 0, 0); //zero translation
	//get joint 3 translation vector

	/* Joint 3 Translation Vector */
	/****     ****
	 * cos(q2)	 *
	 * sin(q2)	 *
	 * D2        *
	 ****     ****/
	translation_v.setValue(this->J2ToJ3(), 0, 0);

#ifdef PRINT_DEBUG_INFO

	/* Display Results */
	ROS_INFO(
			"Joint 3 Translation: X = %f, Y = %f, Z = %f", translation_v.getX(), translation_v.getY(), translation_v.getZ());
#endif

	transform.setOrigin(translation_v);	//Set Translation

	/* Broadcast Transform */
	br.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(), "jaco_joint_2",
					"jaco_joint_3"));
	/***************************************************/

	/**********************Joint_3 Offset**********************/
	/* Joint 3 Rotation */
	rotation_q.setValue(0, 0, 0, 0); //zero rotation
	rot_matrix.setValue(0, 0, 0, 0, 0, 0, 0, 0, 0);
	/* Joint 3 Rotation Matrix */
	/*******					*******
	 * -cos(q3)		sin(q3)		0 *
	 * sin(q2)		cos(q3)		0 *
	 * 0			0		    -1 *
	 *******				    *******/
	rot_matrix.setValue(1, 0, 0, 0, 1, 0, 0, 0, 1);

	rot_matrix.getRotation(rotation_q);

#ifdef PRINT_DEBUG_INFO
	/* Display Results */
	ROS_INFO(
			"Joint 3 Offset Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());
#endif
	transform.setRotation(rotation_q); //Set Rotation

	/* Joint 3 Translation */
	translation_v.setValue(0, 0, 0); //zero translation
	//get joint 3 translation vector

	/* Joint 3 Translation Vector */
	/****     ****
	 * cos(q2)	 *
	 * sin(q2)	 *
	 * D2        *
	 ****     ****/
	translation_v.setValue(0, 0, -this->J3Offset());

#ifdef PRINT_DEBUG_INFO

	/* Display Results */
	ROS_INFO(
			"Joint 3 Offset Translation: X = %f, Y = %f, Z = %f", translation_v.getX(), translation_v.getY(), translation_v.getZ());
#endif

	transform.setOrigin(translation_v);	//Set Translation

	/* Broadcast Transform */
	br.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(), "jaco_joint_3",
					"jaco_joint_3_offset"));
	/***************************************************/


	/**********************Joint_4**********************/
	/* Joint 4 Rotation */
	rotation_q.setValue(0, 0, 0, 0); //zero rotation
	rot_matrix.setValue(0, 0, 0, 0, 0, 0, 0, 0, 0);
	/* Joint 4 Rotation Matrix */
	/*******					*******
	 * 0			0			   -1 *
	 * sin(q4)		cos(q4)			0 *
	 * cos(q4)		-sin(q4)	    0 *
	 *******				    *******/
	rot_matrix.setValue(0,0,-1, sin(q4), cos(q4), 0, cos(q4), -sin(q4), 0);

	rot_matrix.getRotation(rotation_q);

#ifdef PRINT_DEBUG_INFO
	/* Display Results */
	ROS_INFO(
			"Joint 4 Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());
#endif
	transform.setRotation(rotation_q); //Set Rotation

	/* Joint 4 Translation */
	translation_v.setValue(0, 0, 0); //zero translation
	//get joint 4 translation vector

	/* Joint 4 Translation Vector */
	/****     ****
	 * cos(q2)	 *
	 * sin(q2)	 *
	 * D2        *
	 ****     ****/
	translation_v.setValue(this->J3ToJ4(), 0, 0);

#ifdef PRINT_DEBUG_INFO

	/* Display Results */
	ROS_INFO(
			"Joint 4 Translation: X = %f, Y = %f, Z = %f", translation_v.getX(), translation_v.getY(), translation_v.getZ());
#endif

	transform.setOrigin(translation_v);	//Set Translation

	/* Broadcast Transform */
	br.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(), "jaco_joint_3_offset",
					"jaco_joint_4"));
	/***************************************************/

	/**********************Joint_5**********************/
	/* Joint 5 Rotation */
	rotation_q.setValue(0, 0, 0, 0); //zero rotation
	rot_matrix.setValue(0, 0, 0, 0, 0, 0, 0, 0, 0);

	/* Joint 5 Rotation Matrix */
	/*******					                                        *******
	 * cos(-35))*cos(q5)		cos(-35)*-sin(q5)		sin(-35) *
	 * sin(q5)						cos(q5)						0             *
	 * -sin(-35)*cos(q5)		sin(-35)*sin(q5)		cos(-35) *
	 *******				   										    *******/
	rot_matrix.setValue(cos((this->deg_to_rad(-55)))*cos(q5),cos((this->deg_to_rad(-55)))*-sin(q5),sin((this->deg_to_rad(-55))), sin(q5), cos(q5), 0, -sin((this->deg_to_rad(-55)))*cos(q5), sin((this->deg_to_rad(-55)))*sin(q5), cos((this->deg_to_rad(-55))));

	rot_matrix.getRotation(rotation_q);

#ifdef PRINT_DEBUG_INFO
	/* Display Results */
	ROS_INFO(
			"Joint 5 Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());
#endif
	transform.setRotation(rotation_q); //Set Rotation

	/* Joint 5 Translation */
	translation_v.setValue(0, 0, 0); //zero translation
	//get joint 5 translation vector

	/* Joint 5 Translation Vector */
	/****     ****
	 * cos(55)*D4	 *
	 * 0      *
	 * -sin(55)*D4      *
	 ****     ****/
	translation_v.setValue(cos(this->deg_to_rad(55))*this->J4ToJ5(), 0, -sin(this->deg_to_rad(55))*this->J4ToJ5());

#ifdef PRINT_DEBUG_INFO

	/* Display Results */
	ROS_INFO(
			"Joint 5 Translation: X = %f, Y = %f, Z = %f", translation_v.getX(), translation_v.getY(), translation_v.getZ());
#endif

	transform.setOrigin(translation_v);	//Set Translation

	/* Broadcast Transform */
	br.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(), "jaco_joint_4",
					"jaco_joint_5"));
	/***************************************************/


	/**********************Joint_6**********************/
	/* Joint 6 Rotation */
	rotation_q.setValue(0, 0, 0, 0); //zero rotation
	rot_matrix.setValue(0, 0, 0, 0, 0, 0, 0, 0, 0);

	/* Joint 6 Rotation Matrix */
	/*******					                                        *******
	 * cos(-35))*cos(q6)		cos(-35)*-sin(q6)		sin(-35) *
	 * sin(q6)						cos(q6)						0             *
	 * -sin(-35)*cos(q6)		sin(-35)*sin(q6)		cos(-35) *
	 *******				   										    *******/
	rot_matrix.setValue(cos((this->deg_to_rad(55)))*cos(q6),cos((this->deg_to_rad(55)))*-sin(q6),sin((this->deg_to_rad(55))), sin(q6), cos(q6), 0, -sin((this->deg_to_rad(55)))*cos(q6), sin((this->deg_to_rad(55)))*sin(q6), cos((this->deg_to_rad(55))));

	rot_matrix.getRotation(rotation_q);

#ifdef PRINT_DEBUG_INFO
	/* Display Results */
	ROS_INFO(
			"Joint 6 Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());
#endif
	transform.setRotation(rotation_q); //Set Rotation

	/* Joint 6 Translation */
	translation_v.setValue(0, 0, 0); //zero translation
	//get joint 6 translation vector

	/* Joint 6 Translation Vector */
	/****     ****
	 * -cos(55)*D5	 *
	 * 0	 *
	 * -sin(55)*D5       *
	 ****     ****/
	translation_v.setValue(-cos(this->deg_to_rad(55))*this->J5ToJ6(), 0, -sin(this->deg_to_rad(55))*this->J5ToJ6());

#ifdef PRINT_DEBUG_INFO

	/* Display Results */
	ROS_INFO(
			"Joint 6 Translation: X = %f, Y = %f, Z = %f", translation_v.getX(), translation_v.getY(), translation_v.getZ());
#endif

	transform.setOrigin(translation_v);	//Set Translation

	/* Broadcast Transform */
	br.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(), "jaco_joint_5",
					"jaco_joint_6"));
	/***************************************************/

	/**********************Joint_6**********************/
	/* Joint 6 Rotation */
	rotation_q.setValue(0, 0, 0, 0); //zero rotation
	rot_matrix.setValue(0, 0, 0, 0, 0, 0, 0, 0, 0);

	/* Joint 6 Rotation Matrix */
	/*******					                                        *******
	 * cos(-35))*cos(q6)		cos(-35)*-sin(q6)		sin(-35) *
	 * sin(q6)						cos(q6)						0             *
	 * -sin(-35)*cos(q6)		sin(-35)*sin(q6)		cos(-35) *
	 *******				   										    *******/
	rot_matrix.setValue(1,0,0,0,1,0,0,0,1);

	rot_matrix.getRotation(rotation_q);

#ifdef PRINT_DEBUG_INFO
	/* Display Results */
	ROS_INFO(
			"Joint 6 Rotation: X = %f, Y = %f, Z = %f, W = %f", rotation_q.getX(), rotation_q.getY(), rotation_q.getZ(), rotation_q.getW());
#endif
	transform.setRotation(rotation_q); //Set Rotation

	/* Joint 6 Translation */
	translation_v.setValue(0, 0, 0); //zero translation
	//get joint 6 translation vector

	/* Joint 6 Translation Vector */
	/****     ****
	 * -cos(55)*D5	 *
	 * 0	 *
	 * -sin(55)*D5       *
	 ****     ****/
	translation_v.setValue(0,0,-this->J6ToEnd());

#ifdef PRINT_DEBUG_INFO

	/* Display Results */
	ROS_INFO(
			"Joint 6 Translation: X = %f, Y = %f, Z = %f", translation_v.getX(), translation_v.getY(), translation_v.getZ());
#endif

	transform.setOrigin(translation_v);	//Set Translation

	/* Broadcast Transform */
	br.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(), "jaco_joint_6",
					"jaco_end_effector"));
	/***************************************************/

}
}
