#ifndef JACO_DRIVER_JACO_KINEMATIC_CONTROLLER_H_
#define JACO_DRIVER_JACO_KINEMATIC_CONTROLLER_H_

#include <ros/time.h>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <boost/thread/thread.hpp>
#include <geometry_msgs/Twist.h>

#include "jaco_driver/jaco_comm.h"
#include "jaco_driver/jaco_ik_solver.h"

#define DTR 0.0174532925
#define RTD 57.295779513

using namespace Eigen;

namespace jaco
{
	
	class JacoKinematicController
	{		
		public:
		
			JacoKinematicController(JacoComm &, JacoIKSolver &, const ros::NodeHandle &n);
			~JacoKinematicController();
						
			void callBoth(CartesianInfo);
			void callBothNullSpace(AngularInfo);
			void call(CartesianInfo);	
			void callNullSpace(AngularInfo);

			
		private:
			int mode;
			ros::NodeHandle node_handle_;
			JacoComm &arm_comm_;
			JacoIKSolver ik_solver_;
			
			ros::Publisher manip_pub_;
			ros::Publisher husky_cmd_pub_;
			
			void watchdog(const ros::TimerEvent&);
			ros::Timer watchdog_timer;
			double watchdog_interval_seconds;
			
			ros::Time start_time;
			double elipsed_time;
			
			void updateConfig(int);
			
			MatrixXf calcPseudoJacobian(MatrixXf);
			MatrixXf calcNullSpacePseudoJacobian(MatrixXf);
			MatrixXf calcSingularityAwarePseudoJacobian(MatrixXf, int);
			MatrixXf calcPJacobian(MatrixXf);
			VectorXf calcdQ(MatrixXf, VectorXf, int);
			VectorXf calcexdQ(MatrixXf, VectorXf, int);

			void action(VectorXf);
			void actionAll(VectorXf);
			
			VectorXf vGoal;

			VectorXf dQ;
			VectorXf nQ;
			VectorXf exdQ;
			VectorXf exnQ;

			MatrixXf mJaco;
			MatrixXf nJaco;
			MatrixXf mPJaco;
			MatrixXf nPJaco;
			MatrixXf exJaco;
			MatrixXf exPJaco;

			bool has_active_goal;	
			
			ros::Time last_cmd_time;
			ros::Time last_time;
			ros::Time elapsed_time;
			
			double timeout_seconds;
			double default_timeout_seconds;
			AngularInfo joint_velocities_;
			
			std::string urdf_; 
			std::string root_name_;
            std::string tip_name_; 
            unsigned int max_iter_;
            std::string err_;

            JacoAngles current_angle;
            KDL::Jacobian j_;

	};
}

#endif // JACO_DRIVER_JACO_KINEMATIC_CONTROLLER_H
