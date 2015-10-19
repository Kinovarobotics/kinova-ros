#ifndef JACO_DRIVER_JACO_KINEMATIC_CONTROLLER_H_
#define JACO_DRIVER_JACO_KINEMATIC_CONTROLLER_H_

#include <ros/time.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <sensor_msgs/ChannelFloat32.h>
#include <boost/thread/thread.hpp>

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
						
			void call(CartesianInfo);	
			void callForceManip(CartesianInfo);
			void callMotionManip(CartesianInfo);
			void callNullSpace(AngularInfo);
			
		private:
			int mode;
			ros::NodeHandle node_handle_;
			JacoComm &arm_comm_;
			JacoIKSolver ik_solver_;
			
			ros::Publisher manip_pub_;
			
			void watchdog(const ros::TimerEvent&);
			ros::Timer watchdog_timer;
			double watchdog_interval_seconds;
			
			ros::Time start_time;
			double elipsed_time;
			
			void updateConfig(int);
			
			
			MatrixXf calcPseudoJacobian(MatrixXf);
			MatrixXf calcNullSpacePseudoJacobian(MatrixXf);
			MatrixXf calcSingularityAwarePseudoJacobian(MatrixXf, int);
			VectorXf calcdQ(MatrixXf, VectorXf, int);
			VectorXf calcdQnQ(MatrixXf, VectorXf, VectorXf);
			void derivativeJaco(VectorXf, float);
			void maxForceManipulability(VectorXf, int);
			void maxMotionManipulability(VectorXf, int);
			VectorXf linearRegression(VectorXf, int);
			
			MatrixXf getJacobian();
			MatrixXf getPseudoJacobian();
			void action(VectorXf);
			
			VectorXf vGoal;
			VectorXf vGoalTest;
			VectorXf pGoal;
			VectorXf dQ;
			VectorXf nQ;

			MatrixXf mJaco;
			MatrixXf nJaco;
			MatrixXf nNewJaco;
			MatrixXf mPJaco;
			MatrixXf nPJaco;
			MatrixXf nPNewJaco;
			MatrixXf dJaco;
			VectorXf Jq;
			
			float mManip, fManip;
           		sensor_msgs::ChannelFloat32 manip_;			
            
			bool has_active_goal;	
			
			ros::Time last_cmd_time;
			ros::Time last_time;
			ros::Time elapsed_time;
			
			double timeout_seconds;
			double default_timeout_seconds;
			AngularInfo joint_velocities_;
			JacoPose ideal_current_pose;
			
			std::string urdf_; 
			std::string root_name_;
            std::string tip_name_; 
            unsigned int max_iter_;
            std::string err_;
            

            
            JacoAngles current_angle;
            KDL::Jacobian j_, j_new_;

	};
}

#endif // JACO_DRIVER_JACO_KINEMATIC_CONTROLLER_H
