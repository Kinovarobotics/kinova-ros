#ifndef JACO_DRIVER_JACO_MPC_CONTROLLER_H_
#define JACO_DRIVER_JACO_MPC_CONTROLLER_H_

#include <ros/ros.h>
#include <Eigen/Dense>

#include <boost/thread/thread.hpp>

#include "jaco_driver/jaco_comm.h"
#include "jaco_driver/jaco_qp_solver.h"
#include "jaco_driver/jaco_ik_solver.h"

#define DTR 0.0174532925
#define RTD 57.295779513

#define CARTISIAN_MPC 2
#define JOINT_MPC 1

namespace jaco
{
	
	class JacoMPCController
	{		
		public:
		
			JacoMPCController(JacoComm &, JacoIKSolver &, JacoQPSolver &, const ros::NodeHandle &n);
			~JacoMPCController();
						

			bool initMPC();
			bool updateMPC();	
			
			void updateqd();
			void updateq0();
			void updateHgG(int);
			void updateulbG(int);
			void updateB();
			
			double saturate(double);
			void printoutResult();
			bool checkResult(double);
			
			
		private:
			int mode;
			ros::NodeHandle node_handle_;
			JacoComm &arm_comm_;
			JacoIKSolver ik_solver_;
			JacoQPSolver qp_solver_;

			
			void watchdog(const ros::TimerEvent&);
			ros::Timer watchdog_timer;
			double watchdog_interval_seconds;
			
			ros::Time start_time_;
			double elipsed_time_;
			
			
			Eigen::MatrixXd A_;
			Eigen::MatrixXd B_;
			Eigen::MatrixXd B_ex_;
			Eigen::MatrixXd Q_;
			Eigen::MatrixXd S_;
			Eigen::MatrixXd Z_;
			Eigen::MatrixXd q0_;
			Eigen::MatrixXd qd_;
			Eigen::MatrixXd e0_;
			Eigen::VectorXd lbG_init_;
			Eigen::VectorXd ubG_init_;
			Eigen::VectorXd lb_init_;
			Eigen::VectorXd ub_init_;
			Eigen::VectorXd lbG_update_;
			Eigen::VectorXd ubG_update_;
			Eigen::VectorXd lb_update_;
			Eigen::VectorXd ub_update_;
			Eigen::MatrixXd g_update_;
			Eigen::VectorXd gg_update_;
			Eigen::MatrixXd H_update_;
			Eigen::MatrixXd G_update_;
			Eigen::MatrixXd H_unit1_;
			Eigen::MatrixXd H_unit2_;
			Eigen::MatrixXd g_unit1_;
			Eigen::MatrixXd g_unit2_;
			Eigen::MatrixXd g_unit;
			
			Eigen::MatrixXd qd_final_;
			
			double cputime_, b_factor_, q_factor_, s_factor_, z_factor_, thresh_;
			double *H_, *g_, *G_, *lb_, *ub_, *lbG_, *ubG_;
			double *mpc_solution_;
			
			int time_index_;

			void action(double *);
			
			int nWSR_, variables_, constraints_, horizon_, t_count_;
			bool initialized_mpc_;

			int mode_;	
			bool end_point_init_, exe_enable_, update_step_, printresult_;
			
			ros::Time last_cmd_time;
			ros::Time last_time;
			ros::Time elapsed_time;
			
			double timeout_seconds;
			double default_timeout_seconds;
			
			std::string urdf_; 
			std::string root_name_;
            std::string tip_name_; 
            unsigned int max_iter_;
            std::string err_;

            JacoAngles current_angle_;
            KDL::Jacobian j_;
            AngularInfo joint_velocities_;

	};
}

#endif // JACO_DRIVER_JACO_MPC_CONTROLLER_H
