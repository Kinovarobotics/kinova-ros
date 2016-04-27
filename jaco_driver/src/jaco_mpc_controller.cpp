#include "jaco_driver/jaco_mpc_controller.h"

#include <kinova/KinovaTypes.h>

#include <geometry_msgs/Pose.h>

#include "jaco_driver/jaco_types.h"
#include "jaco_driver/jaco_ik_solver.h"
#include "jaco_driver/jaco_qp_solver.h"

#include <math.h>


using namespace Eigen;
using namespace std;

namespace jaco

{
	JacoMPCController::JacoMPCController(JacoComm &arm_comm, JacoIKSolver &ik_solver, JacoQPSolver &qp_solver, const ros::NodeHandle &nh)
		: node_handle_(nh), arm_comm_(arm_comm), ik_solver_(ik_solver), qp_solver_(qp_solver)
	{
		ros::NodeHandle pn("~");

		node_handle_.param<int>("qp_nWSR", nWSR_, 100);
		node_handle_.param<int>("qp_variables", variables_, 6);
		node_handle_.param<int>("qp_constraints", constraints_, 6);
		node_handle_.param<int>("qp_horizon", horizon_, 5);

		node_handle_.param<double>("qp_b_factor", b_factor_, 0.015);
		node_handle_.param<double>("qp_q_factor", q_factor_, 1);
		node_handle_.param<double>("qp_s_factor", s_factor_, 1);
		node_handle_.param<double>("qp_z_factor", z_factor_, 100);	
		node_handle_.param<double>("cpu_time", cputime_, 0.008);
		
		node_handle_.param<double>("status_interval_seconds", watchdog_interval_seconds, 0.015);
		node_handle_.param<string>("urdf", urdf_, "jaco2.urdf");
		node_handle_.param<string>("root_name", root_name_, "jaco_link_base");
		node_handle_.param<string>("tip_name", tip_name_, "jaco_link_6");
		node_handle_.param<string>("error", err_, "");
		
		node_handle_.param<double>("stop_threshold", thresh_, 10);
		max_iter_ = 10;
		initialized_mpc_ = false;
		end_point_init_ = false;
		exe_enable_ = false; 
		update_step_ = true;
		printresult_ = false;
		
		if(qp_solver_.initSolver(variables_, constraints_, horizon_, nWSR_)){
			ROS_INFO("QP Solver is ready to use");
		}	
		
		if( ik_solver_.initFromURDF(urdf_, root_name_, tip_name_, max_iter_, err_)){
			ROS_INFO("IK Solver is ready for use");
		}
		
		A_ = MatrixXd::Identity(variables_,variables_) ;
		B_ = b_factor_ * MatrixXd::Identity(variables_,variables_) ;
		B_ex_ = MatrixXd::Zero(variables_*horizon_,variables_) ;
		Q_ = q_factor_ * MatrixXd::Identity(variables_,variables_) ;
		S_ = s_factor_ * MatrixXd::Identity(variables_,variables_) ;
		Z_ = z_factor_ * MatrixXd::Identity(variables_,variables_) ;
		q0_ = MatrixXd::Zero(1,variables_) ;
		qd_ = MatrixXd::Zero(1,variables_*horizon_) ;
		qd_final_ = MatrixXd::Zero(1, variables_);
		e0_ = MatrixXd::Zero(1,variables_) ;
		lbG_init_ = -360*VectorXd::Ones(variables_*horizon_,1) ;
		ubG_init_ = 360*VectorXd::Ones(variables_*horizon_,1) ;
		lb_init_ = -10*VectorXd::Ones(variables_*horizon_,1) ;
		ub_init_ = 10*VectorXd::Ones(variables_*horizon_,1) ;
		
		H_unit1_ = z_factor_ * b_factor_ * b_factor_ * MatrixXd::Identity(variables_,variables_) ;
		H_unit2_ = s_factor_ * b_factor_ * b_factor_ * MatrixXd::Identity(variables_,variables_) ;
		g_unit1_ = 2.0 * z_factor_ * b_factor_ * MatrixXd::Identity(variables_,variables_) ;
		g_unit2_ = 2.0 * s_factor_ * b_factor_ * MatrixXd::Identity(variables_,variables_) ;

		mpc_solution_ = new double[variables_*horizon_];
		H_ = new double[variables_*horizon_*variables_*horizon_];
		g_ = new double[variables_*horizon_]; 
		G_ = new double[constraints_*horizon_*constraints_*horizon_];
		lb_ = new double[variables_*horizon_]; 
		ub_ = new double[variables_*horizon_]; 
		lbG_ = new double[constraints_*horizon_]; 
		ubG_ = new double[constraints_*horizon_];	
		
		t_count_ = 0;

        //husky_cmd_pub_ = node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

		watchdog_timer = node_handle_.createTimer(ros::Duration(watchdog_interval_seconds),
											   &JacoMPCController::watchdog, this);
		
		ROS_INFO("MPC Controller is Ready");
	}
    
    JacoMPCController::~JacoMPCController()
    {
		watchdog_timer.stop();
		delete[] mpc_solution_;
		delete[] H_;
		delete[] g_ ; 
		delete[] G_;
		delete[] lb_; 
		delete[] ub_ ; 
		delete[] lbG_; 
		delete[] ubG_;
	}
	
	bool JacoMPCController::initMPC()
	{
		lbG_update_ = VectorXd::Zero(variables_*horizon_,1) ;
		ubG_update_ = VectorXd::Zero(variables_*horizon_,1) ;	
		lb_update_ = VectorXd::Zero(variables_*horizon_,1) ;
		ub_update_ = VectorXd::Zero(variables_*horizon_,1) ;
		gg_update_ = VectorXd::Zero(variables_*horizon_,1) ;
		g_update_ = MatrixXd::Zero(1,variables_*horizon_) ;
		H_update_ = MatrixXd::Zero(variables_*horizon_,variables_*horizon_) ;
		G_update_ = MatrixXd::Zero(variables_*horizon_,variables_*horizon_) ;
		
		updateq0();
		updateqd(); 

		updateHgG(JOINT_MPC);
		updateulbG(JOINT_MPC);
		
		Map<VectorXd>(lbG_, horizon_*variables_, 1) =  lbG_update_;
		Map<VectorXd>(ubG_, horizon_*variables_, 1) =  ubG_update_;
		Map<VectorXd>(lb_, horizon_*variables_, 1) = lb_update_;
		Map<VectorXd>(ub_, horizon_*variables_, 1) = ub_update_;
		Map<Matrix<double, Dynamic, Dynamic, RowMajor> >(H_, horizon_*variables_*horizon_*variables_, 1) = H_update_;
		Map<VectorXd>(g_, horizon_*variables_, 1) =  gg_update_;
		Map<Matrix<double, Dynamic, Dynamic, RowMajor> >(G_, horizon_*constraints_*horizon_*constraints_, 1) =  G_update_;
		
		//~ ROS_INFO("g_: %f, %f, %f, %f, %f, %f.", g_[0], g_[1], g_[2], g_[3], g_[4], g_[5]);
		//~ ROS_INFO("g_: %f, %f, %f, %f, %f, %f.", g_[6], g_[7], g_[8], g_[9], g_[10], g_[11]);
		//~ ROS_INFO("g_: %f, %f, %f, %f, %f, %f.", g_[12], g_[13], g_[14], g_[15], g_[16], g_[17]);
		//~ ROS_INFO("g_: %f, %f, %f, %f, %f, %f.", g_[18], g_[19], g_[20], g_[21], g_[22], g_[23]);
		//~ ROS_INFO("g_: %f, %f, %f, %f, %f, %f.", g_[24], g_[25], g_[26], g_[27], g_[28], g_[29]);
		
		//~ ROS_INFO("lbG_: %f, %f, %f, %f, %f, %f.", lbG_[0], lbG_[1], lbG_[2], lbG_[3], lbG_[4], lbG_[5]);
		//~ ROS_INFO("lbG_: %f, %f, %f, %f, %f, %f.", lbG_[6], lbG_[7], lbG_[8], lbG_[9], lbG_[10], lbG_[11]);
		//~ ROS_INFO("lbG_: %f, %f, %f, %f, %f, %f.", lbG_[12], lbG_[13], lbG_[14], lbG_[15], lbG_[16], lbG_[17]);
		//~ ROS_INFO("lbG_: %f, %f, %f, %f, %f, %f.", lbG_[18], lbG_[19], lbG_[20], lbG_[21], lbG_[22], lbG_[23]);
		//~ ROS_INFO("lbG_: %f, %f, %f, %f, %f, %f.", lbG_[24], lbG_[25], lbG_[26], lbG_[27], lbG_[28], lbG_[29]);
		 //~ 
		//~ ROS_INFO("ub_: %f, %f, %f, %f, %f, %f.", ub_[0], ub_[1], ub_[2], ub_[3], ub_[4], ub_[5]);
		//~ ROS_INFO("ub_: %f, %f, %f, %f, %f, %f.", ub_[6], ub_[7], ub_[8], ub_[9], ub_[10], ub_[11]);
		//~ ROS_INFO("ub_: %f, %f, %f, %f, %f, %f.", ub_[12], ub_[13], ub_[14], ub_[15], ub_[16], ub_[17]);
		//~ ROS_INFO("ub_: %f, %f, %f, %f, %f, %f.", ub_[18], ub_[19], ub_[20], ub_[21], ub_[22], ub_[23]);
		//~ ROS_INFO("ub_: %f, %f, %f, %f, %f, %f.", ub_[24], ub_[25], ub_[26], ub_[27], ub_[28], ub_[29]);
		//~ 
		//~ ROS_INFO("lb_: %f, %f, %f, %f, %f, %f.", lb_[0], lb_[1], lb_[2], lb_[3], lb_[4], lb_[5]);
		//~ ROS_INFO("lb_: %f, %f, %f, %f, %f, %f.", lb_[6], lb_[7], lb_[8], lb_[9], lb_[10], lb_[11]);
		//~ ROS_INFO("lb_: %f, %f, %f, %f, %f, %f.", lb_[12], lb_[13], lb_[14], lb_[15], lb_[16], lb_[17]);
		//~ ROS_INFO("lb_: %f, %f, %f, %f, %f, %f.", lb_[18], lb_[19], lb_[20], lb_[21], lb_[22], lb_[23]);
		//~ ROS_INFO("lb_: %f, %f, %f, %f, %f, %f.", lb_[24], lb_[25], lb_[26], lb_[27], lb_[28], lb_[29]);
		//~ 
		//~ ROS_INFO("ubG_: %f, %f, %f, %f, %f, %f.", ubG_[0], ubG_[1], ubG_[2], ubG_[3], ubG_[4], ubG_[5]);
		//~ ROS_INFO("ubG_: %f, %f, %f, %f, %f, %f.", ubG_[6], ubG_[7], ubG_[8], ubG_[9], ubG_[10], ubG_[11]);
		//~ ROS_INFO("ubG_: %f, %f, %f, %f, %f, %f.", ubG_[12], ubG_[13], ubG_[14], ubG_[15], ubG_[16], ubG_[17]);
		//~ ROS_INFO("ubG_: %f, %f, %f, %f, %f, %f.", ubG_[18], ubG_[19], ubG_[20], ubG_[21], ubG_[22], ubG_[23]);
		//~ ROS_INFO("ubG_: %f, %f, %f, %f, %f, %f.", ubG_[24], ubG_[25], ubG_[26], ubG_[27], ubG_[28], ubG_[29]);
		//~ 
		//~ ROS_INFO("H_: %f, %f, %f, %f, %f, %f.", H_[0], H_[31], H_[62], H_[93], H_[124], H_[155]);
		//~ ROS_INFO("H_: %f, %f, %f, %f, %f, %f.", H_[186], H_[217], H_[248], H_[279], H_[310], H_[341]);
		//~ ROS_INFO("H_: %f, %f, %f, %f, %f, %f.", H_[372], H_[403], H_[434], H_[465], H_[496], H_[527]);
		//~ ROS_INFO("H_: %f, %f, %f, %f, %f, %f.", H_[558], H_[589], H_[620], H_[651], H_[682], H_[713]);
		//~ ROS_INFO("H_: %f, %f, %f, %f, %f, %f.", H_[744], H_[775], H_[806], H_[837], H_[868], H_[899]);
		//~ 
		return qp_solver_.computeJointOpt(H_, g_, G_, lb_, ub_, lbG_, ubG_);
	}
		

	bool JacoMPCController::updateMPC()
	{
		lbG_update_ = VectorXd::Zero(variables_*horizon_,1) ;
		ubG_update_ = VectorXd::Zero(variables_*horizon_,1) ;	
		gg_update_ = VectorXd::Zero(variables_*horizon_,1) ;
		g_update_ = MatrixXd::Zero(1,variables_*horizon_) ;
		G_update_ = MatrixXd::Zero(variables_*horizon_,variables_*horizon_) ;
		
		updateq0();
		updateqd(); 
		 
		updateHgG(JOINT_MPC);
		updateulbG(JOINT_MPC);
		
		Map<VectorXd>(lbG_, horizon_*variables_, 1) =  lbG_update_;
		Map<VectorXd>(ubG_, horizon_*variables_, 1) =  ubG_update_;
		Map<VectorXd>(g_, horizon_*variables_, 1) =  gg_update_;
		Map<Matrix<double, Dynamic, Dynamic, RowMajor> >(G_, horizon_*constraints_*horizon_*constraints_, 1) =  G_update_;
		
		return qp_solver_.computeJointOpt(H_, g_, G_, lb_, ub_, lbG_, ubG_);
	}
			
	void JacoMPCController::updateqd()
	{
		// Baby version for test
		if (!end_point_init_){
			geometry_msgs::Pose p_in, ee_pos;
			
			//~ p_in.position.x = 0.0;
			//~ p_in.position.y = -0.5;
			//~ p_in.position.z = 0.0;
			//~ p_in.orientation.x = 0.17;
			//~ p_in.orientation.y = 0.765;
			//~ p_in.orientation.z = -0.3;
			//~ p_in.orientation.w = 0.54;
			
			// Just test 	
			ee_pos = ik_solver_.jointsToCartesian(current_angle_);

			p_in.position.x = ee_pos.position.x;
			p_in.position.y = ee_pos.position.y;
			p_in.position.z = ee_pos.position.z + 0.1;
			p_in.orientation.x = ee_pos.orientation.x;
			p_in.orientation.y = ee_pos.orientation.y;
			p_in.orientation.z = ee_pos.orientation.z;
			p_in.orientation.w = ee_pos.orientation.w;

			qd_final_ = ik_solver_.desiredAngles(p_in, current_angle_);
			
			end_point_init_ = true;
			exe_enable_ = true;
		}
		//~ if (update_step_){
			//~ MatrixXd step = MatrixXd::Zero(1, variables_);
			//~ step = (qd_final_ - q0_)/horizon_;
			//~ 
			//~ for (int i = 0; i < horizon_ ; i++){
				//~ qd_.block(0,i*variables_, 1, variables_) = q0_ + (i+1)*step;
			//~ }
			//~ exe_enable_ = true; 
			//~ update_step_ = false;
			//~ t_count_++;
			//~ ROS_INFO("T_COUNT_: %i", t_count_);
		//~ }
		//~ else if (qd_.block(0,0, 1, variables_) != qd_.block(0,(horizon_ - 1)*variables_, 1, variables_)){
			//~ for (int i = 0; i < horizon_ - 1; i++){
				//~ qd_.block(0,i*variables_, 1, variables_) = qd_.block(0,(i+1)*variables_, 1, variables_);
			//~ }
		//~ }
		//~ else if (checkResult(thresh_) || t_count_ >= 1000){
			//~ exe_enable_ = false;
			//~ t_count_ = 0;
		//~ }
		//~ else{
			//~ //update_step_ = true;	
			//~ exe_enable_ = false;	
		//~ }
		ROS_INFO("qd_final_: %f, %f, %f, %f, %f, %f", qd_final_(0,0),qd_final_(0,1), qd_final_(0,2), qd_final_(0,3), qd_final_(0,4), qd_final_(0,5));
		ROS_INFO("e0_: %f, %f, %f, %f, %f, %f", e0_(0,0),e0_(0,1), e0_(0,2), e0_(0,3), e0_(0,4), e0_(0,5));
		//ROS_INFO("qd_: %f, %f, %f, %f, %f, %f", qd_(0,24),qd_(0,25), qd_(0,26), qd_(0,27), qd_(0,28), qd_(0,29));

		e0_ = -qd_final_ + q0_;
	}
	
	void JacoMPCController::updateq0()
	{
		arm_comm_.getJointAngles(current_angle_);
		q0_(0) = saturate( current_angle_.Actuator1 - 180.0 ); // * DTR;
		q0_(1) = saturate( current_angle_.Actuator2 - 270.0 ); // * DTR;
		q0_(2) = saturate( current_angle_.Actuator3 - 90.0  ); // * DTR;
		q0_(3) = saturate( current_angle_.Actuator4 - 180.0 ); // * DTR;
		q0_(4) = saturate( current_angle_.Actuator5 - 180.0 ); // * DTR;
		q0_(5) = saturate( current_angle_.Actuator6 - 270.0 ); // * DTR;
		
		ROS_INFO("q0_: %f, %f, %f, %f, %f, %f", q0_(0,0), q0_(0,1), q0_(0,2), q0_(0,3), q0_(0,4), q0_(0,5));
	}
	
	void JacoMPCController::updateHgG(int md)
	{
		switch (md)
		{
			case JOINT_MPC: 
			
			// Update Hessian Matrix
			for (int i = horizon_-1; i >= 0 ; i--){
				MatrixXd H_col = MatrixXd::Zero(variables_*horizon_,variables_);
				for (int j = 0; j <= i ; j++){
					if (i == j){
						H_col.block(j*variables_, 0, variables_,variables_) = H_unit1_ + (horizon_-i-1)*H_unit2_ + Q_;
					} 
					else {
						H_col.block(j*variables_, 0, variables_,variables_) = H_unit1_ + (horizon_-i-1)*H_unit2_;
					}
				}
				//ROS_INFO("H_col_: %f, %f, %f, %f, %f, %f.", H_col(6,0), H_col(7,1), H_col(8,2), H_col(9,3), H_col(10,4), H_col(11,5));
				H_update_.block(0,i*variables_,variables_*horizon_,variables_) = H_col;
			}
			
			for (int i = 1; i < horizon_ ; i++){
				for (int j = i-1; j < horizon_-1 ; j++){
					H_update_.block(i*variables_, j*variables_, variables_,variables_) = H_update_.block(j*variables_, i*variables_, variables_,variables_);
				}
			}
			//ROS_INFO("H_check_: %f, %f, %f, %f, %f, %f.", H_update_(0,0), H_update_(1,1), H_update_(2,2), H_update_(3,3), H_update_(4,4), H_update_(5,5));
			// Update g vector
			g_unit = MatrixXd::Zero(1,variables_);
			for (int i = horizon_-1; i >= 0 ; i--){
				if (i == horizon_-1){
					//~ g_unit = q0_ * g_unit1_ - qd_.block(0, i*variables_, 1, variables_)*g_unit1_;
				//~ }
				//~ else {
					//~ g_unit += q0_ * g_unit2_ - qd_.block(0, i*variables_, 1, variables_)*g_unit2_;
				//~ }
					g_unit = e0_ * g_unit1_;
				}
				else {
					g_unit += e0_ * g_unit2_;
				}
				g_update_.block(0, i*variables_, 1, variables_) = g_unit;
			}
			//~ ROS_INFO("g_update_: %f, %f, %f, %f, %f, %f.", g_update_(0,0), g_update_(0,1), g_update_(0,2), g_update_(0,3), g_update_(0,4), g_update_(0,5));
			//~ ROS_INFO("g_update_: %f, %f, %f, %f, %f, %f.", g_update_(0,6), g_update_(0,7), g_update_(0,8), g_update_(0,9), g_update_(0,10), g_update_(0,11));
			//~ ROS_INFO("g_update_: %f, %f, %f, %f, %f, %f.", g_update_(0,12), g_update_(0,13), g_update_(0,14), g_update_(0,15), g_update_(0,16), g_update_(0,17));
			//~ ROS_INFO("g_update_: %f, %f, %f, %f, %f, %f.", g_update_(0,18), g_update_(0,19), g_update_(0,20), g_update_(0,21), g_update_(0,22), g_update_(0,23));
			//~ ROS_INFO("g_update_: %f, %f, %f, %f, %f, %f.", g_update_(0,24), g_update_(0,25), g_update_(0,26), g_update_(0,27), g_update_(0,28), g_update_(0,29));
			gg_update_ = g_update_.transpose();
			
			//Update G
			for (int i = 0; i < horizon_; i++){
				for (int j = 0; j <= i; j++){
					G_update_.block(i*constraints_, j*constraints_, constraints_, constraints_) = B_;
				}
			}
			
			break;
			
			
			case CARTISIAN_MPC:
			// Warning: Underconstruction!!!!!!!!!!!!!!!!!!!!

			break;

		}
		

	}
	
	
	void JacoMPCController::updateulbG(int md)
	{
		for (int i = 0; i < horizon_; i++){
			lbG_update_.block(i*constraints_,0,constraints_,1) = lbG_init_ - e0_.transpose(); //q0_.transpose();
			ubG_update_.block(i*constraints_,0,constraints_,1) = ubG_init_ - e0_.transpose(); //q0_.transpose();
			lb_update_.block(i*constraints_,0,constraints_,1) = lb_init_;
			ub_update_.block(i*constraints_,0,constraints_,1) = ub_init_;
		}
	}
	
	double JacoMPCController::saturate(double input)
	{
		while (input > 180 || input < -180){
			if (input > 180) input -=180;
			else if (input < -180) input+=180;
		}
		return input;
	}
	
	void JacoMPCController::watchdog(const ros::TimerEvent&)
	{
		if (checkResult(thresh_)){
			exe_enable_ = false;
			printresult_ = true;
		}
		start_time_ = ros::Time().now();
		if (exe_enable_){
			action(mpc_solution_);
		}
		elipsed_time_ = ros::Time().now().toSec() - start_time_.toSec();
		ROS_INFO("ELIPSED TIME: %f", elipsed_time_);
		bool success;
		if (!initialized_mpc_) {
			success = initMPC();
			if (success) {
				mpc_solution_ = qp_solver_.getOptimalSolution();
				initialized_mpc_ = true;
			}
		}
		else if (exe_enable_) {
			success = updateMPC();
			if (success) {
				mpc_solution_ = qp_solver_.getOptimalSolution();
				//~ ROS_INFO("OPTIMAL SOLUTION IS READY: %f, %f, %f, %f, %f, %f", mpc_solution_[0], mpc_solution_[1], mpc_solution_[2], mpc_solution_[3], mpc_solution_[4],  mpc_solution_[5] );
				//~ ROS_INFO("OPTIMAL SOLUTION IS READY: %f, %f, %f, %f, %f, %f", mpc_solution_[6], mpc_solution_[7], mpc_solution_[8], mpc_solution_[9], mpc_solution_[10],  mpc_solution_[11] );
				//~ ROS_INFO("OPTIMAL SOLUTION IS READY: %f, %f, %f, %f, %f, %f", mpc_solution_[12], mpc_solution_[13], mpc_solution_[14], mpc_solution_[15], mpc_solution_[16],  mpc_solution_[17] );
				//~ ROS_INFO("OPTIMAL SOLUTION IS READY: %f, %f, %f, %f, %f, %f", mpc_solution_[18], mpc_solution_[19], mpc_solution_[20], mpc_solution_[21], mpc_solution_[22],  mpc_solution_[23] );
				//~ ROS_INFO("OPTIMAL SOLUTION IS READY: %f, %f, %f, %f, %f, %f", mpc_solution_[24], mpc_solution_[25], mpc_solution_[26], mpc_solution_[27], mpc_solution_[28],  mpc_solution_[29] );
			}
		}
		else if (printresult_)  {
			printoutResult();
			printresult_ = false;
		}
		
	}
	
	void JacoMPCController::action(double *opt_sol)
	{
		joint_velocities_.Actuator1 = opt_sol[0] * RTD;
		joint_velocities_.Actuator2 = opt_sol[1] * RTD;
		joint_velocities_.Actuator3 = opt_sol[2] * RTD;
		joint_velocities_.Actuator4 = opt_sol[3] * RTD;
		joint_velocities_.Actuator5 = opt_sol[4] * RTD;
		joint_velocities_.Actuator6 = opt_sol[5] * RTD;
        
		arm_comm_.setJointVelocities(joint_velocities_);
	}
	
	void JacoMPCController::printoutResult()
	{
		arm_comm_.getJointAngles(current_angle_);
		q0_(0) = saturate( current_angle_.Actuator1 - 180.0 ); // * DTR;
		q0_(1) = saturate( current_angle_.Actuator2 - 270.0 ); // * DTR;
		q0_(2) = saturate( current_angle_.Actuator3 - 90.0  ); // * DTR;
		q0_(3) = saturate( current_angle_.Actuator4 - 180.0 ); // * DTR;
		q0_(4) = saturate( current_angle_.Actuator5 - 180.0 ); // * DTR;
		q0_(5) = saturate( current_angle_.Actuator6 - 270.0 ); // * DTR;
		ROS_INFO("FINAL RESULT: %f, %f, %f, %f, %f, %f", q0_(0)-qd_final_(0), q0_(1)-qd_final_(1), q0_(2)-qd_final_(2), q0_(3)-qd_final_(3), q0_(4)-qd_final_(4), q0_(5)-qd_final_(5));
	}
	
	bool JacoMPCController::checkResult(double thresh)
	{
		if (fabs(qd_final_(0,0)-q0_(0,0)) < thresh && fabs(qd_final_(0,1)-q0_(0,1))<thresh && fabs(qd_final_(0,2)-q0_(0,2))<thresh && fabs(qd_final_(0,3)-q0_(0,3))<thresh && fabs(qd_final_(0,4)-q0_(0,4))<thresh && fabs(qd_final_(0,5)-q0_(0,5))<thresh ) {
			return true;
		}
		else {
			return false;
		}
	}
}
