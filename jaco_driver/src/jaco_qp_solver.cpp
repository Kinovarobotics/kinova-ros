#include "jaco_driver/jaco_qp_solver.h"

#include <ros/ros.h>


#define DTR 0.0174532925
#define RTD 57.295779513


namespace jaco
{
	JacoQPSolver::JacoQPSolver()
	{
		variables_ = 0;
		constraints_ = 0;
		horizon_ = 0;
		allocated_ = false;
		
	}

// ----------------------------------------------------------------------------------------------------

	JacoQPSolver::~JacoQPSolver()
	{
		if(allocated_){
			delete jnt_solver_;
			delete cart_solver_;
			delete[] optimal_solution_;
		}
	}

// ----------------------------------------------------------------------------------------------------
	bool JacoQPSolver::initSolver(int variables, int constraints, int horizon, int nWSR)
	{ 
		allocated_ = true;
		variables_ = variables;
		constraints_ = constraints;
		horizon_ = horizon;
		nWSR_ = nWSR;
		qp_init_ = false;
			
		if ( variables_ == 0 || constraints_ == 0 || horizon_ == 0 || nWSR_ == 0 ){ 
			return false;
		}
			
		jnt_solver_ = new qpOASES::QProblem(variables_ * horizon_, constraints_ * horizon_);
		cart_solver_ = new qpOASES::SQProblem(variables_ * horizon_, constraints_ * horizon_);
			
		// Option setting
		qpOASES::Options myOptions;
		myOptions.setToReliable();
		myOptions.printLevel = qpOASES::PL_LOW;
		//myOptions.setToMPC();

		jnt_solver_->setOptions(myOptions);
		cart_solver_->setOptions(myOptions);
			
		optimal_solution_ = new double[variables_*horizon_];
		
		return true;
	}

	bool JacoQPSolver::computeCartOpt(double *H, double *g, double *G, double *lb, double *ub, double *lbG, double *ubG, double cputime)
	{
		int nWSR = nWSR_;
		double cpu_time = cputime;

		qpOASES::returnValue retval;
		if (!qp_init_) {
			retval = cart_solver_->init(H, g, G, lb, ub, lbG, ubG, nWSR, &cpu_time);
				if (retval == qpOASES::SUCCESSFUL_RETURN) {
				ROS_INFO("qpOASES problem successfully initialized.");
				qp_init_ = true;
			}
		}
		else {
		retval = cart_solver_->hotstart(H, g, G, lb, ub, lbG, ubG, nWSR, &cpu_time);
		}
		
		if (cart_solver_->isInfeasible()) ROS_WARN("The quadratic programming is infeasible.");
		
		if (retval == qpOASES::SUCCESSFUL_RETURN) {
			cart_solver_->getPrimalSolution(optimal_solution_);
		}
		else if (retval == qpOASES::RET_MAX_NWSR_REACHED) {
			ROS_WARN("The QP couldn't solve because the maximun number of WSR was reached.");
			return false;
		}
		else { 
			ROS_WARN("The QP couldn't find the solution.");
			return false;
		}	

		return true;
	}
	// ----------------------------------------------------------------------------------------------------	
	bool JacoQPSolver::computeJointOpt(double *H, double *g, double *G, double *lb, double *ub, double *lbG, double *ubG)
	{
		//~ ROS_INFO("g: %f, %f, %f, %f, %f, %f.", g[0], g[1], g[2], g[3], g[4], g[5]);
		//~ ROS_INFO("g: %f, %f, %f, %f, %f, %f.", g[6], g[7], g[8], g[9], g[10], g[11]);
		//~ ROS_INFO("g: %f, %f, %f, %f, %f, %f.", g[12], g[13], g[14], g[15], g[16], g[17]);
		//~ ROS_INFO("g: %f, %f, %f, %f, %f, %f.", g[18], g[19], g[20], g[21], g[22], g[23]);
		//~ ROS_INFO("g: %f, %f, %f, %f, %f, %f.", g[24], g[25], g[26], g[27], g[28], g[29]);
		//~ 
		//~ ROS_INFO("lbG: %f, %f, %f, %f, %f, %f.", lbG[0], lbG[1], lbG[2], lbG[3], lbG[4], lbG[5]);
		//~ ROS_INFO("lbG: %f, %f, %f, %f, %f, %f.", lbG[6], lbG[7], lbG[8], lbG[9], lbG[10], lbG[11]);
		//~ ROS_INFO("lbG: %f, %f, %f, %f, %f, %f.", lbG[12], lbG[13], lbG[14], lbG[15], lbG[16], lbG[17]);
		//~ ROS_INFO("lbG: %f, %f, %f, %f, %f, %f.", lbG[18], lbG[19], lbG[20], lbG[21], lbG[22], lbG[23]);
		//~ ROS_INFO("lbG: %f, %f, %f, %f, %f, %f.", lbG[24], lbG[25], lbG[26], lbG[27], lbG[28], lbG[29]);

		//~ ROS_INFO("ub: %f, %f, %f, %f, %f, %f.", ub[0], ub[1], ub[2], ub[3], ub[4], ub[5]);
		//~ ROS_INFO("ub: %f, %f, %f, %f, %f, %f.", ub[6], ub[7], ub[8], ub[9], ub[10], ub[11]);
		//~ ROS_INFO("ub: %f, %f, %f, %f, %f, %f.", ub[12], ub[13], ub[14], ub[15], ub[16], ub[17]);
		//~ ROS_INFO("ub: %f, %f, %f, %f, %f, %f.", ub[18], ub[19], ub[20], ub[21], ub[22], ub[23]);
		//~ ROS_INFO("ub: %f, %f, %f, %f, %f, %f.", ub[24], ub[25], ub[26], ub[27], ub[28], ub[29]);
		 
		//~ ROS_INFO("lb: %f, %f, %f, %f, %f, %f.", lb[0], lb[1], lb[2], lb[3], lb[4], lb[5]);
		//~ ROS_INFO("lb: %f, %f, %f, %f, %f, %f.", lb[6], lb[7], lb[8], lb[9], lb[10], lb[11]);
		//~ ROS_INFO("lb: %f, %f, %f, %f, %f, %f.", lb[12], lb[13], lb[14], lb[15], lb[16], lb[17]);
		//~ ROS_INFO("lb: %f, %f, %f, %f, %f, %f.", lb[18], lb[19], lb[20], lb[21], lb[22], lb[23]);
		//~ ROS_INFO("lb: %f, %f, %f, %f, %f, %f.", lb[24], lb[25], lb[26], lb[27], lb[28], lb[29]);
		
		//~ ROS_INFO("ubG: %f, %f, %f, %f, %f, %f.", ubG[0], ubG[1], ubG[2], ubG[3], ubG[4], ubG[5]);
		//~ ROS_INFO("ubG: %f, %f, %f, %f, %f, %f.", ubG[6], ubG[7], ubG[8], ubG[9], ubG[10], ubG[11]);
		//~ ROS_INFO("ubG: %f, %f, %f, %f, %f, %f.", ubG[12], ubG[13], ubG[14], ubG[15], ubG[16], ubG[17]);
		//~ ROS_INFO("ubG: %f, %f, %f, %f, %f, %f.", ubG[18], ubG[19], ubG[20], ubG[21], ubG[22], ubG[23]);
		//~ ROS_INFO("ubG: %f, %f, %f, %f, %f, %f.", ubG[24], ubG[25], ubG[26], ubG[27], ubG[28], ubG[29]);
		
		//~ ROS_INFO("H: %f, %f, %f, %f, %f, %f.", H[0], H[31], H[62], H[93], H[124], H[155]);
		//~ ROS_INFO("H: %f, %f, %f, %f, %f, %f.", H[186], H[217], H[248], H[279], H[310], H[341]);
		//~ ROS_INFO("H: %f, %f, %f, %f, %f, %f.", H[372], H[403], H[434], H[465], H[496], H[527]);
		//~ ROS_INFO("H: %f, %f, %f, %f, %f, %f.", H[558], H[589], H[620], H[651], H[682], H[713]);
		//~ ROS_INFO("H: %f, %f, %f, %f, %f, %f.", H[744], H[775], H[806], H[837], H[868], H[899]);
		//~ 
		int nWSR = nWSR_;
		qpOASES::returnValue retval;
		
		if (!qp_init_) {
			retval = jnt_solver_->init(H, g, G, lb, ub, lbG, ubG, nWSR);
			if (retval == qpOASES::SUCCESSFUL_RETURN) {
				ROS_INFO("qpOASES problem successfully initialized.");
				qp_init_ = true;
			}
		}
		else {
			retval = jnt_solver_->hotstart(g, lb, ub, lbG, ubG, nWSR);
		}
		if (jnt_solver_->isInfeasible()) ROS_WARN("The quadratic programming is infeasible.");
		
		if (retval == qpOASES::SUCCESSFUL_RETURN) {
			jnt_solver_->getPrimalSolution(optimal_solution_);
			ROS_INFO("OPTIMAL SOLUTION IS READY: %f, %f, %f, %f, %f, %f", optimal_solution_[0], optimal_solution_[1], optimal_solution_[2], optimal_solution_[3], optimal_solution_[4],  optimal_solution_[5] );
			ROS_INFO("OPTIMAL SOLUTION IS READY: %f, %f, %f, %f, %f, %f", optimal_solution_[6], optimal_solution_[7], optimal_solution_[8], optimal_solution_[9], optimal_solution_[10],  optimal_solution_[11] );
			ROS_INFO("OPTIMAL SOLUTION IS READY: %f, %f, %f, %f, %f, %f", optimal_solution_[12], optimal_solution_[13], optimal_solution_[14], optimal_solution_[15], optimal_solution_[16],  optimal_solution_[17] );
			ROS_INFO("OPTIMAL SOLUTION IS READY: %f, %f, %f, %f, %f, %f", optimal_solution_[18], optimal_solution_[19], optimal_solution_[20], optimal_solution_[21], optimal_solution_[22],  optimal_solution_[23] );
			ROS_INFO("OPTIMAL SOLUTION IS READY: %f, %f, %f, %f, %f, %f", optimal_solution_[24], optimal_solution_[25], optimal_solution_[26], optimal_solution_[27], optimal_solution_[28],  optimal_solution_[29] );
			
		}
		else if (retval == qpOASES::RET_MAX_NWSR_REACHED) {
			ROS_WARN("The QP couldn't solve because the maximun number of WSR was reached.");
			return false;
		}
		else { 
			ROS_WARN("The QP couldn't find the solution.");
			return false;
		}	
		return true;
	}
		
	double* JacoQPSolver::getOptimalSolution()
	{
		return optimal_solution_;
	}	

// ----------------------------------------------------------------------------------------------------
}

