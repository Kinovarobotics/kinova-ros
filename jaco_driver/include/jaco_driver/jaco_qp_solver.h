#ifndef JACO_DRIVER_JACO_QP_SOLVER_H_
#define JACO_DRIVER_JACO_QP_SOLVER_H_

#include <qpOASES.hpp>
#include <ros/ros.h>


namespace jaco
{

	class JacoQPSolver
	{
		public:
			JacoQPSolver();

			~JacoQPSolver();

			bool initSolver(int variables, int constraints, int horizon, int nWSR);

			bool computeCartOpt(double *H, double *g, double *G, double *lb, double *ub, double *lbG, double *ubG, double cputime);
			bool computeJointOpt(double *H, double *g, double *G, double *lb, double *ub, double *lbG, double *ubG);			

			double* getOptimalSolution();

		protected:
			double* optimal_solution_;
		
		private:
			qpOASES::QProblem *jnt_solver_;
			qpOASES::SQProblem *cart_solver_;
				
			bool qp_init_, allocated_;
				
			int nWSR_, variables_, constraints_, horizon_;
			
	}; 

} 




#endif
