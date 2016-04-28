#include <kinova/KinovaTypes.h>

#include "jaco_driver/jaco_kinematic_controller.h"
#include <geometry_msgs/Pose.h>

#include "jaco_driver/jaco_types.h"
#include "jaco_driver/jaco_ik_solver.h"


using namespace Eigen;
using namespace std;

namespace jaco

{
	JacoKinematicController::JacoKinematicController(JacoComm &arm_comm, JacoIKSolver &ik_solver, const ros::NodeHandle &nh)
		: node_handle_(nh), arm_comm_(arm_comm), ik_solver_(ik_solver)
	{
		ros::NodeHandle pn("~");
		
		node_handle_.param<double>("status_interval_seconds", watchdog_interval_seconds, 0.1);
		node_handle_.param<string>("urdf", urdf_, "jaco2.urdf");
		node_handle_.param<string>("root_name", root_name_, "jaco_link_base");
		node_handle_.param<string>("tip_name", tip_name_, "jaco_link_6");
		node_handle_.param<string>("error", err_, "");
		
		max_iter_ = 10;

		mode = 0;
		 
		if( ik_solver_.initFromURDF(urdf_, root_name_, tip_name_, max_iter_, err_))
		{
			ROS_INFO("KDL is ready for use");
		}
		else
		{
			ROS_WARN("KDL is NOT ready");
		}
		
		
        vGoal  = VectorXf::Zero(6,1);
        dQ     = VectorXf::Zero(6,1);
        nQ     = VectorXf::Zero(6,1);
        exdQ   = VectorXf::Zero(8,1);
        exnQ   = VectorXf::Zero(8,1);

        mJaco  = MatrixXf::Zero(6,6);
		nJaco  = MatrixXf::Zero(3,6);
        mPJaco = MatrixXf::Zero(6,6);
        nPJaco = MatrixXf::Zero(6,3);
        exJaco = MatrixXf::Zero(6,8);
        exPJaco = MatrixXf::Zero(8,6);
        
        husky_cmd_pub_ = node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

		watchdog_timer = node_handle_.createTimer(ros::Duration(watchdog_interval_seconds),
											   &JacoKinematicController::watchdog, this);
		
		ROS_INFO("Kinematic Controller is Ready");
	}
    
    JacoKinematicController::~JacoKinematicController()
    {
		watchdog_timer.stop();
	}
	
	void JacoKinematicController::callBoth(CartesianInfo dV)
	{
		mode = 0;
		
		vGoal(0) = dV.X;
		vGoal(1) = dV.Y;
		vGoal(2) = dV.Z;
		vGoal(3) = dV.ThetaX;
		vGoal(4) = dV.ThetaY;
		vGoal(5) = dV.ThetaZ;

		updateConfig(mode);

		exPJaco = calcPJacobian(exJaco);

		exdQ = calcexdQ(exPJaco, vGoal, 1);
				
		actionAll(exdQ);
	}
	
	void JacoKinematicController::callBothNullSpace(AngularInfo dJ)
	{
		mode = 0;
		
		exnQ(0) = 0.0;
		exnQ(1) = 0.0;
		exnQ(2) = dJ.Actuator1;
		exnQ(3) = dJ.Actuator2;
		exnQ(4) = dJ.Actuator3;
		exnQ(5) = dJ.Actuator4;
		exnQ(6) = dJ.Actuator5;
		exnQ(7) = dJ.Actuator6;

		updateConfig(mode);

		exPJaco = calcPJacobian(exJaco);

		exdQ = calcexdQ(exPJaco, exnQ, 2);
		
		actionAll(exdQ);
		
	}
	
	void JacoKinematicController::call(CartesianInfo dV)
	{
		mode = 1;
		
		vGoal(0) = dV.X;
		vGoal(1) = dV.Y;
		vGoal(2) = dV.Z;
		vGoal(3) = dV.ThetaX;
		vGoal(4) = dV.ThetaY;
		vGoal(5) = dV.ThetaZ;

		updateConfig(mode);

		mPJaco = calcPseudoJacobian(mJaco);
//		mPJaco = calcSingularityAwarePseudoJacobian(mJaco, 6);

		dQ = calcdQ(mPJaco, vGoal, mode);
		
		action(dQ);
	}
	

	
	void JacoKinematicController::callNullSpace(AngularInfo dJ)
	{
		mode = 2;
		
		nQ(0) = dJ.Actuator1;
		nQ(1) = dJ.Actuator2;
		nQ(2) = dJ.Actuator3;
		nQ(3) = dJ.Actuator4;
		nQ(4) = dJ.Actuator5;
		nQ(5) = dJ.Actuator6;

		updateConfig(mode);

		nPJaco = calcNullSpacePseudoJacobian(nJaco);

		dQ = calcdQ(nPJaco, nQ, mode);
					
		action(dQ);
		
	}
	

	void JacoKinematicController::updateConfig(int md)
	{		
		geometry_msgs::Pose ee_pos;
		arm_comm_.getJointAngles(current_angle);
		j_ = ik_solver_.jointToJacobian(current_angle);
		ee_pos = ik_solver_.jointsToCartesian(current_angle);
//		ROS_INFO("POSITION: x:%f, y: %f", ee_pos.position.x, ee_pos.position.y);
		switch (md)
		{
			case 0:
			
			for (int i = 0; i < 6; i++)
			{
				for (int j = 0; j < 6; j++)
				{
					exJaco(i,j+2) = j_(i,j);
				}
            }
			exJaco(0,0) = -100;
			exJaco(5,1) = 100;
			exJaco(0,1) = -100*(-ee_pos.position.y+0.31);
			exJaco(1,1) = -100*ee_pos.position.x;

//            ROS_INFO("x: %f Y: %f", ee_pos.position.x, ee_pos.position.y);
			break;
			
			case 1:
				
			for (int i = 0; i < 6; i++)
			{
				for (int j = 0; j < 6; j++)
				{
					mJaco(i,j) = j_(i,j);
				}
			}
			
			break;
			
			case 2:
			
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 6; j++)
				{
					nJaco(i,j) = j_(i,j);
				}
			}
			
			break;
			
			case 3:
			
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 6; j++)
				{
					nJaco(i,j) = j_(i,j);
				}
			}
			
			break;
		}
	}
			


	MatrixXf JacoKinematicController::calcPseudoJacobian(MatrixXf mJ)
	{		
		MatrixXf mPJ = MatrixXf::Zero(6,6);
		mPJ = mJ * mJ.transpose();
		mPJ = mPJ.inverse();
		mPJ = mJ.transpose()*mPJ;
			
		return mPJ;

	}
	
	MatrixXf JacoKinematicController::calcNullSpacePseudoJacobian(MatrixXf mJ)
	{	
		MatrixXf mPJ1 = MatrixXf::Zero(3,3);
		MatrixXf mPJ2 = MatrixXf::Zero(6,3);
		mPJ1 = mJ * mJ.transpose();
		mPJ1 = mPJ1.inverse();
		mPJ2 = mJ.transpose()*mPJ1;
			
		return mPJ2;
	}
	
	MatrixXf JacoKinematicController::calcSingularityAwarePseudoJacobian(MatrixXf mJ, int n)
	{		
		MatrixXf J = MatrixXf::Zero(n,n);
		MatrixXf U = MatrixXf::Zero(n,n);
		MatrixXf V = MatrixXf::Zero(n,n);
		MatrixXf Sinv = MatrixXf::Zero(n,n);
		VectorXf S = VectorXf::Zero(n,1);
		
		JacobiSVD<MatrixXf> svd(mJ, ComputeThinU | ComputeThinV);
		U = svd.matrixU();
		V = svd.matrixV();
		S = svd.singularValues();
		
		for (int i = 0; i < n; i++)
		{
			Sinv(i,i) = fabsf(S(i)) < 0.05?0.0:1.0/S(i);
		}
		
		J = V*Sinv*U.transpose();
		
		return J;
		
		
	}
	
	MatrixXf JacoKinematicController::calcPJacobian(MatrixXf mJ)
	{		
		MatrixXf mPJ1 = MatrixXf::Zero(6,6);
		MatrixXf mPJ2 = MatrixXf::Zero(8,6);
		mPJ1 = mJ * mJ.transpose();
		mPJ1 = mPJ1.inverse();
		mPJ2 = mJ.transpose()*mPJ1;
			
		return mPJ2;

	}
			
	
	VectorXf JacoKinematicController::calcdQ(MatrixXf PJ, VectorXf x, int md)
	{
		VectorXf dq = VectorXf::Zero(6,1);
		
		switch (md)
		{
			case 0:
			dq = PJ*x;
			
			break;
			
			case 1:
			dq = PJ*x;
			
			break;
			
			case 2:
			dq = (MatrixXf::Identity(6,6)-PJ*nJaco)*x;

			break;

		}
		
		return dq;

	}
	
	VectorXf JacoKinematicController::calcexdQ(MatrixXf PJ, VectorXf x, int md)
	{
		VectorXf dq = VectorXf::Zero(8,1);
		switch (md)
		{
			case 1:
			dq = PJ*x;
			
			break;
			
			case 2:
			dq = (MatrixXf::Identity(8,8)-PJ*exJaco)*x;

			break;
		}
		
//		dq = (MatrixXf::Identity(8,8)-PJ*exJaco)*x;

		return dq;

	}
	
		
	void JacoKinematicController::action(VectorXf dq)
	{
		joint_velocities_.Actuator1 = dq(0);
		joint_velocities_.Actuator2 = dq(1);
		joint_velocities_.Actuator3 = dq(2);
		joint_velocities_.Actuator4 = dq(3);
		joint_velocities_.Actuator5 = dq(4);
		joint_velocities_.Actuator6 = dq(5);

        
		arm_comm_.setJointVelocities(joint_velocities_);
	}
	
	void JacoKinematicController::actionAll(VectorXf dq)
	{
		geometry_msgs::Twist husky_cmd;
		husky_cmd.linear.x = dq(0);
		husky_cmd.angular.z = dq(1);
		
		husky_cmd_pub_.publish(husky_cmd);
		
		joint_velocities_.Actuator1 = dq(2);
		joint_velocities_.Actuator2 = dq(3);
		joint_velocities_.Actuator3 = dq(4);
		joint_velocities_.Actuator4 = dq(5);
		joint_velocities_.Actuator5 = dq(6);
		joint_velocities_.Actuator6 = dq(7);

        
		arm_comm_.setJointVelocities(joint_velocities_);
	}


	void JacoKinematicController::watchdog(const ros::TimerEvent&)
	{
	}
}
	
		
		
		
		
