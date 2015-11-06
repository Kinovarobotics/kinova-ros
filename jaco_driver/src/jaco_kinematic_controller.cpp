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
		node_handle_.param<string>("urdf", urdf_, "/home/xwu/simulation_ws/src/jaco-ros/jaco2/urdf/jaco2.urdf");
		node_handle_.param<string>("root_name", root_name_, "jaco_link_base");
		node_handle_.param<string>("tip_name", tip_name_, "jaco_link_6");
		node_handle_.param<string>("error", err_, "");
		node_handle_.param<bool>("use_motion_manipulator_index", use_motion_manipulator_index_, false);
		node_handle_.param<bool>("use_force_manipulator_index", use_force_manipulator_index_, false);
		node_handle_.param<bool>("use_joint_limit_avoid", use_joint_limit_avoid_, true);
		
		max_iter_ = 10;
		mode = 0;
		 
		if( ik_solver_.initFromURDF(urdf_, root_name_, tip_name_, max_iter_, err_))
		{
			ROS_INFO("KDL is ready for use");
		}
		
		
        vGoal  = VectorXf::Zero(6,1);
        pGoal  = VectorXf::Zero(6,1);
        dQ     = VectorXf::Zero(6,1);
        nQ     = VectorXf::Zero(6,1);
        vGoal  = VectorXf::Zero(6,1);
        Jq     = VectorXf::Zero(6,1);
        vGoalTest = VectorXf::Zero(3,1);
        exdQ   = VectorXf::Zero(8,1);
        exnQ   = VectorXf::Zero(8,1);

        mJaco  = MatrixXf::Zero(6,6);
		nJaco  = MatrixXf::Zero(3,6);
		nNewJaco  = MatrixXf::Zero(3,6);
        mPJaco = MatrixXf::Zero(6,6);
        nPJaco = MatrixXf::Zero(6,3);
        nPNewJaco = MatrixXf::Zero(6,3);
        dJaco = MatrixXf::Zero(3,6);
        
        exJaco = MatrixXf::Zero(6,8);
        exPJaco = MatrixXf::Zero(8,6);
        
        manip_.values.resize(1);
        manip_pub_ = node_handle_.advertise<sensor_msgs::ChannelFloat32>("minipulability", 1);
        
        husky_cmd_pub_ = node_handle_.advertise<geometry_msgs::Twist>("vrep/twistCommand", 1);

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
		
		if(use_joint_limit_avoid_ == true)
		{
			callJointLimit();
		}
		
		if(use_motion_manipulator_index_ == true)
		{
			callMotionManip(dV);
			updateexdQ();
		}
		
		
		if(use_force_manipulator_index_ == true)
		{
			callForceManip(dV);
			updateexdQ();
		}
		
			
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
	
	void JacoKinematicController::callForceManip(CartesianInfo dV)
	{
		vGoalTest(0) = 0.0;
		vGoalTest(1) = 0.0;
		vGoalTest(2) = -1.0;
		
		if (fabsf(vGoalTest(0)) > 0.01 ||  fabsf(vGoalTest(1)) > 0.01 || fabsf(vGoalTest(2)) > 0.01)
		{
			getJacobian();
			nPJaco = calcNullSpacePseudoJacobian(nJaco);
			VectorXf dQQ = VectorXf::Zero(6,1);
			float norm_dq = dQ.norm();

			for (int i = 0; i < 6; i++)
			{
				dQQ(i) = dQ(i)/norm_dq;
				derivativeJaco(dQQ, dQ(i));		
				maxForceManipulability(vGoalTest, i);
				dQQ(i) = 0.0;
				Jq(i) = fabsf(Jq(i))< 0.001?0.0:Jq(i);	
			}

			nQ = linearRegression(Jq, 5000);
			dQ = calcdQnQ(nPJaco, vGoalTest, nQ);
			
			manip_.values[0] = fManip;
			manip_pub_.publish(manip_);
		}
		else
		{
			dQ = VectorXf::Zero(6,1);
		}

	}
	
	void JacoKinematicController::callMotionManip(CartesianInfo dV)
	{		
		vGoalTest(0) = dV.X;
		vGoalTest(1) = dV.Y;
		vGoalTest(2) = dV.Z;

		if (fabsf(vGoalTest(0)) > 0.01 ||  fabsf(vGoalTest(1)) > 0.01 || fabsf(vGoalTest(2)) > 0.01)
		{
			getJacobian();
			nPJaco = calcNullSpacePseudoJacobian(nJaco);
			VectorXf dQQ = VectorXf::Zero(6,1);
			float norm_dq = dQ.norm();
	
			for (int i = 0; i < 6; i++)
			{
				dQQ(i) = dQ(i)/norm_dq;
				derivativeJaco(dQQ, dQ(i));		
				maxMotionManipulability(vGoalTest, i);
				dQQ(i) = 0.0;
				Jq(i) = fabsf(Jq(i))< 0.001?0.0:Jq(i);	
			}

			nQ = linearRegression(Jq, 50);
			dQ = calcdQnQ(nPJaco, vGoalTest, nQ);
			
			manip_.values[0] = mManip;
			manip_pub_.publish(manip_);
		}
		else
		{
			dQ = VectorXf::Zero(6,1);
		}

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
	
	void JacoKinematicController::callJointLimit()
	{
		exnQ = VectorXf::Zero(8,1);			
		exnQ(2) = (90.0 - ( current_angle.Actuator1 - 180.0 )) * DTR;
//		dQ(1) = ( current_angle.Actuator2 - 270.0 ) * DTR;
//		dQ(2) = ( current_angle.Actuator3 - 90.0  ) * DTR;
//		dQ(3) = ( current_angle.Actuator4 - 180.0 ) * DTR;
//		dQ(4) = ( current_angle.Actuator5 - 180.0 ) * DTR;
//		dQ(5) = ( current_angle.Actuator6 - 270.0 ) * DTR;
		
		exnQ = calcexdQ(exPJaco, exnQ, 2);
		exdQ += exnQ;
//		ROS_INFO("JOINT LIMIT DQ: %f, %f, %f, %f, %f, %f, %f, %f", exnQ(0), exnQ(1), exnQ(2), exnQ(3), exnQ(4), exnQ(5), exnQ(6), exnQ(7));
	}
		
			
		
	
	void JacoKinematicController::derivativeJaco(VectorXf dq, float dqq)
	{
		JacoAngles new_angle;
		new_angle.Actuator1 = current_angle.Actuator1 + dq(0);
		new_angle.Actuator2 = current_angle.Actuator2 + dq(1);
		new_angle.Actuator3 = current_angle.Actuator3 + dq(2);
		new_angle.Actuator4 = current_angle.Actuator4 + dq(3);
		new_angle.Actuator5 = current_angle.Actuator5 + dq(4);
		new_angle.Actuator6 = current_angle.Actuator6 + dq(5);
		
		j_new_ = ik_solver_.jointToJacobian(new_angle);

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				nNewJaco(i,j) = j_new_(i,j);
			}
		}
		dqq = fabsf(dqq) < 0.01?1.0:dqq;
		dJaco = (nNewJaco - nJaco)/dqq;
/*		
		ROS_INFO("Derivative of Pesudo Jacobian: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", 
				dJaco(0,0), dJaco(0,1), dJaco(0,2), dJaco(0,3), dJaco(0,4), dJaco(0,5), 
				dJaco(1,0), dJaco(1,1), dJaco(1,2), dJaco(1,3), dJaco(1,4), dJaco(1,5),
				dJaco(2,0), dJaco(2,1), dJaco(2,2), dJaco(2,3), dJaco(2,4), dJaco(2,5)
				);
*/
	}
	
	void JacoKinematicController::maxForceManipulability(VectorXf dx, int n)
	{

		float dxnorm, opttemp1, opttemp2;
		VectorXf dq = VectorXf::Zero(6,1);
		MatrixXf opt1 = MatrixXf::Zero(3,3);
		
		VectorXf temp1 = VectorXf::Zero(6,1);
		VectorXf temp2 = VectorXf::Zero(6,1);		
		VectorXf shortX = VectorXf::Zero(3,1);
		
		dxnorm =sqrtf(dx(0)*dx(0) + dx(1)*dx(1) +dx(2)*dx(2));
		shortX(0) = dx(0)/dxnorm;
		shortX(1) = dx(1)/dxnorm;
		shortX(2) = dx(2)/dxnorm;
		
		opt1 = nJaco*nJaco.transpose();
		fManip = shortX.transpose()*opt1*shortX;
		
//		ROS_INFO("Manipulability: %f", fManip);
			
		temp1 = dJaco.transpose()*shortX;
		opttemp1 = temp1.transpose()*nJaco.transpose()*shortX;
			
		temp2 = nJaco.transpose()*shortX;
		opttemp2 = temp2.transpose()*dJaco.transpose()*shortX;			
		Jq(n) = opttemp1 + opttemp2;
//		ROS_INFO("Jq: %f", Jq(n));

	}
	
	void JacoKinematicController::maxMotionManipulability(VectorXf dx, int n)
	{

		float dxnorm, opttemp, temp3;
		VectorXf dq = VectorXf::Zero(6,1);
		MatrixXf opt1 = MatrixXf::Zero(3,3);
		
		MatrixXf temp = MatrixXf::Zero(3,3);
		MatrixXf temp1 = MatrixXf::Zero(3,3);
		MatrixXf temp2 = MatrixXf::Zero(3,3);	
		VectorXf shortX = VectorXf::Zero(3,1);
		
		dxnorm =sqrtf(dx(0)*dx(0) + dx(1)*dx(1) +dx(2)*dx(2));
		shortX(0) = dx(0)/dxnorm;
		shortX(1) = dx(1)/dxnorm;
		shortX(2) = dx(2)/dxnorm;
		
		opt1 = nJaco*nJaco.transpose();
		mManip = shortX.transpose()*opt1.inverse()*shortX;
		
//		ROS_INFO("Motion Manipulability: %f", mManip);
			
		temp1 = dJaco*nJaco.transpose();		
		temp2 = nJaco*dJaco.transpose();
		temp = temp1+temp2;
		
		temp3 = shortX.transpose()*shortX;
		temp3 = 1/temp3;
		opttemp = temp3*temp3*shortX.transpose()*temp*shortX;
//		opttemp = fabsf(opttemp)< 0.01?0.0:1.0/opttemp;			
		Jq(n) = -mManip*mManip*opttemp;
//		ROS_INFO("Jq: %f", Jq(n));

	}
	
	VectorXf JacoKinematicController::linearRegression(VectorXf jq, int K)
	{
		VectorXf dq = VectorXf::Zero(6,1);		

		dq = -K*jq;

//		ROS_INFO("dq: %f, %f, %f, %f, %f, %f", dq(0), dq(1), dq(2), dq(3), dq(4), dq(5));
		return dq;
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
			exJaco(0,0) = -1.0;
			exJaco(5,1) = 1.0;
			exJaco(0,1) = -ee_pos.position.y+0.31;
			exJaco(1,1) = -ee_pos.position.x;
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
	
	void JacoKinematicController::getJacobian()
	{		
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 6; j++)
			{
				nJaco(i,j) = exJaco(i,j+2);
				dQ(j) = exdQ(j+2);
			}
		}
	}
			
	
	void JacoKinematicController::updateexdQ()
	{
		exdQ(2) += dQ(0);
		exdQ(3) += dQ(1);
		exdQ(4) += dQ(2);
		exdQ(5) += dQ(3);
		exdQ(6) += dQ(4);
		exdQ(7) += dQ(5);
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
			
			case 3:
			dq = PJ*x;
			
			break;
		}
		return dq;
//		double K = 1.0;	
//		dq = PJ*(x+K*(pGoal-cP));

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
//		double K = 1.0;	
//		dq = PJ*(x+K*(pGoal-cP));

	}
	
	VectorXf JacoKinematicController::calcdQnQ(MatrixXf PJ, VectorXf x, VectorXf nq)
	{
		VectorXf dq = VectorXf::Zero(6,1);
		VectorXf xx = VectorXf::Zero(3,1);

//		dq = PJ*x + 5.0*(MatrixXf::Identity(6,6)-PJ*nJaco)*nq;
		dq = 5*(MatrixXf::Identity(6,6)-PJ*nJaco)*nq;
//		ROS_INFO("FINAL: %f, %f, %f, %f, %f, %f", dq(0), dq(1), dq(2), dq(3), dq(4), dq(5));
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

	MatrixXf JacoKinematicController::getPseudoJacobian()
	{
		return mPJaco;
	};

	void JacoKinematicController::watchdog(const ros::TimerEvent&)
	{
	}
}
	
		
		
		
		
