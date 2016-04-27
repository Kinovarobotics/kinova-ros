#include "jaco_driver/jaco_ik_solver.h"

#include <urdf/model.h>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <tf_conversions/tf_kdl.h>

#define DTR 0.0174532925
#define RTD 57.295779513


namespace jaco
{

// ----------------------------------------------------------------------------------------------------

JacoIKSolver::JacoIKSolver()
{
}

// ----------------------------------------------------------------------------------------------------

JacoIKSolver::~JacoIKSolver()
{
}

// ----------------------------------------------------------------------------------------------------

bool JacoIKSolver::initFromURDF(const std::string urdf, const std::string root_name,
                            const std::string tip_name, unsigned int max_iter, std::string error)
{
    urdf::Model robot_model;
    KDL::Tree tree;

    if (!robot_model.initFile(urdf))
    {
        error += "Could not initialize robot model";
        return false;
    }

    if (!kdl_parser::treeFromFile(urdf, tree))
    {
        error += "Could not initialize tree object";
        return false;
    }

    if (tree.getSegment(root_name) == tree.getSegments().end())
    {
        error += "Could not find root link '" + root_name + "'.";
        return false;
    }

    if (tree.getSegment(tip_name) == tree.getSegments().end())
    {
        error += "Could not find tip link '" + tip_name + "'.";
        return false;
    }

    if (!tree.getChain(root_name, tip_name, chain_))
    {
        error += "Could not initialize chain object";
        return false;
    }

    // Get the joint limits from the robot model

    q_min_.resize(chain_.getNrOfJoints());
    q_max_.resize(chain_.getNrOfJoints());
    q_seed_.resize(chain_.getNrOfJoints());

    joint_names_.resize(chain_.getNrOfJoints());

    unsigned int j = 0;
    for(unsigned int i = 0; i < chain_.getNrOfSegments(); ++i)
    {
        const KDL::Joint& kdl_joint = chain_.getSegment(i).getJoint();
        if (kdl_joint.getType() != KDL::Joint::None)
        {
//            std::cout << chain_.getSegment(i).getName() << " -> " << kdl_joint.getName() << " -> " << chain_.getSegment(i + 1).getName() << std::endl;

            boost::shared_ptr<const urdf::Joint> joint = robot_model.getJoint(kdl_joint.getName());
            if (joint && joint->limits)
            {
                q_min_(j) = joint->limits->lower;
                q_max_(j) = joint->limits->upper;
                q_seed_(j) = (q_min_(j) + q_max_(j)) / 2;
            }
            else
            {
                q_min_(j) = -1e9;
                q_max_(j) = 1e9;
                q_seed_(j) = 0;

            }

            joint_names_[j] = kdl_joint.getName();

            ++j;
        }
    }
;
    
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_)); 
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(chain_));
    ik_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(chain_, q_min_, q_max_, *fk_solver_, *ik_vel_solver_, max_iter));
    std::cout << "Using normal solver" << std::endl;
    
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(chain_));

    return true;
}

 //----------------------------------------------------------------------------------------------------

geometry_msgs::Pose JacoIKSolver::jointsToCartesian(JacoAngles JAngle)
{
	KDL::JntArray joint_in;
	KDL::Frame position_out;
	geometry_msgs::Pose p_out;
	joint_in.resize(6);
	joint_in(0) = ( JAngle.Actuator1 - 180.0 ) * DTR;
	joint_in(1) = ( JAngle.Actuator2 - 270.0 ) * DTR;
	joint_in(2) = ( JAngle.Actuator3 - 90.0  ) * DTR;
	joint_in(3) = ( JAngle.Actuator4 - 180.0 ) * DTR;
	joint_in(4) = ( JAngle.Actuator5 - 180.0 ) * DTR;
	joint_in(5) = ( JAngle.Actuator6 - 270.0 ) * DTR;
	
	
    fk_solver_->JntToCart(joint_in, position_out);
    
    tf::poseKDLToMsg(position_out, p_out);
    return p_out;
}

 //----------------------------------------------------------------------------------------------------

JacoAngles JacoIKSolver::cartesianToJoints(geometry_msgs::Pose p_in, JacoAngles JAngle)
{
	JacoAngles JAngle_out;
	
	KDL::JntArray q_out, joint_in;
	KDL::Frame f_in;
	tf::PoseMsgToKDL(p_in,f_in);  
	
	joint_in.resize(6);
	joint_in(0) = ( JAngle.Actuator1 - 180.0 ) * DTR;
	joint_in(1) = ( JAngle.Actuator2 - 270.0 ) * DTR;
	joint_in(2) = ( JAngle.Actuator3 - 90.0  ) * DTR;
	joint_in(3) = ( JAngle.Actuator4 - 180.0 ) * DTR;
	joint_in(4) = ( JAngle.Actuator5 - 180.0 ) * DTR;
	joint_in(5) = ( JAngle.Actuator6 - 270.0 ) * DTR;
	
    ik_solver_->CartToJnt(joint_in, f_in, q_out);
    
    JAngle_out.Actuator1 = q_out(0) * RTD + 180;
    JAngle_out.Actuator2 = q_out(1) * RTD + 270;
    JAngle_out.Actuator3 = q_out(2) * RTD + 90;
    JAngle_out.Actuator4 = q_out(3) * RTD + 180;
    JAngle_out.Actuator5 = q_out(4) * RTD + 180;
    JAngle_out.Actuator6 = q_out(5) * RTD + 270;
    
    return JAngle_out;
    
}
// ----------------------------------------------------------------------------------------------------

KDL::Jacobian JacoIKSolver::jointToJacobian(JacoAngles JAngle)
{
	KDL::JntArray joint_in;
	KDL::Jacobian jac;
	joint_in.resize(6);
	jac.resize(chain_.getNrOfJoints());

	joint_in(0) = ( JAngle.Actuator1 - 180.0 ) * DTR;
	joint_in(1) = ( JAngle.Actuator2 - 270.0 ) * DTR;
	joint_in(2) = ( JAngle.Actuator3 - 90.0  ) * DTR;
	joint_in(3) = ( JAngle.Actuator4 - 180.0 ) * DTR;
	joint_in(4) = ( JAngle.Actuator5 - 180.0 ) * DTR;
	joint_in(5) = ( JAngle.Actuator6 - 270.0 ) * DTR;
	
	jnt_to_jac_solver_->JntToJac(joint_in, jac);
	
	return jac;
}
// ----------------------------------------------------------------------------------------------
Eigen::MatrixXd JacoIKSolver::desiredAngles(geometry_msgs::Pose p_in, JacoAngles JAngle)
{
	Eigen::MatrixXd JAngle_out = Eigen::MatrixXd::Zero(1,6);
	
	KDL::JntArray q_out, joint_in;
	KDL::Frame f_in;
	tf::PoseMsgToKDL(p_in,f_in);  
	
	joint_in.resize(6);
	joint_in(0) = ( JAngle.Actuator1 - 180.0 ) * DTR;
	joint_in(1) = ( JAngle.Actuator2 - 270.0 ) * DTR;
	joint_in(2) = ( JAngle.Actuator3 - 90.0  ) * DTR;
	joint_in(3) = ( JAngle.Actuator4 - 180.0 ) * DTR;
	joint_in(4) = ( JAngle.Actuator5 - 180.0 ) * DTR;
	joint_in(5) = ( JAngle.Actuator6 - 270.0 ) * DTR;
	
    ik_solver_->CartToJnt(joint_in, f_in, q_out);
    
    for (int i = 0; i < 6; i++){
		JAngle_out(0,i)= saturate(q_out(i) * RTD) ;
	}
    
    return JAngle_out;
    
}
// -------------------------------------------------------------------------------------
KDL::Jacobian JacoIKSolver::desiredJacobian(Eigen::MatrixXd qd)
{
	KDL::JntArray joint_in;
	KDL::Jacobian jac;
	joint_in.resize(6);
	jac.resize(chain_.getNrOfJoints());
	
	qd = DTR*qd;
	
	for (int i = 0; i < 6; i++){
		joint_in(i) = qd(0,i);
	}
	
	jnt_to_jac_solver_->JntToJac(joint_in, jac);
	
	return jac;
}

double JacoIKSolver::saturate(double input)
{
	while (input > 180 || input < -180){
		if (input > 180) input -=180;
		else if (input < -180) input+=180;
	}
	return input;
};
}
