#include "jaco_driver/jaco_ik_solver.h"

#include <urdf/model.h>
#include <ros/ros.h>

#include <kdl_parser/kdl_parser.hpp>

#include <kdl/tree.hpp>

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
    
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_)); 
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(chain_));
    ik_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(chain_, q_min_, q_max_, *fk_solver_, *ik_vel_solver_, max_iter));
    std::cout << "Using normal solver" << std::endl;
    
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(chain_));

    return true;
}

 //----------------------------------------------------------------------------------------------------

//bool JacoIKSolver::jointsToCartesian(const KDL::Frame& f_in, KDL::JntArray& q_out)
//{
//    int status = fk_solver_->JntToCart(q_in, p_out, f_in,);
//    return (status == 0);
//}

 //----------------------------------------------------------------------------------------------------

bool JacoIKSolver::cartesianToJoints(const KDL::Frame& f_in, KDL::JntArray& q_out)
{
    return cartesianToJoints(f_in, q_out, q_seed_);
}

// ----------------------------------------------------------------------------------------------------

bool JacoIKSolver::cartesianToJoints(const KDL::Frame& f_in, KDL::JntArray& q_out, const KDL::JntArray& q_seed)
{
    int status = ik_solver_->CartToJnt(q_seed, f_in, q_out);
    return (status == 0);
}
//----------------------------------------------------------------------------------------------------------


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
};
}
