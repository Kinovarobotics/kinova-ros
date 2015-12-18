#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <ros/ros.h>
using namespace gazebo;

class MimicPlugin : public ModelPlugin
{

	public:
        MimicPlugin();
		~MimicPlugin();
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf );
        void UpdateChild();

        std::string mimic_joint_name_;
        std::string joint_name_;
        double multiplier_;
        physics::ModelPtr model_;
        physics::WorldPtr world_;
        bool kill_sim;
        event::ConnectionPtr updateConnection;

        physics::JointPtr joint_;
        physics::JointPtr mimic_joint_;

};

