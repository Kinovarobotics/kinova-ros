#include <gazebo_mimic_plugin/mimic_plugin.h>

using namespace gazebo;

MimicPlugin::MimicPlugin():  ModelPlugin()
{
  kill_sim = false;

  joint_.reset();
  mimic_joint_.reset();
}

MimicPlugin::~MimicPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);

  kill_sim = true;
}

void MimicPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  this->model_ = _parent;
  this->world_ = this->model_->GetWorld();


  joint_name_ = "joint";
  if (_sdf->HasElement("joint"))
    joint_name_ = _sdf->GetElement("joint")->Get<std::string>();

  mimic_joint_name_ = "mimicJoint";
  if (_sdf->HasElement("mimicJoint"))
    mimic_joint_name_ = _sdf->GetElement("mimicJoint")->Get<std::string>();

  multiplier_ = 1.0;
  if (_sdf->HasElement("multiplier"))
    multiplier_ = _sdf->GetElement("multiplier")->Get<double>();


  // Get the name of the parent model
  std::string modelName = _sdf->GetParent()->Get<std::string>("name");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MimicPlugin::UpdateChild, this));
  gzdbg << "Plugin model name: " << modelName << "\n";

  joint_ = model_->GetJoint(joint_name_);
  mimic_joint_ = model_->GetJoint(mimic_joint_name_);
}

void MimicPlugin::UpdateChild()
{
    mimic_joint_->SetAngle(0, math::Angle(joint_->GetAngle(0).Radian()*multiplier_));
}

GZ_REGISTER_MODEL_PLUGIN(MimicPlugin);

