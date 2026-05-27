#include "edu_simulation/gazebo_motor_plugin.hpp"

#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>

namespace eduart {
namespace simulation {

GazeboMotorPlugin::GazeboMotorPlugin()
  : _node(std::make_shared<gz::transport::Node>())
{

}

GazeboMotorPlugin::~GazeboMotorPlugin()
{

}

void GazeboMotorPlugin::Configure(
  const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, 
  gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& event_manager)
{
  // Communication to Inner System
  if (sdf->HasElement("joint_name") == false) {
    gzerr << "no joint name present in plugin section --> cancel configuring" << std::endl;
    return;
  }

  const std::string joint_name = sdf->FindElement("joint_name")->Get<std::string>();
  const auto model_name = gz::sim::Model(entity).Name(ecm);  

  if (auto joint_entity = ecm.EntityByName(joint_name)) {
    _joint_entity = *joint_entity;
  }
  if (!ecm.EntityHasComponentType(_joint_entity, gz::sim::components::JointVelocity().TypeId())) {
    ecm.CreateComponent(_joint_entity, gz::sim::components::JointVelocity());
  }
  if (!ecm.EntityHasComponentType(_joint_entity, gz::sim::components::JointForceCmd().TypeId())) {
    ecm.CreateComponent(_joint_entity, gz::sim::components::JointForceCmd());
  }

  // Communication to Outer System
  gz::transport::AdvertiseMessageOptions opts;
  opts.SetMsgsPerSec(20); // limit publishing frequency to 20 Hz
  _pub_velocity = _node->Advertise<gz::msgs::Double>(
    model_name + '/' + joint_name + "/get_joint_velocity", opts
  );
  _node->Subscribe(
    model_name + '/' + joint_name + "/set_joint_velocity",
    &GazeboMotorPlugin::callbackReceiveJointVelocity, this
  );

  // Control
  robot::algorithm::Pid::Parameter pid_parameter{
    .kp = 0.5,
    .ki = 5.0,
    .kd = 0.0,
    .limit = 30.0,
    .input_filter_weight = 1.0,
    .use_anti_windup = true
  };

  if (const auto element = sdf->FindElement("pid"); element != nullptr) {
    if (const auto kp_element = element->FindElement("kp"); kp_element != nullptr) {
      pid_parameter.kp = kp_element->Get<double>();
    }
    if (const auto ki_element = element->FindElement("ki"); ki_element != nullptr) {
      pid_parameter.ki = ki_element->Get<double>();
    }
    if (const auto kd_element = element->FindElement("kd"); kd_element != nullptr) {
      pid_parameter.kd = kd_element->Get<double>();
    }
  }
  if (const auto element = sdf->FindElement("max_torque"); element != nullptr) {
    pid_parameter.limit = element->Get<double>();
  }

  _pid = std::make_shared<robot::algorithm::Pid>(pid_parameter);
}

void GazeboMotorPlugin::PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{
  if (_joint_entity == 0 || std::chrono::duration<double>(info.dt).count() <= 0.0) {
    // not ready or dt <= 0 --> return
    return;
  }

  // Control Loop
  const double dt = std::chrono::duration<double>(info.dt).count();
  const double velocity = ecm.Component<gz::sim::components::JointVelocity>(_joint_entity)->Data()[0];
  _effort[0] = _pid->process(_cmd_velocity[0], velocity, dt);

  if (ecm.SetComponentData<gz::sim::components::JointForceCmd>(_joint_entity, _effort) == false) {
    gzerr << "error during setting force (" << _effort[0] << ") to joint" << std::endl;
  }
}

void GazeboMotorPlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm)
{
  const auto velocity = ecm.Component<gz::sim::components::JointVelocity>(_joint_entity)->Data();
  gz::msgs::Double velocity_msgs;
  velocity_msgs.set_data(velocity[0]);
  _pub_velocity.Publish(velocity_msgs);
}

void GazeboMotorPlugin::callbackReceiveJointVelocity(const gz::msgs::Double& velocity)
{
  _cmd_velocity[0] = velocity.data();
}

} // end namespace simulation
} // end namespace eduart

#include <gz/plugin/Register.hh>
 
// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
  eduart::simulation::GazeboMotorPlugin,
  gz::sim::System,
  eduart::simulation::GazeboMotorPlugin::ISystemConfigure,
  eduart::simulation::GazeboMotorPlugin::ISystemPreUpdate,
  eduart::simulation::GazeboMotorPlugin::ISystemPostUpdate
)
