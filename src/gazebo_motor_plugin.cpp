#include "edu_simulation/gazebo_motor_plugin.hpp"

#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>

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
  std::cout << __PRETTY_FUNCTION__ << std::endl;

  // Communication to Inner System
  if (sdf->HasElement("joint_name") == false) {
    std::cout << "no joint name present in plugin section --> cancel configuring" << std::endl;
    return;
  }

  const std::string joint_name = sdf->FindElement("joint_name")->Get<std::string>();
  const auto model_name = gz::sim::Model(entity).Name(ecm);  

  if (auto joint_entity = ecm.EntityByName(joint_name)) {
    _joint_entity = *joint_entity;
  }
  if (!ecm.EntityHasComponentType(_joint_entity, gz::sim::components::JointVelocity().TypeId())) {
    std::cout << "creating joint velocity component" << std::endl;
    ecm.CreateComponent(_joint_entity, gz::sim::components::JointVelocity());
  }

  // Communication to Outer System
  _node->Subscribe(
    model_name + '/' + joint_name + "/set_joint_velocity",
    &GazeboMotorPlugin::callbackReceiveJointVelocity, this
  );
}

void GazeboMotorPlugin::PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{
  if (_joint_entity == 0) {
    // not ready --> return
    return;
  }

  // gz::sim::Joint joint(_joint_entity);
  // joint.SetVelocity(ecm, _velocity);


  // std::cout << "set velocity" << std::endl;
  if (ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(_joint_entity, _velocity) == false) {
    // std::cout << "error during setting velocity (" << _velocity[0] << ") to joint" << std::endl;
  }
}

void GazeboMotorPlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm)
{
  const auto velocity = ecm.Component<gz::sim::components::JointVelocity>(_joint_entity)->Data();
  // std::cout << "read velocity: ";
  // for (const auto data : velocity) {
  //   std::cout << data << " ";
  // }
  // std::cout << std::endl;
}

void GazeboMotorPlugin::callbackReceiveJointVelocity(const gz::msgs::Double& velocity)
{
  std::cout << "received velocity = " << velocity.data() << std::endl;
  _velocity[0] = velocity.data();
  // _velocity[1] = velocity.data();
  // _velocity[2] = velocity.data();
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
