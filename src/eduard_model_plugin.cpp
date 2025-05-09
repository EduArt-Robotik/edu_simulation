#include "edu_simulation/eduard_model_plugin.hpp"

namespace eduart {
namespace simulation {

EduardModelPlugin::EduardModelPlugin()
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  // if (rclcpp::ok() == false) {
  //   rclcpp::init(0, 0);
  // }
  // _ros_executer = std::make_shared<gazebo_ros::Executor>();
}

EduardModelPlugin::~EduardModelPlugin()
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
}

void EduardModelPlugin::Configure(
  const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, 
  gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& event_manager)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  _robot = std::make_shared<EduardGazeboBot>(entity, sdf, ecm, event_manager);
}

void EduardModelPlugin::PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{
  if (_robot == nullptr) {
    return;
  }

  _robot->preUpdate(info, ecm);
}

void EduardModelPlugin::Update(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{
  if (_robot == nullptr) {
    return;
  }

  _robot->update(info, ecm);
}

void EduardModelPlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm)
{
  if (_robot == nullptr) {
    return;
  }

  _robot->postUpdate(info, ecm);
}

} // end namespace simulation
} // end namespace eduart

#include <gz/plugin/Register.hh>
 
// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
  eduart::simulation::EduardModelPlugin,
  gz::sim::System,
  eduart::simulation::EduardModelPlugin::ISystemConfigure,
  eduart::simulation::EduardModelPlugin::ISystemPreUpdate,
  eduart::simulation::EduardModelPlugin::ISystemUpdate,
  eduart::simulation::EduardModelPlugin::ISystemPostUpdate
)
