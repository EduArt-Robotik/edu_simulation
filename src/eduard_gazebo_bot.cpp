#include "edu_simulation/eduard_gazebo_bot.hpp"
#include "edu_simulation/gazebo_hardware_adapter.hpp"
#include "edu_simulation/eduard_hardware_component_factory.hpp"

#include <bot/eduard_v2.hpp>

#include <gz/sim/Events.hh>
#include <gz/sim/Model.hh>

namespace eduart {
namespace simulation {

EduardGazeboBot::EduardGazeboBot(
  const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, 
  gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& event_manager)
  : robot::bot::EduardV2(
      "eduard_gazebo_bot", std::make_unique<GazeboHardwareAdapter>(sdf), gz::sim::Model(entity).Name(ecm)
    )
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  auto hardware_adapter = std::dynamic_pointer_cast<GazeboHardwareAdapter>(_hardware_interface);

  const auto model_name = gz::sim::Model(entity).Name(ecm);
  std::cout << "model name = " << model_name << std::endl;
  std::cout << "entity = " << entity << std::endl;
  EduardHardwareComponentFactory factory(hardware_adapter, entity, sdf, ecm, *this);

  initialize(factory);
  _mode_state_machine.switchToMode(eduart::robot::RobotMode::INACTIVE);

}

EduardGazeboBot::~EduardGazeboBot()
{

}

void EduardGazeboBot::preUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{

}

void EduardGazeboBot::update(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{

}

void EduardGazeboBot::postUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm)
{

}

} // end namespace simulation
} // end namespace eduart
