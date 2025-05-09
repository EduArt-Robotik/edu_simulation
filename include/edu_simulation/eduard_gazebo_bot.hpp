/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/bot/eduard_v2.hpp>

#include <gz/sim/System.hh>

namespace eduart {
namespace simulation {

class EduardGazeboBot// : public robot::bot::EduardV2
{
public:
  EduardGazeboBot(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, 
    gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& event_manager);
  // ~EduardGazeboBot() override;

  void preUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm);
  void update(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm);
  void postUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm);
};

} // end namespace simulation
} // end namespace eduart
