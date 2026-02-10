/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <gz/sim/System.hh>

#include <edu_robot/color.hpp>
#include <edu_robot/lighting.hpp>

#include "edu_simulation/gazebo_lighting.hpp"

namespace eduart {
namespace simulation {

class GazeboLightingPlugin : public gz::sim::System
                           , public gz::sim::ISystemConfigure
                           , public gz::sim::ISystemUpdate
{
public:
  GazeboLightingPlugin();
  ~GazeboLightingPlugin() override;

  void Configure(
    const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, 
    gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& event_manager) override;
  void Update(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;

private:
  void callbackReceiveLightingCommand(const LightingCommand& command);

  gz::sim::Entity _lighting_entity;
  gz::sim::Entity _visual_entity;
  robot::Color _color;
  robot::Lighting::Mode _mode;
};

} // end namespace simulation
} // end namespace eduart