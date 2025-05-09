/**
 * Copyright EduArt Robotik GmbH 2025
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <gz/sim/System.hh>

namespace eduart {
namespace simulation {

class EduardModelPlugin : public gz::sim::System
                        , public gz::sim::ISystemConfigure
{
public:
  EduardModelPlugin();
  ~EduardModelPlugin() override;

  void Configure(
    const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, 
    gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventMgr) override;
};

} // end namespace simulation
} // end namespace eduart
