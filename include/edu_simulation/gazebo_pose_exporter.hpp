/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <gz/sim/System.hh>

#include <gz/sim/Model.hh>

namespace eduart {
namespace simulation {

class PoseExporter : public gz::sim::System
                   , public gz::sim::ISystemConfigure
                   , public gz::sim::ISystemPostUpdate
{
public:
  PoseExporter();
  ~PoseExporter() override;

  void Configure(
    const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, 
    gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& event_manager) override;
  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:

  gz::sim::Model _model;
  std::ofstream _file;

  enum class ExportFormat
  {
    CSV,
    TUM
  } _export_format = ExportFormat::CSV;
};

} // end namespace simulation
} // end namespace eduart
