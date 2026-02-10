#include "edu_simulation/gazebo_pose_exporter.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>

namespace eduart {
namespace simulation {

PoseExporter::PoseExporter()
{

}

PoseExporter::~PoseExporter()
{
  if (_file.is_open()) {
    _file.close();
  }
}

void PoseExporter::Configure(
  const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, 
  gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& event_manager)
{
  _model = gz::sim::Model(entity);

  if (const auto element = sdf->FindElement("export_format"); element != nullptr) {
    const auto format_string = element->GetAttribute("type")->GetAsString();
    if (format_string == "csv") {
      _export_format = ExportFormat::CSV;
      gzlog << "export format set to CSV" << std::endl;
    }
    else if (format_string == "tum") {
      _export_format = ExportFormat::TUM;
      gzlog << "export format set to TUM" << std::endl;
    }
    else {
      gzlog << "unknown export format: " << format_string << ", using default (CSV)" << std::endl;
    }
  }

  if (const auto element = sdf->FindElement("file"); element != nullptr) {
    const auto file_name = element->GetAttribute("name")->GetAsString();
    _file.open(file_name, std::ios::out);

    if (_file.is_open() == false) {
      gzerr << "could not open file: " << file_name << std::endl;
    }
    else {
      gzlog << "export poses to file: " << file_name << std::endl;
      if (_export_format == ExportFormat::CSV) {
        _file << "time,position_x,position_y,position_z,orientation_x,orientation_y,orientation_z,orientation_w" << std::endl;
      }
    }
  }
}

void PoseExporter::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm)
{
  if (_file.is_open() == false) {
    return;
  }

  const auto pose = gz::sim::worldPose(_model.Entity(), ecm);
  const auto time = info.simTime;

  if (_export_format == ExportFormat::CSV) {
    _file << std::chrono::duration<double>(time).count() << ","
          << pose.Pos().X() << ","
          << pose.Pos().Y() << ","
          << pose.Pos().Z() << ","
          << pose.Rot().X() << ","
          << pose.Rot().Y() << ","
          << pose.Rot().Z() << ","
          << pose.Rot().W() << std::endl;
  }
  else if (_export_format == ExportFormat::TUM) {
    _file << std::chrono::duration<double>(time).count() << " "
          << pose.Pos().X() << " "
          << pose.Pos().Y() << " "
          << pose.Pos().Z() << " "
          << pose.Rot().X() << " "
          << pose.Rot().Y() << " "
          << pose.Rot().Z() << " "
          << pose.Rot().W() << std::endl;
  }
}

} // end namespace simulation
} // end namespace eduart

GZ_ADD_PLUGIN(
  eduart::simulation::PoseExporter,
  gz::sim::System,
  eduart::simulation::PoseExporter::ISystemConfigure,
  eduart::simulation::PoseExporter::ISystemPostUpdate
)