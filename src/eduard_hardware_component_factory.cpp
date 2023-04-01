#include "edu_simulation/eduard_hardware_component_factory.hpp"
#include "edu_simulation/gazebo_motor_controller.hpp"

namespace eduart {
namespace simulation {

EduardHardwareComponentFactory::EduardHardwareComponentFactory(sdf::ElementPtr sdf)
{
  for (auto link = sdf->GetElement("link"); link != nullptr; link = link->GetNextElement("link")) {
    const auto& link_name = link->GetAttribute("name")->GetAsString();

    if (link_name.find("wheel") != std::string::npos) {
      auto motor_controller_hardware = std::make_shared<GazeboMotorController>();

      _motor_controller_hardware[link_name] = motor_controller_hardware;
      _motor_sensor_hardware[link_name] = motor_controller_hardware;
    }
  }
}

} // end namespace simulation
} // end namespace eduart
