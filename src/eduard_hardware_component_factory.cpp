#include "edu_simulation/eduard_hardware_component_factory.hpp"
#include "edu_simulation/gazebo_motor_controller.hpp"
#include "edu_simulation/gazebo_lighting.hpp"
#include "edu_simulation/gazebo_range_sensor.hpp"
#include "edu_simulation/gazebo_imu_sensor.hpp"

namespace eduart {
namespace simulation {

EduardHardwareComponentFactory::EduardHardwareComponentFactory(sdf::ElementPtr sdf)
{
  for (auto link = sdf->GetElement("link"); link != nullptr; link = link->GetNextElement("link")) {
    const auto& link_name = link->GetAttribute("name")->GetAsString();

    if (link_name.find("motor") != std::string::npos) {
      auto motor_controller_hardware = std::make_shared<GazeboMotorController>();

      _motor_controller_hardware[link_name] = motor_controller_hardware;
      _motor_sensor_hardware[link_name] = motor_controller_hardware;
    }
  }

  _lighting_hardware["head"] = std::make_shared<GazeboLighting>();
  _lighting_hardware["right_side"] = std::make_shared<GazeboLighting>();
  _lighting_hardware["left_side"] = std::make_shared<GazeboLighting>();
  _lighting_hardware["back"] = std::make_shared<GazeboLighting>();
  _lighting_hardware["all"] = std::make_shared<GazeboLighting>();

  _range_sensor_hardware["range/front/left"] = std::make_shared<GazeboRangeSensor>();
  _range_sensor_hardware["range/front/right"] = std::make_shared<GazeboRangeSensor>();
  _range_sensor_hardware["range/rear/left"] = std::make_shared<GazeboRangeSensor>();
  _range_sensor_hardware["range/rear/right"] = std::make_shared<GazeboRangeSensor>();

  _imu_sensor_hardware["imu"] = std::make_shared<GazeboImuSensor>();
}

} // end namespace simulation
} // end namespace eduart
