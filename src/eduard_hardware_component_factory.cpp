#include "edu_simulation/eduard_hardware_component_factory.hpp"
#include "edu_simulation/gazebo_motor_controller.hpp"
#include "edu_simulation/gazebo_lighting.hpp"
#include "edu_simulation/gazebo_range_sensor.hpp"
#include "edu_simulation/gazebo_imu_sensor.hpp"

#include <gazebo/sensors/sensors.hh>

namespace eduart {
namespace simulation {

static std::string get_full_name(sdf::ElementPtr sdf)
{
  std::string name = sdf->GetAttribute("name")->GetAsString();

  for (sdf::ElementPtr element = sdf->GetParent(); element != nullptr; element = element->GetParent()) {
    name = element->GetAttribute("name")->GetAsString() + "::" + name;
  }

  return name;
}

EduardHardwareComponentFactory::EduardHardwareComponentFactory(
  gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf, rclcpp::Node& ros_node)
{
  for (auto link = sdf->GetElement("link"); link != nullptr; link = link->GetNextElement("link")) {
    const auto& link_name = link->GetAttribute("name")->GetAsString();

    // Motor Controller
    if (link_name.find("motor") != std::string::npos) {
      auto motor_controller_hardware = std::make_shared<GazeboMotorController>();

      _motor_controller_hardware[link_name] = motor_controller_hardware;
      _motor_sensor_hardware[link_name] = motor_controller_hardware;
    }
    // Range Sensor
    else if (link_name.find("range") != std::string::npos) {
      auto sensor_sdf = link->GetElement("sensor");

      if (sensor_sdf == nullptr) {
        throw std::invalid_argument("Range sensor requires a sensor tag.");
      }

      const auto sensor_name = sensor_sdf->GetAttribute("name")->GetAsString();
      auto sensor = gazebo::sensors::get_sensor(get_full_name(sensor_sdf));

      if (sensor == nullptr) {
        throw std::runtime_error("No sensor found in simulation. Actually this should not happen!");
      }

      std::cout << "add range sensor hardware: " << sensor_name << std::endl;
      _range_sensor_hardware[sensor_name] = std::make_shared<GazeboRangeSensor>(sensor, ros_node);
    }
    // IMU Sensor
    else if (link_name.find("imu") != std::string::npos) {
      auto sensor_sdf = link->GetElement("sensor");

      if (sensor_sdf == nullptr) {
        throw std::invalid_argument("Range sensor requires a sensor tag.");
      }

      const auto sensor_name = sensor_sdf->GetAttribute("name")->GetAsString();
      auto sensor = gazebo::sensors::get_sensor(get_full_name(sensor_sdf));

      if (sensor == nullptr) {
        throw std::runtime_error("No sensor found in simulation. Actually this should not happen!");
      }

      std::cout << "add imu sensor hardware: " << sensor_name << std::endl;
      _imu_sensor_hardware[sensor_name] = std::make_shared<GazeboImuSensor>(sensor, ros_node);
    }
  }

  _lighting_hardware["head"] = std::make_shared<GazeboLighting>();
  _lighting_hardware["right_side"] = std::make_shared<GazeboLighting>();
  _lighting_hardware["left_side"] = std::make_shared<GazeboLighting>();
  _lighting_hardware["back"] = std::make_shared<GazeboLighting>();
  _lighting_hardware["all"] = std::make_shared<GazeboLighting>();


  // _imu_sensor_hardware["imu"] = std::make_shared<GazeboImuSensor>();
}

} // end namespace simulation
} // end namespace eduart
