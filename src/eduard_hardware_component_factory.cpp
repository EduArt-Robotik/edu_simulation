#include "edu_simulation/eduard_hardware_component_factory.hpp"
#include "edu_simulation/gazebo_motor_controller.hpp"
#include "edu_simulation/gazebo_lighting.hpp"
#include "edu_simulation/gazebo_range_sensor.hpp"
#include "edu_simulation/gazebo_imu_sensor.hpp"
#include "edu_simulation/gazebo_hardware_adapter.hpp"

#include <gazebo/sensors/sensors.hh>

namespace eduart {
namespace simulation {

// static std::string get_full_name(sdf::ElementPtr sdf)
// {
//   std::string name = sdf->GetAttribute("name")->GetAsString();

//   for (sdf::ElementPtr element = sdf->GetParent(); element != nullptr; element = element->GetParent()) {
//     name = element->GetAttribute("name")->GetAsString() + "::" + name;
//   }

//   return name;
// }

// static std::string get_joint_name(sdf::ElementPtr sdf, const std::string& motor_name)
// {
//   for (auto joint = sdf->GetElement("joint"); joint != nullptr; joint = joint->GetNextElement("joint")) {
//     if (joint->GetElement("child")->GetValue()->GetAsString() == motor_name) {
//       return joint->GetAttribute("name")->GetAsString();
//     }
//   }

//   throw std::invalid_argument("Not joint found with child tag that matches given motor name.");
// }

EduardHardwareComponentFactory::EduardHardwareComponentFactory(
  std::shared_ptr<GazeboHardwareAdapter> hardware_adapter, const gz::sim::Entity& entity,
  const std::shared_ptr<const sdf::Element>& sdf, gz::sim::EntityComponentManager& ecm, rclcpp::Node& ros_node)
{
  for (auto link = sdf->GetElement("link"); link != nullptr; link = link->GetNextElement("link")) {
    const auto& link_name = link->GetAttribute("name")->GetAsString();

    // Motor Controller
    if (link_name.find("motor") != std::string::npos) {
      const auto joint_name = get_joint_name(sdf, link_name);
      const auto model_name = parent->GetName();
      const bool is_mecanum = model_name.find("mecanum") != std::string::npos;

      auto motor_joint = parent->GetJoint(joint_name);
      auto motor_controller_hardware = std::make_shared<GazeboMotorController>(
        link_name, parent, motor_joint, is_mecanum
      );

      _motor_controller_hardware.push_back(motor_controller_hardware);
      hardware_adapter->registerMotorController(motor_controller_hardware);
    }
    // Range Sensor
    // else if (link_name.find("range") != std::string::npos) {
    //   auto sensor_sdf = link->GetElement("sensor");

    //   if (sensor_sdf == nullptr) {
    //     throw std::invalid_argument("Range sensor requires a sensor tag.");
    //   }

    //   const auto sensor_name = sensor_sdf->GetAttribute("name")->GetAsString();
    //   auto sensor = gazebo::sensors::get_sensor(get_full_name(sensor_sdf));

    //   if (sensor == nullptr) {
    //     throw std::runtime_error("No sensor found in simulation. Actually this should not happen!");
    //   }

    //   std::cout << "add range sensor hardware: " << sensor_name << std::endl;
    //   _hardware[sensor_name] = std::make_shared<GazeboRangeSensor>(sensor, ros_node);
    // }
    // // IMU Sensor
    // else if (link_name.find("imu") != std::string::npos) {
    //   auto sensor_sdf = link->GetElement("sensor");

    //   if (sensor_sdf == nullptr) {
    //     throw std::invalid_argument("Range sensor requires a sensor tag.");
    //   }

    //   const auto sensor_name = sensor_sdf->GetAttribute("name")->GetAsString();
    //   auto sensor = gazebo::sensors::get_sensor(get_full_name(sensor_sdf));

    //   if (sensor == nullptr) {
    //     throw std::runtime_error("No sensor found in simulation. Actually this should not happen!");
    //   }

    //   std::cout << "add imu sensor hardware: " << sensor_name << std::endl;
    //   _hardware[sensor_name] = std::make_shared<GazeboImuSensor>(sensor, ros_node);
    // }
  }

  _hardware["head"] = std::make_shared<GazeboLighting>();
  _hardware["right_side"] = std::make_shared<GazeboLighting>();
  _hardware["left_side"] = std::make_shared<GazeboLighting>();
  _hardware["back"] = std::make_shared<GazeboLighting>();
  _hardware["all"] = std::make_shared<GazeboLighting>();
}

} // end namespace simulation
} // end namespace eduart
