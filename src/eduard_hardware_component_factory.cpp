#include "edu_simulation/eduard_hardware_component_factory.hpp"
#include "edu_simulation/gazebo_motor_controller.hpp"
#include "edu_simulation/gazebo_lighting.hpp"
#include "edu_simulation/gazebo_range_sensor.hpp"
#include "edu_simulation/gazebo_imu_sensor.hpp"
#include "edu_simulation/gazebo_hardware_adapter.hpp"

#include <gz/sim/Model.hh>

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

static std::string get_joint_name(const sdf::ElementConstPtr sdf, const std::string& motor_name)
{
  for (auto joint = sdf->FindElement("joint"); joint != nullptr; joint = joint->GetNextElement("joint")) {
    if (joint->GetNextElement("child")->GetValue()->GetAsString() == motor_name) {
      return joint->GetAttribute("name")->GetAsString();
    }
  }

  throw std::invalid_argument("Not joint found with child tag that matches given motor name.");
}

EduardHardwareComponentFactory::EduardHardwareComponentFactory(
  std::shared_ptr<GazeboHardwareAdapter> hardware_adapter, const gz::sim::Entity& entity,
  const std::shared_ptr<const sdf::Element>& sdf, gz::sim::EntityComponentManager& ecm, rclcpp::Node& ros_node)
{
  const auto model_name = gz::sim::Model(entity).Name(ecm);
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  std::cout << "name = " << sdf->GetName() << std::endl;
  for (auto element = sdf->FindElement(model_name); element != nullptr; element = sdf->GetNextElement()) {
    std::cout << "element name = " << sdf->GetName() << std::endl;
  }

  // Motor Controller
  for (auto motor = sdf->FindElement("motor"); motor != nullptr; motor = motor->GetNextElement("motor")) {
    const auto& name = motor->GetAttribute("name")->GetAsString();
    const auto& joint_name = motor->GetAttribute("joint_name")->GetAsString();
    
    std::cout << "motor name = " << model_name << std::endl;
    std::cout << "joint name = " << joint_name << std::endl;
  
    auto motor_controller_hardware = std::make_shared<GazeboMotorController>(
      name, model_name + "/" + joint_name + "/" + "set_joint_velocity"
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

  _hardware["range/front/left"] = std::make_shared<GazeboRangeSensor>(ros_node);
  _hardware["range/front/right"] = std::make_shared<GazeboRangeSensor>(ros_node);
  _hardware["range/rear/left"] = std::make_shared<GazeboRangeSensor>(ros_node);
  _hardware["range/rear/right"] = std::make_shared<GazeboRangeSensor>(ros_node);

  _hardware["imu"] = std::make_shared<GazeboImuSensor>(ros_node);

  _hardware["head"] = std::make_shared<GazeboLighting>();
  _hardware["right_side"] = std::make_shared<GazeboLighting>();
  _hardware["left_side"] = std::make_shared<GazeboLighting>();
  _hardware["back"] = std::make_shared<GazeboLighting>();
  _hardware["all"] = std::make_shared<GazeboLighting>();
}

} // end namespace simulation
} // end namespace eduart
