#include "edu_simulation/eduard_hardware_component_factory.hpp"
#include "edu_simulation/gazebo_motor_controller.hpp"
#include "edu_simulation/gazebo_lighting.hpp"
#include "edu_simulation/gazebo_imu_sensor.hpp"
#include "edu_simulation/gazebo_hardware_adapter.hpp"
#include "edu_simulation/gazebo_tof_ring_sensor.hpp"

#include <edu_robot/hardware/can_gateway/sensor_point_cloud_fusion.hpp>

#include <gz/sim/Model.hh>

namespace eduart {
namespace simulation {

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

  // Motor Controller
  for (auto motor = sdf->FindElement("motor"); motor != nullptr; motor = motor->GetNextElement("motor")) {
    const auto& name = motor->GetAttribute("name")->GetAsString();
    const auto& joint_name = motor->GetAttribute("joint_name")->GetAsString();
    
    std::cout << "motor name = " << name << std::endl;
    std::cout << "joint name = " << joint_name << std::endl;
  
    auto motor_controller_hardware = std::make_shared<GazeboMotorController>(
      name,
      model_name + "/" + joint_name + "/" + "set_joint_velocity",
      model_name + "/" + joint_name + "/" + "get_joint_velocity"
    );

    _motor_controller_hardware.push_back(motor_controller_hardware);
    hardware_adapter->registerMotorController(motor_controller_hardware);
  }
  // Tof and Range Sensor
  const std::vector<std::string> tof_sensors_left  = {"front", "rear"};
  const std::vector<std::string> tof_sensors_right = {"front", "rear"};

  // Left Ring
  auto parameter_left = SensorTofRingHardware::get_parameter(
    "tof_sensor_ring_left", tof_sensors_left, ros_node
  );

  parameter_left.tof_sensor[0].name = "range.front.left";
  parameter_left.tof_sensor[0].virtual_range_sensor = true;
  parameter_left.tof_sensor[0].gz_feedback_topic_name = model_name + "/range/front/left/points";
  parameter_left.tof_sensor[0].transform.setOrigin({0.17, 0.063, 0.045});
  parameter_left.tof_sensor[0].transform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  parameter_left.tof_sensor[1].name = "range.rear.left";
  parameter_left.tof_sensor[1].virtual_range_sensor = true;
  parameter_left.tof_sensor[1].gz_feedback_topic_name = model_name + "/range/rear/left/points";
  parameter_left.tof_sensor[1].transform.setOrigin({-0.17, 0.063, 0.05});
  parameter_left.tof_sensor[1].transform.setRotation(tf2::Quaternion(0.0, 0.0, 1.0, 0.0));

  auto left_ring = std::make_shared<SensorTofRingHardware>("tof_sensor_ring_left", parameter_left, ros_node);

  // Right Ring
  auto parameter_right = SensorTofRingHardware::get_parameter(
    "tof_sensor_ring_right", tof_sensors_right, ros_node
  );

  parameter_right.tof_sensor[0].name = "range.front.right";
  parameter_right.tof_sensor[0].virtual_range_sensor = true;
  parameter_right.tof_sensor[0].gz_feedback_topic_name = model_name + "/range/front/right/points";
  parameter_right.tof_sensor[0].transform.setOrigin({0.17, -0.063, 0.045});
  parameter_right.tof_sensor[0].transform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  parameter_right.tof_sensor[1].name = "range.rear.right";
  parameter_right.tof_sensor[1].virtual_range_sensor = true;
  parameter_right.tof_sensor[1].gz_feedback_topic_name = model_name + "/range/rear/right/points";
  parameter_right.tof_sensor[1].transform.setOrigin({-0.17, -0.063, 0.05});
  parameter_right.tof_sensor[1].transform.setRotation(tf2::Quaternion(0.0, 0.0, 1.0, 0.0));

  auto right_ring = std::make_shared<SensorTofRingHardware>("tof_sensor_ring_right", parameter_right, ros_node);
  std::vector<std::shared_ptr<robot::SensorPointCloud::SensorInterface>> ring = { left_ring, right_ring };
  _hardware["tof_sensor_ring"] = std::make_shared<robot::hardware::can_gateway::SensorPointCloudFusion>(ring);
  _hardware.insert(left_ring->virtualRangeSensor().begin(), left_ring->virtualRangeSensor().end());
  _hardware.insert(right_ring->virtualRangeSensor().begin(), right_ring->virtualRangeSensor().end());

  // IMU Sensor
  _hardware["imu"] = std::make_shared<GazeboImuSensor>(model_name, ros_node);


  // _hardware["range/front/left"] = std::make_shared<GazeboRangeSensor>(ros_node);
  // _hardware["range/front/right"] = std::make_shared<GazeboRangeSensor>(ros_node);
  // _hardware["range/rear/left"] = std::make_shared<GazeboRangeSensor>(ros_node);
  // _hardware["range/rear/right"] = std::make_shared<GazeboRangeSensor>(ros_node);


  _hardware["head"] = std::make_shared<GazeboLighting>();
  _hardware["right_side"] = std::make_shared<GazeboLighting>();
  _hardware["left_side"] = std::make_shared<GazeboLighting>();
  _hardware["back"] = std::make_shared<GazeboLighting>();
  _hardware["all"] = std::make_shared<GazeboLighting>();
}

} // end namespace simulation
} // end namespace eduart
