#include "edu_simulation/object_detection_sensor.hpp"

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include <cstddef>
#include <gazebo/sensors/LogicalCameraSensor.hh>

#include <gazebo_ros/node.hpp>

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <ignition/math/Quaternion.hh>
#include <memory>
#include <rclcpp/qos.hpp>
#include <string>

namespace eduart {
namespace simulation {

ObjectDetectionSensor::ObjectDetectionSensor()
  : gazebo::SensorPlugin()
{

}

ObjectDetectionSensor::~ObjectDetectionSensor()
{
 
}

void ObjectDetectionSensor::Init()
{
  gazebo::SensorPlugin::Init();

}

void ObjectDetectionSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  auto logical_camera = std::dynamic_pointer_cast<gazebo::sensors::LogicalCameraSensor>(_sensor);

  if (logical_camera == nullptr) {
    std::cout << "nix wars" << std::endl;
    return;
  }

  _on_update_connection = _sensor->ConnectUpdated(std::bind(&ObjectDetectionSensor::OnUpdate, this));
  _logical_camera = logical_camera;
  _logical_camera->SetActive(true);

  _filter_string = _sdf->Get<std::string>("filter_string", "").first;
  _frame_id = _sdf->Get<std::string>("frame_id", "object_sensor").first;

  _ros_node = gazebo_ros::Node::Get(_sdf);
  _pub_object_pose = _ros_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "object/pose", rclcpp::QoS(1).best_effort()
  );
}

void ObjectDetectionSensor::OnUpdate()
{
  std::cout << "found objects = " << _logical_camera->Image().model_size() << std::endl;

  for (std::size_t i = 0; i < _logical_camera->Image().model_size(); ++i) {
    std::cout << "object " << i << ": " << _logical_camera->Image().model(i).name() << std::endl;
  }

  const auto& objects = _logical_camera->Image();

  for (std::size_t i = 0; i < objects.model_size(); ++i) {
    if (objects.model(i).name().find(_filter_string) != std::string::npos) {
      const auto model_pose = objects.model(i).pose();
      geometry_msgs::msg::PoseStamped msg;
      const ignition::math::Vector3d position_model(model_pose.position().x(), model_pose.position().y(), model_pose.position().z());
      const ignition::math::Quaterniond orientation_model(model_pose.orientation().w(), model_pose.orientation().x(), model_pose.orientation().y(), model_pose.orientation().z());

      msg.header.stamp = _ros_node->get_clock()->now();
      msg.header.frame_id = _ros_node->get_name();

      const auto relative_position = position_model - _logical_camera->Pose().Pos();
      const auto relative_orientation = orientation_model * _logical_camera->Pose().Rot().Inverse();

      msg.pose.position.x = relative_position.X();
      msg.pose.position.y = relative_position.Y();
      msg.pose.position.z = relative_position.Z();

      msg.pose.orientation.w = relative_orientation.W();
      msg.pose.orientation.x = relative_orientation.X();
      msg.pose.orientation.y = relative_orientation.Y();
      msg.pose.orientation.z = relative_orientation.Z();

      _pub_object_pose->publish(msg);
    }
  }
}

extern "C" GZ_PLUGIN_VISIBLE gazebo::SensorPlugin * RegisterPlugin();
gazebo::SensorPlugin * RegisterPlugin()
{
  return (gazebo::SensorPlugin *)(new ObjectDetectionSensor());
}

} // end namespace simulation
} // end namespace eduart

// GZ_REGISTER_SENSOR_PLUGIN(eduart::simulation::ObjectDetectionSensor)
