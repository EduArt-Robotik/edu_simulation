/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "gazebo/sensors/LogicalCameraSensor.hh"
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Plugin.hh>

#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace eduart {
namespace simulation {

class ObjectDetectionSensor : public gazebo::SensorPlugin
{
public:
  ObjectDetectionSensor();
  ~ObjectDetectionSensor() override;
  
  void Init() override;
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

private:
  void OnUpdate();

  std::shared_ptr<gazebo::sensors::LogicalCameraSensor> _logical_camera;
  gazebo::event::ConnectionPtr _on_update_connection;

  std::shared_ptr<rclcpp::Node> _ros_node;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> _pub_object_pose;
};

} // end namespace simulation
} // end namespace eduart
