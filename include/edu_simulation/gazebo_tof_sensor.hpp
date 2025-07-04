/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/sensor_point_cloud.hpp>

#include "edu_simulation/gazebo_virtual_range_sensor.hpp"

#include <gz/msgs.hh>
#include <gz/transport/Node.hh>

#include <cstddef>

namespace eduart {
namespace simulation {

class SensorTofHardware : public robot::SensorPointCloud::SensorInterface
{
public:
  using MeasurementCompleteCallback = std::function<void()>;
  struct Parameter {
    struct {
      std::size_t vertical = 8;
      std::size_t horizontal = 8;
    } number_of_zones; // number of sensor zones (pixel)
    struct {
      robot::Angle vertical = robot::Angle::createFromDegree(45);
      robot::Angle horizontal = robot::Angle::createFromDegree(45);
    } fov;
  };

  SensorTofHardware(
    const std::string& name, const Parameter& parameter, rclcpp::Node& ros_node,
    const std::string& gz_feedback_topic_name, std::shared_ptr<SensorVirtualRange> virtual_range_sensor = nullptr);
  ~SensorTofHardware() override = default;

  void initialize(const robot::SensorPointCloud::Parameter& parameter) override;
  static Parameter get_parameter(const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node);
  inline void registerMeasurementCompleteCallback(MeasurementCompleteCallback callback){
    _callback_finished_measurement = callback;
  }

private:
  void processMeasurement(const gz::msgs::PointCloudPacked& cloud);

  const Parameter _parameter;
  rclcpp::Node& _ros_node;
  std::shared_ptr<gz::transport::Node> _node;
  MeasurementCompleteCallback _callback_finished_measurement = nullptr;
  std::shared_ptr<SensorVirtualRange> _virtual_range_sensor = nullptr;

  struct { 
    std::size_t number_of_zones;
    std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud;
  } _processing_data;
};                              

} // end namespace simulation
} // end namespace eduart
