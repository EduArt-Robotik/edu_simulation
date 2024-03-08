/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/sensor_range.hpp>

#include <gazebo/plugins/RayPlugin.hh>

namespace eduart {
namespace simulation {

class GazeboRangeSensor : public robot::SensorRange::SensorInterface
{
public:
  GazeboRangeSensor(gazebo::sensors::SensorPtr sensor, rclcpp::Node& ros_node);
  ~GazeboRangeSensor() override;

  void initialize(const robot::SensorRange::Parameter& parameter) override;

private:
  void getMeasurement();

  gazebo::sensors::RaySensorPtr _sensor;
  std::shared_ptr<rclcpp::TimerBase> _timer_get_measurement;
};

} // end namespace simulation
} // end namespace eduart
