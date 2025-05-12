/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/sensor_range.hpp>

namespace eduart {
namespace simulation {

class GazeboRangeSensor : public robot::SensorRange::SensorInterface
{
public:
  GazeboRangeSensor(rclcpp::Node& ros_node);
  ~GazeboRangeSensor() override;

  void initialize(const robot::SensorRange::Parameter& parameter) override;

private:
  void getMeasurement();

  std::shared_ptr<rclcpp::TimerBase> _timer_get_measurement;
};

} // end namespace simulation
} // end namespace eduart
