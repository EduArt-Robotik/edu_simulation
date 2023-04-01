/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/range_sensor.hpp>
#include <edu_robot/hardware_component_interface.hpp>

namespace eduart {
namespace simulation {

class GazeboRangeSensor : public robot::RangeSensor::SensorInterface
{
public:
  GazeboRangeSensor();
  ~GazeboRangeSensor() override;

  void initialize(const robot::RangeSensor::Parameter& parameter) override;  
};

} // end namespace simulation
} // end namespace eduart
