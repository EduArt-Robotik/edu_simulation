/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_component_interface.hpp>
#include <edu_robot/imu_sensor.hpp>

namespace eduart {
namespace simulation {

class GazeboImuSensor : public robot::ImuSensor::SensorInterface
{
public:
  GazeboImuSensor();
  ~GazeboImuSensor() override;

  void initialize(const robot::ImuSensor::Parameter& parameter) override;  
};

} // end namespace simulation
} // end namespace eduart
