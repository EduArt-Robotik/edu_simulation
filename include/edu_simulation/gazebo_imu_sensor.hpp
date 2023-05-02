/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_component_interface.hpp>
#include <edu_robot/imu_sensor.hpp>

#include <gazebo/sensors/SensorTypes.hh>

namespace eduart {
namespace simulation {

class GazeboImuSensor : public robot::ImuSensor::SensorInterface
{
public:
  GazeboImuSensor(gazebo::sensors::SensorPtr sensor, rclcpp::Node& ros_node);
  ~GazeboImuSensor() override;

  void initialize(const robot::ImuSensor::Parameter& parameter) override;

private:
  void getMeasurement();

  gazebo::sensors::ImuSensorPtr _sensor;
  std::shared_ptr<rclcpp::TimerBase> _timer_get_measurement;
};

} // end namespace simulation
} // end namespace eduart
