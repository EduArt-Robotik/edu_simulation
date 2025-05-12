/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/sensor_imu.hpp>

namespace eduart {
namespace simulation {

class GazeboImuSensor : public robot::SensorImu::SensorInterface
{
public:
  GazeboImuSensor(rclcpp::Node& ros_node);
  ~GazeboImuSensor() override;

  void initialize(const robot::SensorImu::Parameter& parameter) override;

private:
  void getMeasurement();

  std::shared_ptr<rclcpp::TimerBase> _timer_get_measurement;
};

} // end namespace simulation
} // end namespace eduart
