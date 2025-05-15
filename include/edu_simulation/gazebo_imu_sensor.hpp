/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/sensor_imu.hpp>

#include <gz/transport.hh>
#include <gz/msgs/details/imu.pb.h>

namespace eduart {
namespace simulation {

class GazeboImuSensor : public robot::SensorImu::SensorInterface
{
public:
  GazeboImuSensor(const std::string& model_name, rclcpp::Node& ros_node);
  ~GazeboImuSensor() override;

  void initialize(const robot::SensorImu::Parameter& parameter) override;

private:
  void receiveMeasurement(const gz::msgs::IMU& measurement);

  std::shared_ptr<gz::transport::Node> _gz_node;
  std::string _model_name;
};

} // end namespace simulation
} // end namespace eduart
