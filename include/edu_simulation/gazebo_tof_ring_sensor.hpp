/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_simulation/gazebo_tof_sensor.hpp"

#include <edu_robot/hardware/communicator_node.hpp>
#include <edu_robot/hardware/message_buffer.hpp>

#include <edu_robot/sensor_point_cloud.hpp>

#include <Eigen/Geometry>

#include <memory>

namespace eduart {
namespace simulation {

class SensorTofRingHardware : public robot::SensorPointCloud::SensorInterface
{
public:
  struct Parameter {
    struct TofSensor {
      SensorTofHardware::Parameter parameter;
      std::string name;
      std::string gz_feedback_topic_name;
      tf2::Transform transform;
      bool virtual_range_sensor = false;
    };
    std::vector<TofSensor> tof_sensor;

    inline std::size_t number_sensors() const { return tof_sensor.size(); };
  };

  SensorTofRingHardware(
    const std::string& name, const Parameter& parameter, rclcpp::Node& ros_node);
  ~SensorTofRingHardware() override = default;

  void initialize(const robot::SensorPointCloud::Parameter& parameter) override;
  static Parameter get_parameter(
    const std::string& name, const std::vector<std::string>& sensor_names, rclcpp::Node& ros_node);
  inline const Parameter& parameter() const { return _parameter; }
  inline const std::map<std::string, std::shared_ptr<HardwareInterface>>& virtualRangeSensor() const {
    return _virtual_range_sensor;
  }

private:
  void processPointcloudMeasurement(sensor_msgs::msg::PointCloud2& point_cloud, const std::size_t sensor_index);

  const Parameter _parameter;
  rclcpp::Node& _ros_node;
  std::vector<std::shared_ptr<SensorTofHardware>> _sensor;
  std::map<std::string, std::shared_ptr<HardwareInterface>> _virtual_range_sensor;

  struct {
    std::vector<bool> received_points;
    std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud;
  } _processing_data;
};

} // end namespace simulation
} // end namespace robot
