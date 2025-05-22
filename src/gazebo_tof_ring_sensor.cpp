#include "edu_simulation/gazebo_tof_ring_sensor.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <functional>
#include <string>

namespace eduart {
namespace simulation {

static std::string get_virtual_range_sensor_name(const std::string& tof_sensor_name)
{
  const std::string removed_prefix(tof_sensor_name, tof_sensor_name.find(".")); // remove tof prefix
  const std::string sensor_name_with_dots = "range" + removed_prefix;
  std::string range_sensor_name = sensor_name_with_dots;

  for (auto& character : range_sensor_name) {
    if (character == '.') {
      character = '/';
    }
  }

  return range_sensor_name;
}

static std::shared_ptr<sensor_msgs::msg::PointCloud2> create_point_cloud(
  const SensorTofRingHardware::Parameter& parameter)
{
  std::size_t number_of_points = 0;

  for (const auto& sensor : parameter.tof_sensor) {
    number_of_points += sensor.parameter.number_of_zones.horizontal * sensor.parameter.number_of_zones.vertical;
  }

  // Preparing Point Cloud
  auto point_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  point_cloud->header.frame_id = "unkown";
  point_cloud->width = 0;
  point_cloud->height = 1;
  point_cloud->is_bigendian = false;
  point_cloud->point_step = 4 * sizeof(float);
  point_cloud->row_step = point_cloud->point_step * point_cloud->width;
  point_cloud->data.reserve(number_of_points * point_cloud->point_step);

  sensor_msgs::msg::PointField point_field;
  point_field.name = "x";
  point_field.offset = 0;
  point_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_field.count = 1;
  point_cloud->fields.push_back(point_field);

  point_field.name = "y";
  point_field.offset = 4;
  point_cloud->fields.push_back(point_field);

  point_field.name = "z";
  point_field.offset = 8;
  point_cloud->fields.push_back(point_field);

  point_field.name = "sigma";
  point_field.offset = 12;
  point_cloud->fields.push_back(point_field);

  return point_cloud;
}

SensorTofRingHardware::Parameter SensorTofRingHardware::get_parameter(
  const std::string& name, const std::vector<std::string>& sensor_names, rclcpp::Node& ros_node)
{
  Parameter parameter;
  SensorTofHardware::Parameter sensor_parameter;
  const std::string side = name.find("right") != std::string::npos ? "right" : "left"; // \todo only works if right or left is contained in name

  for (const auto& sensor_name : sensor_names) {
    parameter.tof_sensor.push_back({
      SensorTofHardware::get_parameter(
        name + '.' + sensor_name, sensor_parameter, ros_node
      ),
      "tof." + sensor_name + "." + side,
      "none",
      robot::Sensor::get_transform_from_parameter(name + '.' + sensor_name, ros_node),
      false
    });

    // virtual range sensor handling
    ros_node.declare_parameter<bool>(
      name + '.' + sensor_name + ".virtual_range_sensor", parameter.tof_sensor.back().virtual_range_sensor);
    parameter.tof_sensor.back().virtual_range_sensor = ros_node.get_parameter(name + '.' + sensor_name + ".virtual_range_sensor").as_bool();
  }

  return parameter;
}

SensorTofRingHardware::SensorTofRingHardware(
  const std::string& name, const Parameter& parameter, rclcpp::Node& ros_node)
  : _parameter(parameter)
  , _ros_node(ros_node)
{
  (void)name;

  for (std::size_t i = 0; i < _parameter.tof_sensor.size(); ++i) {
    // add virtual range sensor if wanted
    std::shared_ptr<SensorVirtualRange> virtual_range_sensor = nullptr;

    if (_parameter.tof_sensor[i].virtual_range_sensor) {
      virtual_range_sensor = std::make_shared<SensorVirtualRange>(_parameter.tof_sensor[i].transform);
      std::string range_sensor_name = get_virtual_range_sensor_name(_parameter.tof_sensor[i].name);
      _virtual_range_sensor[range_sensor_name] = virtual_range_sensor;
    }

    // instantiate and register tof sensor
    _sensor.emplace_back(std::make_shared<SensorTofHardware>(
      _parameter.tof_sensor[i].name, _parameter.tof_sensor[i].parameter, _ros_node,
      _parameter.tof_sensor[i].gz_feedback_topic_name, virtual_range_sensor
    ));
    _sensor.back()->registerCallbackProcessMeasurementData(
      std::bind(&SensorTofRingHardware::processPointcloudMeasurement, this, std::placeholders::_1, i)
    );
  }
}

void SensorTofRingHardware::initialize(const robot::SensorPointCloud::Parameter& parameter)
{
  for (auto& tof_sensor : _sensor) {
    tof_sensor->initialize(parameter);
  }

  _processing_data.point_cloud = create_point_cloud(_parameter);
  _processing_data.point_cloud->width = 0;
  _processing_data.point_cloud->height = 1;
  _processing_data.received_points.resize(_parameter.number_sensors(), false);
}

void SensorTofRingHardware::processPointcloudMeasurement(
  sensor_msgs::msg::PointCloud2& point_cloud, const std::size_t sensor_index)
{
  // Note: expect the given point cloud is of exact same type as sensor point cloud.

  // Transform given point cloud into sensor frame.
  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(_parameter.tof_sensor[sensor_index].transform);
  sensor_msgs::msg::PointCloud2 point_cloud_transformed;
  tf2::doTransform(point_cloud, point_cloud_transformed, transform);

  // Copy given point cloud into sensor point cloud.
  _processing_data.point_cloud->width += point_cloud.width * point_cloud.height;
  _processing_data.point_cloud->data.insert(
    _processing_data.point_cloud->data.end(),
    point_cloud_transformed.data.begin(),
    point_cloud_transformed.data.end()
  );
  _processing_data.received_points[sensor_index] = true;

  for (const auto received : _processing_data.received_points) {
    if (received == false) {
      // Measurement no finished --> do nothing
      return;
    }
  }

  // Measurement finished --> publish point cloud.
  _processing_data.point_cloud->header.stamp = _ros_node.get_clock()->now();
  _callback_process_measurement(*_processing_data.point_cloud);
  // TODO:  Clarify behavior when one or more tof sensors are missing.
  //        Data could still be published?

  // Prepare new iteration
  _processing_data.point_cloud->data.clear();
  _processing_data.point_cloud->width = 0;
  _processing_data.point_cloud->height = 1;
  std::fill(_processing_data.received_points.begin(), _processing_data.received_points.end(), false);
}

} // end namespace simulation
} // end namespace robot
