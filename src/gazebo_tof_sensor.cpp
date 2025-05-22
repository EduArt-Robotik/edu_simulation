#include "edu_simulation/gazebo_tof_sensor.hpp"
#include "edu_simulation/gazebo_virtual_range_sensor.hpp"

#include <rclcpp/logging.hpp>

#include <memory>
#include <limits>
#include <cstddef>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace eduart {
namespace simulation {

static std::shared_ptr<sensor_msgs::msg::PointCloud2> create_point_cloud(
  const SensorTofHardware::Parameter& parameter)
{
  // Preparing Point Cloud
  auto point_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  point_cloud->header.frame_id = "unkown";
  point_cloud->width = parameter.number_of_zones.horizontal;
  point_cloud->height = parameter.number_of_zones.vertical;
  point_cloud->is_bigendian = false;
  point_cloud->point_step = 4 * sizeof(float);
  point_cloud->row_step = point_cloud->point_step * point_cloud->width;
  point_cloud->data.resize(point_cloud->width * point_cloud->height * point_cloud->point_step);

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

static void set_nan_points(sensor_msgs::msg::PointCloud2& point_cloud)
{
  const std::size_t number_of_points = point_cloud.height * point_cloud.width;

  for (std::size_t point_idx = 0; point_idx < number_of_points; ++point_idx) {
    const std::size_t idx_point_x = point_idx * point_cloud.point_step + point_cloud.fields[0].offset;
    const std::size_t idx_point_y = point_idx * point_cloud.point_step + point_cloud.fields[1].offset;
    const std::size_t idx_point_z = point_idx * point_cloud.point_step + point_cloud.fields[2].offset;
    const std::size_t idx_sigma   = point_idx * point_cloud.point_step + point_cloud.fields[3].offset;

    *reinterpret_cast<float*>(&point_cloud.data[idx_point_x]) = std::numeric_limits<float>::quiet_NaN();
    *reinterpret_cast<float*>(&point_cloud.data[idx_point_y]) = std::numeric_limits<float>::quiet_NaN();
    *reinterpret_cast<float*>(&point_cloud.data[idx_point_z]) = std::numeric_limits<float>::quiet_NaN();
    *reinterpret_cast<float*>(&point_cloud.data[idx_sigma])   = std::numeric_limits<float>::quiet_NaN();    
  }
}

static void copy_point_cloud(sensor_msgs::msg::PointCloud2& out, const gz::msgs::PointCloudPacked& in)
{
  const std::size_t count_points = in.data().size() / in.point_step();


  for (std::size_t i = 0; i < count_points; ++i) {
    // \todo estimate field index by name
    const float x = *reinterpret_cast<const float*>(&in.data()[i * in.point_step() + in.field(0).offset()]);
    const float y = *reinterpret_cast<const float*>(&in.data()[i * in.point_step() + in.field(1).offset()]);
    const float z = *reinterpret_cast<const float*>(&in.data()[i * in.point_step() + in.field(2).offset()]);

    *reinterpret_cast<float*>(&out.data[i * out.point_step + out.fields[0].offset]) = x;
    *reinterpret_cast<float*>(&out.data[i * out.point_step + out.fields[1].offset]) = y;
    *reinterpret_cast<float*>(&out.data[i * out.point_step + out.fields[2].offset]) = z;
  }
}

SensorTofHardware::Parameter
SensorTofHardware::get_parameter(const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node)
{
  SensorTofHardware::Parameter parameter;

  // parameter declaration
  ros_node.declare_parameter<int>(
    name + ".number_of_zones.vertical", default_parameter.number_of_zones.vertical);
  ros_node.declare_parameter<int>(
    name + ".number_of_zones.horizontal", default_parameter.number_of_zones.horizontal);
  ros_node.declare_parameter<float>(name + ".fov.vertical", default_parameter.fov.vertical);
  ros_node.declare_parameter<float>(name + ".fov.horizontal", default_parameter.fov.horizontal);

  // parameter reading
  parameter.number_of_zones.vertical = ros_node.get_parameter(name + ".number_of_zones.vertical").as_int();
  parameter.number_of_zones.horizontal = ros_node.get_parameter(name + ".number_of_zones.horizontal").as_int();
  parameter.fov.vertical = ros_node.get_parameter(name + ".fov.vertical").as_double();
  parameter.fov.horizontal = ros_node.get_parameter(name + ".fov.horizontal").as_double();

  return parameter;
}

SensorTofHardware::SensorTofHardware(
  const std::string& name, const Parameter& parameter, rclcpp::Node& ros_node,
  const std::string& gz_feedback_topic_name, std::shared_ptr<SensorVirtualRange> virtual_range_sensor)
  : robot::SensorPointCloud::SensorInterface()
  , _parameter(parameter)
  , _ros_node(ros_node)
  , _node(std::make_shared<gz::transport::Node>())
  , _virtual_range_sensor(virtual_range_sensor)
{
  (void)name;

  // Subscribe to simulated sensor
  _node->Subscribe(gz_feedback_topic_name, &SensorTofHardware::processMeasurement, this);
}

// is called by the executer thread of rx data endpoint
void SensorTofHardware::processMeasurement(const gz::msgs::PointCloudPacked& cloud)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }
  if (_processing_data.point_cloud == nullptr) {
    // not initialized --> return
    return;
  }
  if (cloud.width() != _processing_data.point_cloud->width || cloud.height() != _processing_data.point_cloud->height) {
    RCLCPP_ERROR(rclcpp::get_logger("SensorTofHardware"), "point cloud size mismatch!");
    return;
  }

  copy_point_cloud(*_processing_data.point_cloud, cloud);
  _callback_process_measurement(*_processing_data.point_cloud);

  if (_virtual_range_sensor) {
    _virtual_range_sensor->processPointCloudMeasurement(*_processing_data.point_cloud);
  }
}

void SensorTofHardware::initialize(const robot::SensorPointCloud::Parameter& parameter)
{
  (void)parameter;

  // Preparing Processing Data
  _processing_data.number_of_zones = _parameter.number_of_zones.horizontal * _parameter.number_of_zones.vertical;
  _processing_data.point_cloud = create_point_cloud(_parameter);
  set_nan_points(*_processing_data.point_cloud);
}

} // end namespace simulation
} // end namespace eduart
