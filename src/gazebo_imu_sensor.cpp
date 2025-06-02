#include "edu_simulation/gazebo_imu_sensor.hpp"

namespace eduart {
namespace simulation {

using namespace std::chrono_literals;

GazeboImuSensor::GazeboImuSensor(const std::string& name, const std::string& gz_topic_name)
  : _gz_node(std::make_shared<gz::transport::Node>())
{
  (void)name;
  _gz_node->Subscribe(gz_topic_name, &GazeboImuSensor::receiveMeasurement, this);
}

GazeboImuSensor::~GazeboImuSensor()
{

}

void GazeboImuSensor::initialize(const robot::SensorImu::Parameter &parameter)
{
  (void)parameter;
}

void GazeboImuSensor::receiveMeasurement(const gz::msgs::IMU& measurement)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }

  const auto sensor_orientation = measurement.orientation();
  const Eigen::Quaterniond orientation(
    sensor_orientation.w(), sensor_orientation.x(), sensor_orientation.y(), sensor_orientation.z());

  const auto sensor_acceleration = measurement.linear_acceleration();
  const Eigen::Vector3d linear_acceleration(sensor_acceleration.x(), sensor_acceleration.y(), sensor_acceleration.z());
  
  const auto sensor_velocity = measurement.angular_velocity();
  const Eigen::Vector3d angular_velocity(sensor_velocity.x(), sensor_velocity.y(), sensor_velocity.z());

  _callback_process_measurement(orientation, angular_velocity, linear_acceleration);
}

} // end namespace simulation
} // end namespace eduart
