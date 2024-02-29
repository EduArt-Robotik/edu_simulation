#include "edu_simulation/gazebo_imu_sensor.hpp"

#include <gazebo/sensors/ImuSensor.hh>

namespace eduart {
namespace simulation {

using namespace std::chrono_literals;

GazeboImuSensor::GazeboImuSensor(gazebo::sensors::SensorPtr sensor, rclcpp::Node& ros_node)
  : _sensor(std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(sensor))
  , _timer_get_measurement(ros_node.create_wall_timer(
      100ms, std::bind(&GazeboImuSensor::getMeasurement, this))
    )
{
  if (_sensor == nullptr) {
    throw std::invalid_argument("Imu sensor requires a imu sensor as its parent.");
  }
  _sensor->SetActive(true);
}

GazeboImuSensor::~GazeboImuSensor()
{

}

void GazeboImuSensor::initialize(const robot::SensorImu::Parameter &parameter)
{
  (void)parameter;
}

void GazeboImuSensor::getMeasurement()
{
  if (_callback_process_measurement == nullptr) {
    return;
  }

  // \todo check sim time as measurement time
  // auto world = gazebo::physics::get_world();
  // const gazebo::common::Time cur_time = world->SimTime(); 
  const auto sensor_orientation = _sensor->Orientation();
  const Eigen::Quaterniond orientation(
    sensor_orientation.W(), sensor_orientation.X(), sensor_orientation.Y(), sensor_orientation.Z());
  const auto sensor_acceleration = _sensor->LinearAcceleration();
  const Eigen::Vector3d linear_acceleration(sensor_acceleration.X(), sensor_acceleration.Y(), sensor_acceleration.Z());
  const auto sensor_velocity = _sensor->AngularVelocity();
  const Eigen::Vector3d angular_velocity(sensor_velocity.X(), sensor_velocity.Y(), sensor_velocity.Z());

  _callback_process_measurement(orientation, angular_velocity, linear_acceleration);
}

} // end namespace simulation
} // end namespace eduart
