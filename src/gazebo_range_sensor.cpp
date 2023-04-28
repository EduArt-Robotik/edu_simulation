#include "edu_simulation/gazebo_range_sensor.hpp"

#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/MultiRayShape.hh>

namespace eduart {
namespace simulation {

using namespace std::chrono_literals;

GazeboRangeSensor::GazeboRangeSensor(gazebo::sensors::SensorPtr sensor, rclcpp::Node& ros_node)
  : _sensor(std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(sensor))
  , _timer_get_measurement(ros_node.create_wall_timer(
      100ms, std::bind(&GazeboRangeSensor::getMeasurement, this))
    )
{
  if (_sensor == nullptr) {
    throw std::invalid_argument("Range sensor requires a ray sensor as its parent.");
  }
  _sensor->SetActive(true);
}

GazeboRangeSensor::~GazeboRangeSensor()
{

}

void GazeboRangeSensor::initialize(const robot::RangeSensor::Parameter &parameter)
{
  (void)parameter;
}

void GazeboRangeSensor::getMeasurement()
{
  auto world = gazebo::physics::get_world();
  const gazebo::common::Time cur_time = world->SimTime();

  // Find ray with minimal distance.
  const std::size_t num_ranges = _sensor->LaserShape()->GetSampleCount() * _sensor->LaserShape()->GetVerticalSampleCount();
  double ray_distance = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < num_ranges; ++i) {
    ray_distance = std::min(_sensor->LaserShape()->GetRange(i), ray_distance);
  }

  if (_callback_process_measurement == nullptr) {
    return;
  }

  _sensor->SetActive(true);
  _callback_process_measurement(ray_distance);  
}

} // end namespace simulation
} // end namespace eduart
