#include "edu_simulation/gazebo_range_sensor.hpp"

namespace eduart {
namespace simulation {

using namespace std::chrono_literals;

GazeboRangeSensor::GazeboRangeSensor(rclcpp::Node& ros_node)
  : _timer_get_measurement(ros_node.create_wall_timer(
      100ms, std::bind(&GazeboRangeSensor::getMeasurement, this))
    )
{

}

GazeboRangeSensor::~GazeboRangeSensor()
{

}

void GazeboRangeSensor::initialize(const robot::SensorRange::Parameter &parameter)
{
  (void)parameter;
}

void GazeboRangeSensor::getMeasurement()
{
  // \todo check sim time as measurement time
  // auto world = gazebo::physics::get_world();
  // const gazebo::common::Time cur_time = world->SimTime();

  // Find ray with minimal distance.
  // const std::size_t num_ranges = _sensor->LaserShape()->GetSampleCount() * _sensor->LaserShape()->GetVerticalSampleCount();
  // double ray_distance = std::numeric_limits<double>::max();

  // for (std::size_t i = 0; i < num_ranges; ++i) {
  //   ray_distance = std::min(_sensor->LaserShape()->GetRange(i), ray_distance);
  // }

  // if (_callback_process_measurement == nullptr) {
  //   return;
  // }

  // _sensor->SetActive(true);
  // _callback_process_measurement(ray_distance);  
}

} // end namespace simulation
} // end namespace eduart
