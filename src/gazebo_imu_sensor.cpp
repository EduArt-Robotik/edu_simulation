#include "edu_simulation/gazebo_imu_sensor.hpp"

namespace eduart {
namespace simulation {

GazeboImuSensor::GazeboImuSensor()
{

}

GazeboImuSensor::~GazeboImuSensor()
{

}

void GazeboImuSensor::initialize(const robot::ImuSensor::Parameter &parameter)
{
  (void)parameter;
}

} // end namespace simulation
} // end namespace eduart
