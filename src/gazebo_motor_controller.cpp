#include "edu_simulation/gazebo_motor_controller.hpp"

namespace eduart {
namespace simulation {

GazeboMotorController::GazeboMotorController(const std::string& name)
  : robot::MotorController::HardwareInterface(name, 1)
  , _low_pass_filter({0.2f})
  , _measured_rpm(1, 0.0)
{

}

GazeboMotorController::~GazeboMotorController()
{

}

void GazeboMotorController::processSetValue(const std::vector<robot::Rpm>& rpm)
{
  if (rpm.size() != 1) {
    throw std::invalid_argument("GazeboMotorController::processSetValue(): given rpm vector must have size 1.");
  }

  _measured_rpm[0] = _low_pass_filter(rpm[0]);
  _callback_process_measurement(_measured_rpm, true);
}

void GazeboMotorController::initialize(const robot::Motor::Parameter &parameter)
{
  (void)parameter;
}

} // end namespace simulation
} // end namespace eduart
