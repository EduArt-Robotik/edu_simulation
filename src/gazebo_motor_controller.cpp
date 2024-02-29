#include "edu_simulation/gazebo_motor_controller.hpp"

namespace eduart {
namespace simulation {

GazeboMotorController::GazeboMotorController(const std::string& name)
  : robot::MotorController::HardwareInterface(name, 1)
{

}

GazeboMotorController::~GazeboMotorController()
{

}

void GazeboMotorController::processSetValue(const std::vector<robot::Rpm>& rpm)
{
  _callback_process_measurement(rpm, true);
}

void GazeboMotorController::initialize(const robot::Motor::Parameter &parameter)
{
  (void)parameter;
}

} // end namespace simulation
} // end namespace eduart
