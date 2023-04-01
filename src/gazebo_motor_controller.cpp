#include "edu_simulation/gazebo_motor_controller.hpp"

namespace eduart {
namespace simulation {

GazeboMotorController::GazeboMotorController()
{

}

GazeboMotorController::~GazeboMotorController()
{

}

void GazeboMotorController::processSetValue(const robot::Rpm &rpm)
{
  _callback_process_measurement(rpm);  
}

void GazeboMotorController::initialize(const robot::MotorController::Parameter &parameter)
{

}

} // end namespace simulation
} // end namespace eduart
