#include "edu_simulation/gazebo_hardware_adapter.hpp"
#include "edu_simulation/gazebo_motor_controller.hpp"

namespace eduart {
namespace simulation {

GazeboHardwareAdapter::GazeboHardwareAdapter(sdf::ElementPtr sdf)
{
  (void)sdf;
}

GazeboHardwareAdapter::~GazeboHardwareAdapter()
{

}
  
void GazeboHardwareAdapter::enable()
{
  for (auto& motor_controller : _motor_controller) {
    motor_controller->enable();
  }
}

void GazeboHardwareAdapter::disable()
{
  for (auto& motor_controller : _motor_controller) {
    motor_controller->disable();
  }
}
  
robot::RobotStatusReport GazeboHardwareAdapter::getStatusReport()
{
  // \todo implement me!
  return { };
}

robot::diagnostic::Diagnostic GazeboHardwareAdapter::processDiagnosticsImpl()
{
  // \todo implement me!  
  return { };
}

} // end namespace simulation
} // end namespace eduart
