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
  // return fake values
  return {
    37.0,
    {
      19.2,
      19.2
    },
    {
      0.6,
      0.6
    }
  };
}

robot::diagnostic::Diagnostic GazeboHardwareAdapter::processDiagnosticsImpl()
{
  // \todo implement me!  
  return { };
}

} // end namespace simulation
} // end namespace eduart
