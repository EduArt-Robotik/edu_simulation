#include "edu_simulation/gazebo_hardware_adapter.hpp"

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
  // \todo a implementation would be nice here
}

void GazeboHardwareAdapter::disable()
{
  // \todo a implementation would be nice here
}
  
robot::RobotStatusReport GazeboHardwareAdapter::getStatusReport()
{
  return { };
}

robot::diagnostic::Diagnostic GazeboHardwareAdapter::processDiagnosticsImpl()
{
  return { };
}

} // end namespace simulation
} // end namespace eduart
