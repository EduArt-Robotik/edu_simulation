#include "edu_simulation/eduard_gazebo_bot.hpp"
#include "edu_simulation/gazebo_hardware_adapter.hpp"
#include "edu_simulation/eduard_hardware_component_factory.hpp"

namespace eduart {
namespace simulation {

EduardGazeboBot::EduardGazeboBot(sdf::ElementPtr sdf)
  : robot::eduard::Eduard("eduard_gazebo_bot", std::make_unique<GazeboHardwareAdapter>(sdf))
{
  // EduardHardwareComponentFactory factory(sdf);

  // initialize(factory);
}

EduardGazeboBot::~EduardGazeboBot()
{

}

Eigen::MatrixXf EduardGazeboBot::getKinematicMatrix(const eduart::robot::Mode mode) const
{
  return { };
}

} // end namespace simulation
} // end namespace eduart
