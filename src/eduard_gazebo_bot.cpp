#include "edu_simulation/eduard_gazebo_bot.hpp"
#include "edu_simulation/gazebo_hardware_adapter.hpp"
#include "edu_simulation/eduard_hardware_component_factory.hpp"

#include <gazebo/physics/Model.hh>

namespace eduart {
namespace simulation {

EduardGazeboBot::EduardGazeboBot(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf, const std::string& ns)
  : robot::eduard::Eduard(
      "eduard_gazebo_bot", std::make_unique<GazeboHardwareAdapter>(sdf), ns
    )
  , _parent(parent)
{
  EduardHardwareComponentFactory factory(sdf);

  initialize(factory);
}

EduardGazeboBot::~EduardGazeboBot()
{

}

void EduardGazeboBot::OnUpdate()
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  Eigen::VectorXf radps_measured(_motor_controllers.size());

  for (std::size_t i = 0; i < _motor_controllers.size(); ++i) {
    radps_measured(i) = _motor_controllers[i]->getMeasuredRpm().radps();
  }

  const Eigen::Vector3f velocity_measured = _inverse_kinematic_matrix * radps_measured;
  _parent->SetLinearVel(
    ignition::math::Vector3d(velocity_measured.x(), velocity_measured.y(), velocity_measured.z())
  );
}

} // end namespace simulation
} // end namespace eduart
