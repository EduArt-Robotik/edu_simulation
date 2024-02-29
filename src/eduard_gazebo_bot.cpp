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
  EduardHardwareComponentFactory factory(parent, sdf, *this);

  initialize(factory);
}

EduardGazeboBot::~EduardGazeboBot()
{

}

void EduardGazeboBot::OnUpdate()
{
  Eigen::VectorXf radps_measured(_motor_controllers.size());

  for (std::size_t i = 0; i < _motor_controllers.size(); ++i) {
    radps_measured(i) = _motor_controllers[i]->getMeasuredRpm()[0].radps(); // \todo make index access safety.
  }

  const Eigen::Vector3f velocity_measured = _inverse_kinematic_matrix * radps_measured;
  const Eigen::Rotation2Df rot(_parent->WorldPose().Yaw());
  const Eigen::Vector2f linear_velocity = rot * Eigen::Vector2f(velocity_measured.x(), velocity_measured.y());

  _parent->SetLinearVel(ignition::math::Vector3d(linear_velocity.x(), linear_velocity.y(), 0.0));
  _parent->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, velocity_measured.z()));
}

} // end namespace simulation
} // end namespace eduart
