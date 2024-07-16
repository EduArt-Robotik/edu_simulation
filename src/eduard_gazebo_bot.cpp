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
  auto hardware_adapter = std::dynamic_pointer_cast<GazeboHardwareAdapter>(_hardware_interface);
  const auto model_name = parent->GetName();
  _is_mecanum = model_name.find("mecanum") != std::string::npos;
  EduardHardwareComponentFactory factory(hardware_adapter, parent, sdf, *this);

  initialize(factory);
  _mode_state_machine.switchToMode(eduart::robot::RobotMode::INACTIVE);
}

EduardGazeboBot::~EduardGazeboBot()
{

}

void EduardGazeboBot::OnUpdate()
{
  if (_is_mecanum == false) {
    // do nothing, velocity is applied on motors
    return;
  }

  Eigen::VectorXf radps_measured(_motor_controllers.size());

  for (std::size_t i = 0; i < _motor_controllers.size(); ++i) {
    radps_measured(i) = _motor_controllers[i]->getMeasuredRpm()[0].radps(); // \todo make index access safety.
  }

  const Eigen::Vector3f velocity_measured = _inverse_kinematic_matrix * radps_measured;
  const Eigen::Rotation2Df rot(_parent->WorldPose().Yaw());
  const Eigen::Vector2f linear_velocity = rot * Eigen::Vector2f(velocity_measured.x(), velocity_measured.y());

  _parent->SetLinearVel(ignition::math::Vector3d(linear_velocity.x(), linear_velocity.y(), 0.0));
  _parent->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, velocity_measured.z() * M_PI * 2.0)); // HACK!
}

} // end namespace simulation
} // end namespace eduart
