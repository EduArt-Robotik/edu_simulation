#include "edu_simulation/eduard_gazebo_bot.hpp"
#include "edu_simulation/gazebo_hardware_adapter.hpp"
#include "edu_simulation/eduard_hardware_component_factory.hpp"

#include <bot/eduard_v2.hpp>
#include <gazebo/physics/Model.hh>

namespace eduart {
namespace simulation {

EduardGazeboBot::EduardGazeboBot(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf, const std::string& ns)
  : robot::bot::EduardV2(
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

void EduardGazeboBot::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  if (_is_mecanum == false) {
    // do nothing, velocity is applied on motors
    return;
  }

  const double dt = (info.simTime - _stamp_last_update).Double();
  Eigen::VectorXf radps_measured(_motor_controllers.size());

  for (std::size_t i = 0; i < _motor_controllers.size(); ++i) {
    radps_measured(i) = _motor_controllers[i]->getMeasuredRpm()[0].radps(); // \todo make index access safety.
  }

  // calculate current velocity vector from wheel rotations
  const Eigen::Vector3f velocity_measured = _inverse_kinematic_matrix * radps_measured;

  // transfrom velocity vector into world coordinate system
  const Eigen::Rotation2Df rot(_parent->WorldPose().Yaw());
  const Eigen::Vector2f linear_velocity = rot * Eigen::Vector2f(velocity_measured.x(), velocity_measured.y());

  // apply velocity to robot model (world coordinate system. note: no idea why its in world coordinate)
  _current_pose.SetX((linear_velocity * dt).x() + _current_pose.X());
  _current_pose.SetY((linear_velocity * dt).y() + _current_pose.Y());

  const double yaw = _current_pose.Yaw() + dt * velocity_measured.z();
  _current_pose.Rot().Euler(0.0, 0.0, yaw);

  _parent->SetWorldPose(_current_pose);

  _stamp_last_update = info.simTime;
}

} // end namespace simulation
} // end namespace eduart
