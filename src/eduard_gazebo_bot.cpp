#include "edu_simulation/eduard_gazebo_bot.hpp"
#include "edu_simulation/gazebo_hardware_adapter.hpp"
#include "edu_simulation/eduard_hardware_component_factory.hpp"

#include <gz/sim/Events.hh>
#include <gz/sim/Model.hh>

namespace eduart {
namespace simulation {

EduardGazeboBot::EduardGazeboBot(
  const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, 
  gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& event_manager, robot::DriveKinematic kinematic)
  : robot::bot::EduardV3(
      "eduard_gazebo_bot", std::make_unique<GazeboHardwareAdapter>(sdf), gz::sim::Model(entity).Name(ecm)
    )
{
  auto hardware_adapter = std::dynamic_pointer_cast<GazeboHardwareAdapter>(_hardware_interface);

  // get parameter
  _mecanum_kinematic = kinematic == robot::DriveKinematic::MECANUM_DRIVE;

  // initialize hardware layer
  const auto model_name = gz::sim::Model(entity).Name(ecm);
  EduardHardwareComponentFactory factory(hardware_adapter, entity, sdf, ecm, *this);

  try {
    initialize(factory);
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "error occurred during initialization. what = %s.", ex.what());
  }

  // set initial mode
  _mode_state_machine.switchToMode(eduart::robot::RobotMode::INACTIVE);
}

EduardGazeboBot::~EduardGazeboBot()
{

}

Eigen::MatrixXf EduardGazeboBot::getKinematicMatrix(const robot::DriveKinematic kinematic) const
{
  // give user feedback
  if (kinematic == robot::DriveKinematic::MECANUM_DRIVE) {
    if (_mecanum_kinematic) {
      RCLCPP_INFO(get_logger(), "already mecanum kinematics selected.");
    }
    else {
      RCLCPP_INFO(get_logger(), "can't switch to mecanum kinematics. Wrong wheels mounted for this!");
    }
  }
  else if (kinematic == robot::DriveKinematic::SKID_DRIVE) {
    if (_mecanum_kinematic == false) {
      RCLCPP_INFO(get_logger(), "already skid kinematics selected.");
    }
    else {
      RCLCPP_INFO(get_logger(), "can't switch to skid kinematics. Wrong wheels mounted for this!");
    }
  }
  else {
    RCLCPP_ERROR(get_logger(), "unsupported kinematics type.");
  }

  // return kinematic
  Eigen::MatrixXf kinematic_matrix;

  if (_mecanum_kinematic) {
    // mecanum kinematic
    const float l_x = _parameter.mecanum.length.x;
    const float l_y = _parameter.mecanum.length.y;
    const float wheel_radius = _parameter.mecanum.wheel_diameter * 0.5f;

    kinematic_matrix.resize(4, 3);
    kinematic_matrix <<  1.0f, -1.0f, (l_x + l_y) * 0.5f * 0.9f, // reduce rotation by 10%, because model requires it.
                         1.0f,  1.0f, (l_x + l_y) * 0.5f * 0.9f, // reduce rotation by 10%, because model requires it.
                        -1.0f, -1.0f, (l_x + l_y) * 0.5f * 0.9f, // reduce rotation by 10%, because model requires it.
                        -1.0f,  1.0f, (l_x + l_y) * 0.5f * 0.9f; // reduce rotation by 10%, because model requires it.
    kinematic_matrix *= 1.0f / wheel_radius;    
  }
  else {
    // skid kinematic
    const float l_x = _parameter.skid.length.x;
    const float l_y = _parameter.skid.length.y;
    const float wheel_radius = _parameter.skid.wheel_diameter * 0.5f;
    const float l_squared = l_x * l_x + l_y * l_y;

    kinematic_matrix.resize(4, 3);
    kinematic_matrix <<  1.0f, 0.0f, l_squared / (2.0f * l_y),
                         1.0f, 0.0f, l_squared / (2.0f * l_y),
                        -1.0f, 0.0f, l_squared / (2.0f * l_y),
                        -1.0f, 0.0f, l_squared / (2.0f * l_y);
    kinematic_matrix *= 1.0f / wheel_radius;    
  }

  return kinematic_matrix;
}


} // end namespace simulation
} // end namespace eduart
