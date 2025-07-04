#include "edu_simulation/eduard_gazebo_bot.hpp"
#include "edu_simulation/gazebo_hardware_adapter.hpp"
#include "edu_simulation/eduard_hardware_component_factory.hpp"

#include <gz/sim/Events.hh>
#include <gz/sim/Model.hh>

namespace eduart {
namespace simulation {

EduardGazeboBot::EduardGazeboBot(
  const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, 
  gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& event_manager)
  : robot::bot::EduardV3(
      "eduard_gazebo_bot", std::make_unique<GazeboHardwareAdapter>(sdf), gz::sim::Model(entity).Name(ecm)
    )
{
  auto hardware_adapter = std::dynamic_pointer_cast<GazeboHardwareAdapter>(_hardware_interface);

  // get parameter
  declare_parameter<std::string>("kinematic", "mecanum");
  _mecanum_kinematic = get_parameter("kinematic").as_string() == "mecanum";

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
  Eigen::MatrixXf kinematic_matrix;

  // if (kinematic == robot::DriveKinematic::MECANUM_DRIVE) {
    const float l_x = _parameter.mecanum.length.x;
    const float l_y = _parameter.mecanum.length.y;
    const float wheel_radius = _parameter.mecanum.wheel_diameter * 0.5f;

    kinematic_matrix.resize(4, 3);
    kinematic_matrix <<  1.0f, -1.0f, (l_x + l_y) * 0.5f * 0.9f, // reduce rotation by 10%, because model requires it.
                         1.0f,  1.0f, (l_x + l_y) * 0.5f * 0.9f, // reduce rotation by 10%, because model requires it.
                        -1.0f, -1.0f, (l_x + l_y) * 0.5f * 0.9f, // reduce rotation by 10%, because model requires it.
                        -1.0f,  1.0f, (l_x + l_y) * 0.5f * 0.9f; // reduce rotation by 10%, because model requires it.
    kinematic_matrix *= 1.0f / wheel_radius;    
  // }
  // else {
  //   throw std::invalid_argument("Eduard: given kinematic is not supported.");
  // }

  return kinematic_matrix;
}


} // end namespace simulation
} // end namespace eduart
