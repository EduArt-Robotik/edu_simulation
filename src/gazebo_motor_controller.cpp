#include "edu_simulation/gazebo_motor_controller.hpp"

#include <gz/msgs.hh>

namespace eduart {
namespace simulation {

GazeboMotorController::GazeboMotorController(
  const std::string& name, const std::string& gz_velocity_topic_name, const std::string& gz_feedback_topic_name)
  : robot::MotorController::HardwareInterface(name, 1)
  , _gz_node(std::make_shared<gz::transport::Node>())
  , _low_pass_filter({0.2f})
  , _measured_rpm(1, 0.0)
{
  _gz_pub_velocity = _gz_node->Advertise<gz::msgs::Double>(gz_velocity_topic_name);
  _gz_node->Subscribe(gz_feedback_topic_name, &GazeboMotorController::processFeedback, this);
}

GazeboMotorController::~GazeboMotorController()
{

}

void GazeboMotorController::processSetValue(const std::vector<robot::Rpm>& rpm)
{
  if (rpm.size() != 1) {
    throw std::invalid_argument("GazeboMotorController::processSetValue(): given rpm vector must have size 1.");
  }

  // if motor controller is not enabled set set point to zero
  const robot::Rpm set_point = _is_enabled ? rpm[0] : robot::Rpm(0.0);
  gz::msgs::Double velocity_msgs;

  velocity_msgs.set_data(_low_pass_filter(set_point.radps()));
  _gz_pub_velocity.Publish(velocity_msgs);
}

void GazeboMotorController::initialize(const robot::Motor::Parameter &parameter)
{
  _low_pass_filter.clear();
}

void GazeboMotorController::processFeedback(const gz::msgs::Double& velocity)
{
  _measured_rpm[0] = robot::Rpm::fromRadps(velocity.data());
  _callback_process_measurement(_measured_rpm, _is_enabled);
}

} // end namespace simulation
} // end namespace eduart
