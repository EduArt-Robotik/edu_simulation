/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>
#include <edu_robot/algorithm/low_pass_filter.hpp>

#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>

namespace eduart {
namespace simulation {

class GazeboMotorController : public robot::MotorController::HardwareInterface
{
public:
  GazeboMotorController(
    const std::string& name, gazebo::physics::ModelPtr model, gazebo::physics::JointPtr joint, const bool is_mecanum = true);
  ~GazeboMotorController() override;

  void processSetValue(const std::vector<robot::Rpm>& rpm) override;
  void initialize(const robot::Motor::Parameter& parameter) override;

private:
  void processController();

  bool _is_mecanum = true;
  robot::algorithm::LowPassFiler<float> _low_pass_filter;
  std::vector<robot::Rpm> _measured_rpm;
  gazebo::physics::JointPtr _joint;
  gazebo::physics::JointController _controller;
  gazebo::event::ConnectionPtr _update_connection;  
};

} // end namespace simulation
} // end namespace eduart
