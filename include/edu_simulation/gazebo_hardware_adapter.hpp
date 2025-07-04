/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_robot_interface.hpp>

#include <sdf/Element.hh>

namespace eduart {
namespace simulation {

class GazeboMotorController;

class GazeboHardwareAdapter : public robot::HardwareRobotInterface
{
public:
  GazeboHardwareAdapter(const sdf::ElementConstPtr sdf);
  ~GazeboHardwareAdapter() override;

  void enable() override;
  void disable() override;
  robot::RobotStatusReport getStatusReport() override;

  inline void registerMotorController(std::shared_ptr<GazeboMotorController> motor_controller) {
    _motor_controller.push_back(motor_controller);
  }

private:
  robot::diagnostic::Diagnostic processDiagnosticsImpl() override;

  std::vector<std::shared_ptr<GazeboMotorController>> _motor_controller;
};

} // end namespace simulation
} // end namespace eduart
