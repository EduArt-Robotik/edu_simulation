/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>

namespace eduart {
namespace simulation {

class GazeboMotorController : public robot::MotorController::HardwareInterface
{
public:
  GazeboMotorController(const std::string& name);
  ~GazeboMotorController() override;

  void processSetValue(const std::vector<robot::Rpm>& rpm) override;
  void initialize(const robot::Motor::Parameter& parameter) override;
};

} // end namespace simulation
} // end namespace eduart
