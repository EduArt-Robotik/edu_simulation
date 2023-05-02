/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>

namespace eduart {
namespace simulation {

class GazeboMotorController : public robot::MotorController::ComponentInterface
                            , public robot::MotorController::SensorInterface
{
public:
  GazeboMotorController();
  ~GazeboMotorController() override;

  void processSetValue(const robot::Rpm& rpm) override;
  void initialize(const robot::MotorController::Parameter& parameter) override;
};

} // end namespace simulation
} // end namespace eduart
