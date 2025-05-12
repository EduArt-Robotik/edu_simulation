/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>
#include <edu_robot/algorithm/low_pass_filter.hpp>

#include <gz/transport.hh>

namespace eduart {
namespace simulation {

class GazeboMotorController : public robot::MotorController::HardwareInterface
{
public:
  GazeboMotorController(const std::string& name, const std::string& gz_velocity_topic_name);
  ~GazeboMotorController() override;

  void processSetValue(const std::vector<robot::Rpm>& rpm) override;
  void initialize(const robot::Motor::Parameter& parameter) override;
  inline void enable() {
    _is_enabled = true;
  }
  inline void disable() {
    _is_enabled = false;
  }

private:
  void processController();

  std::shared_ptr<gz::transport::Node> _gz_node;
  gz::transport::Node::Publisher _gz_pub_velocity;

  bool _is_enabled = false;
  robot::algorithm::LowPassFiler<float> _low_pass_filter;
  std::vector<robot::Rpm> _measured_rpm;
};

} // end namespace simulation
} // end namespace eduart
