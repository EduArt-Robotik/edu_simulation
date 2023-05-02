/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/robot_hardware_interface.hpp>

#include <gazebo/gazebo.hh>

namespace eduart {
namespace simulation {

class GazeboHardwareAdapter : public robot::RobotHardwareInterface
{
public:
  GazeboHardwareAdapter(sdf::ElementPtr sdf);
  ~GazeboHardwareAdapter() override;

  void enable() override;
  void disable() override;
  robot::RobotStatusReport getStatusReport() override;
};

} // end namespace simulation
} // end namespace eduart