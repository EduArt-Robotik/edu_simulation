/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_component_factory.hpp>

#include <gazebo/gazebo.hh>

namespace eduart {
namespace simulation {

class EduardHardwareComponentFactory : public robot::HardwareComponentFactory
{
public:
  EduardHardwareComponentFactory(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf, rclcpp::Node& ros_node);
};

} // end namespace simulation
} // end namespace eduart