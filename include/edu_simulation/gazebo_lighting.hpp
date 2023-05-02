/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_component_interface.hpp>
#include <edu_robot/lighting.hpp>

namespace eduart {
namespace simulation {

class GazeboLighting : public robot::Lighting::ComponentInterface
{
public:
  GazeboLighting();
  ~GazeboLighting() override;

  void processSetValue(const robot::Color& color, const robot::Lighting::Mode& mode) override;
  void initialize(const robot::Lighting::Parameter& parameter) override;  
};

} // end namespace simulation
} // end namespace eduart
