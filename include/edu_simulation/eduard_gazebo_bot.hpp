/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <eduard/eduard.hpp>

namespace eduart {
namespace simulation {

class EduardGazeboBot : public robot::eduard::Eduard
{
public:
  EduardGazeboBot();
  ~EduardGazeboBot() override;
};

} // end namespace simulation
} // end namespace eduart
