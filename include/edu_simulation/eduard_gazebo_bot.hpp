/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/eduard/eduard.hpp>

#include <gazebo/gazebo.hh>

namespace eduart {
namespace simulation {

class EduardGazeboBot : public robot::eduard::Eduard
{
public:
  EduardGazeboBot(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf, const std::string& ns);
  ~EduardGazeboBot() override;

  void OnUpdate();

private:
  gazebo::physics::ModelPtr _parent;
};

} // end namespace simulation
} // end namespace eduart
