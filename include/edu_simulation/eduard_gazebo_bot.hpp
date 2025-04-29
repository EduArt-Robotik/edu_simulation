/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/bot/eduard_v2.hpp>

#include <gazebo/gazebo.hh>

namespace eduart {
namespace simulation {

class EduardGazeboBot : public robot::bot::EduardV2
{
public:
  EduardGazeboBot(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf, const std::string& ns);
  ~EduardGazeboBot() override;

  void OnUpdate(const gazebo::common::UpdateInfo & info);

private:
  gazebo::physics::ModelPtr _parent;
  bool _is_mecanum = true;
  gazebo::common::Time _stamp_last_update;
  ignition::math::Pose3d _current_pose;
};

} // end namespace simulation
} // end namespace eduart
