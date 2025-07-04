/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/bot/eduard_v3.hpp>

#include <gz/sim/System.hh>

namespace eduart {
namespace simulation {

class EduardGazeboBot : public robot::bot::EduardV3
{
public:
  EduardGazeboBot(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, 
    gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& event_manager);
  ~EduardGazeboBot() override;

private:
  virtual Eigen::MatrixXf getKinematicMatrix(const robot::DriveKinematic kinematic) const override; 
  
  bool _mecanum_kinematic = true;
};

} // end namespace simulation
} // end namespace eduart
