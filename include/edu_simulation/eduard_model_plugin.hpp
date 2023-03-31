/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <gazebo/gazebo.hh>
#include <rclcpp/rclcpp.hpp>

// #include <

namespace eduart {
namespace simulation {

class EduardModelPlugin : public gazebo::ModelPlugin
{
public:
  EduardModelPlugin();

  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

private:
  void OnUpdate();

  gazebo::physics::ModelPtr _model;
  gazebo::event::ConnectionPtr _update_connection;
  std::shared_ptr<rclcpp::Executor> _ros_executer;
};

} // end namespace simulation
} // end namespace eduart
