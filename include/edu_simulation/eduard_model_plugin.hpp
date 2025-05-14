/**
 * Copyright EduArt Robotik GmbH 2025
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_simulation/eduard_gazebo_bot.hpp"

#include <gz/sim/System.hh>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

namespace eduart {
namespace simulation {

class EduardModelPlugin : public gz::sim::System
                        , public gz::sim::ISystemConfigure
                        , public gz::sim::ISystemPreUpdate
                        , public gz::sim::ISystemUpdate
                        , public gz::sim::ISystemPostUpdate
{
public:
  EduardModelPlugin();
  ~EduardModelPlugin() override;

  void Configure(
    const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, 
    gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& event_manager) override;

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;
  void Update(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;
  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  std::shared_ptr<EduardGazeboBot> _robot;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> _ros_executer;
  std::thread _run_executer;
  std::atomic_bool _is_running{false};
};

} // end namespace simulation
} // end namespace eduart
