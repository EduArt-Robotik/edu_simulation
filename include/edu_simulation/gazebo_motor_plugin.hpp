/**
 * Copyright EduArt Robotik GmbH 2025
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <gz/transport.hh>
#include <gz/sim/System.hh>
#include <gz/msgs.hh>

#include <edu_robot/algorithm/pid_controller.hpp>

namespace eduart {
namespace simulation {

class GazeboMotorPlugin : public gz::sim::System
                        , public gz::sim::ISystemConfigure
                        , public gz::sim::ISystemPreUpdate
                        , public gz::sim::ISystemPostUpdate
{
public:
  GazeboMotorPlugin();
  ~GazeboMotorPlugin() override;

  void Configure(
    const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, 
    gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& event_manager) override;
  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;
  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  void callbackReceiveJointVelocity(const gz::msgs::Double& velocity);

  // Communication
  std::shared_ptr<gz::transport::Node> _node;
  gz::transport::Node::Publisher _pub_velocity;

  // Physic Access
  gz::sim::Entity _joint_entity = 0;
  std::chrono::steady_clock::time_point _stamp_last_sent_velocity;

  // Control
  std::shared_ptr<robot::algorithm::Pid> _pid;
  std::vector<double> _cmd_velocity = { 0.0 };
  std::vector<double> _effort = { 0.0 };
};

} // end namespace simulation
} // end namespace eduart
