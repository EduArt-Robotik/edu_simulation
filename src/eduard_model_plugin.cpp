#include "edu_simulation/eduard_model_plugin.hpp"

namespace eduart {
namespace simulation {

EduardModelPlugin::EduardModelPlugin()
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  // if (rclcpp::ok() == false) {
  //   rclcpp::init(0, 0);
  // }
  // _ros_executer = std::make_shared<gazebo_ros::Executor>();
}

EduardModelPlugin::~EduardModelPlugin()
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
}

void EduardModelPlugin::Configure(
  const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, 
  gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventMgr)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
}

} // end namespace simulation
} // end namespace eduart

#include <gz/plugin/Register.hh>
 
// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
  eduart::simulation::EduardModelPlugin,
  gz::sim::System,
  eduart::simulation::EduardModelPlugin::ISystemConfigure
)
