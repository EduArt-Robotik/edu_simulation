#include "edu_simulation/eduard_model_plugin.hpp"

namespace eduart {
namespace simulation {

using namespace std::chrono_literals;

EduardModelPlugin::EduardModelPlugin()
{

}

EduardModelPlugin::~EduardModelPlugin()
{
  RCLCPP_INFO(rclcpp::get_logger("EduardModelPlugin"), "shuting down...");
  _is_running = false;
  _run_executer.join();
}

void EduardModelPlugin::Configure(
  const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, 
  gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& event_manager)
{
  robot::DriveKinematic kinematic = robot::DriveKinematic::MECANUM_DRIVE;

  if (rclcpp::ok() == false) {
    // std::string kinematic_string = "kinematic:=";

    // if (const auto element = sdf->FindElement("kinematic"); element != nullptr) {
    //   const auto kinematic = element->GetAttribute("value")->GetAsString();
    //   kinematic_string += kinematic;
    // }
    // else {
    //   // default
    //   kinematic_string += "mecanum";
    // }

    constexpr const char* argv[] = {
      "eduard_model_plugin",
      "--ros-args",
      "-p",
      "use_sim_time:=True",
      "-p",
      "imu.publish_tf:=False"
      // "-p",
      // kinematic_string.data()
    };
    constexpr int argc = sizeof(argv) / sizeof(char*);

    rclcpp::init(
      argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None
    );
  }

  if (const auto element = sdf->FindElement("kinematic"); element != nullptr) {
    if (element->GetAttribute("value")->GetAsString() == "mecanum") {
      kinematic = robot::DriveKinematic::MECANUM_DRIVE;
    }
    else {
      kinematic = robot::DriveKinematic::SKID_DRIVE;
    }
  }

  _ros_executer = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  _robot = std::make_shared<EduardGazeboBot>(entity, sdf, ecm, event_manager, kinematic);
  _ros_executer->add_node(_robot);
  _is_running = true;

  // spin node in separated thread
  _run_executer = std::thread([this](){
    while (_is_running) {
      _ros_executer->spin_once(100ms);
    }
  });
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
