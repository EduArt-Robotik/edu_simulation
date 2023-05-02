#include "edu_simulation/eduard_model_plugin.hpp"
#include "edu_simulation/eduard_hardware_component_factory.hpp"
#include "edu_simulation/eduard_gazebo_bot.hpp"

#include <gazebo/physics/Model.hh>

namespace eduart {
namespace simulation {

EduardModelPlugin::EduardModelPlugin()
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;

  if (rclcpp::ok() == false) {
    rclcpp::init(0, 0);
  }
  _ros_executer = std::make_shared<gazebo_ros::Executor>();
}

void EduardModelPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  auto model_element = sdf->GetParent();
  std::string ns;

  if (sdf->HasElement("robot_namespace")) {
    ns = sdf->GetElement("robot_namespace")->GetValue()->GetAsString();
  }

  auto bot = std::make_shared<EduardGazeboBot>(parent, model_element, ns);
  _robot_ros_node = bot;
  _ros_executer->add_node(bot);
  _model = parent;

  std::cout << "sdf pointer = " << sdf << std::endl;
  std::cout << "element name: " << sdf->GetName() << std::endl;
  std::cout << "parent : " << sdf->GetParent()->GetName() << std::endl;

  _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&EduardModelPlugin::OnUpdate, this)
  );
  _update_bot_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&EduardGazeboBot::OnUpdate, bot)
  );
}

void EduardModelPlugin::OnUpdate()
{

}

} // end namespace simulation
} // end namespace eduart

GZ_REGISTER_MODEL_PLUGIN(eduart::simulation::EduardModelPlugin)