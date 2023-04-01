#include "edu_simulation/eduard_model_plugin.hpp"
#include "edu_simulation/eduard_hardware_component_factory.hpp"
#include "edu_simulation/eduard_gazebo_bot.hpp"

#include <gazebo/physics/Model.hh>

namespace eduart {
namespace simulation {

EduardModelPlugin::EduardModelPlugin()
{
  _ros_executer = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
}

void EduardModelPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr sdf)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&EduardModelPlugin::OnUpdate, this)
  );

  auto model_element = sdf->GetParent();
  EduardGazeboBot bot(model_element);
  std::cout << "sdf pointer = " << sdf << std::endl;
  std::cout << "element name: " << sdf->GetName() << std::endl;
  std::cout << "parent : " << sdf->GetParent()->GetName() << std::endl;
}

void EduardModelPlugin::OnUpdate()
{

}

} // end namespace simulation
} // end namespace eduart

GZ_REGISTER_MODEL_PLUGIN(eduart::simulation::EduardModelPlugin)