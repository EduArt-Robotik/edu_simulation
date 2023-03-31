#include "edu_simulation/eduard_model_plugin.hpp"

namespace eduart {
namespace simulation {

EduardModelPlugin::EduardModelPlugin()
{
  _ros_executer = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
}

void EduardModelPlugin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&EduardModelPlugin::OnUpdate, this)
  );
}

void EduardModelPlugin::OnUpdate()
{

}

} // end namespace simulation
} // end namespace eduart

GZ_REGISTER_MODEL_PLUGIN(eduart::simulation::EduardModelPlugin)