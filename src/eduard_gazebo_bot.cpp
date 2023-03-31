#include "edu_simulation/eduard_gazebo_bot.hpp"

namespace eduart {
namespace simulation {

EduardGazeboBot::EduardGazeboBot()
  : robot::eduard::Eduard("eduard_gazebo_bot", nullptr)
{

}

EduardGazeboBot::~EduardGazeboBot()
{

}

} // end namespace simulation
} // end namespace eduart
