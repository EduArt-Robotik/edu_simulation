#include "edu_simulation/gazebo_lighting.hpp"

namespace eduart {
namespace simulation {

GazeboLighting::GazeboLighting()
{

}

GazeboLighting::~GazeboLighting()
{

}

void GazeboLighting::processSetValue(const robot::Color &color, const robot::Lighting::Mode &mode)
{
  (void)color;
  (void)mode;
}

void GazeboLighting::initialize(const robot::Lighting::Parameter &parameter)
{
  (void)parameter;
}

} // end namespace simulation
} // end namespace eduart
