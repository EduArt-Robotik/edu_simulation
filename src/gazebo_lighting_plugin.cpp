#include "edu_simulation/gazebo_lighting_plugin.hpp"

#include <gz/sim/components/Light.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Material.hh>

namespace eduart {
namespace simulation {

GazeboLightingPlugin::GazeboLightingPlugin()
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;

  _color = {255, 0, 0};
}

GazeboLightingPlugin::~GazeboLightingPlugin()
{

}

void GazeboLightingPlugin::Configure(
  const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, 
  gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& event_manager)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;

  // gather entities for light and visual
  if (sdf->HasElement("lighting_name") == false) {
    gzerr << "no lighting name present in plugin section --> cancel configuring" << std::endl;
    return;
  }
  if (sdf->HasElement("visual_name") == false) {
    gzerr << "no visual name present in plugin section --> cancel configuring" << std::endl;
    return;
  }

  const std::string lighting_name = sdf->FindElement("lighting_name")->Get<std::string>();
  const std::string visual_name = sdf->FindElement("visual_name")->Get<std::string>();

  if (auto lighting_entity = ecm.EntityByName(lighting_name)) {
    _lighting_entity = *lighting_entity;
  }
  else {
    gzerr << "lighting entity with name [" << lighting_name << "] not found --> cancel configuring" << std::endl;
    return;
  }
  if (auto visual_entity = ecm.EntityByName(visual_name)) {
    if (!ecm.EntityHasComponentType(*visual_entity, gz::sim::components::Material().TypeId())) {
      ecm.CreateComponent(*visual_entity, gz::sim::components::Material());
    }
    _visual_entity = *visual_entity;
  }
  else {
    gzerr << "visual entity with name [" << visual_name << "] not found --> cancel configuring" << std::endl;
    return;
  }
}

void GazeboLightingPlugin::Update(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm)
{
  if (_lighting_entity == gz::sim::kNullEntity || _visual_entity == gz::sim::kNullEntity) {
    // not ready --> return
    return;
  }

  auto light = ecm.Component<gz::sim::components::Light>(_lighting_entity);
  auto material = ecm.Component<gz::sim::components::Material>(_visual_entity);

  if (light == nullptr) {
    gzerr << "light component not found in entity [" << _lighting_entity << "] --> cancel updating" << std::endl;
    return;
  }
  if (material == nullptr) {
    gzerr << "material component not found in entity [" << _visual_entity << "] --> cancel updating" << std::endl;
    return;
  }

  light->Data().SetIntensity(1.0);
  material->Data().SetEmissive(gz::math::Color(_color.r / 255.0, _color.g / 255.0, _color.b / 255.0));

}

} // end namespace simulation
} // end namespace eduart

#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(
  eduart::simulation::GazeboLightingPlugin,
  gz::sim::System,
  eduart::simulation::GazeboLightingPlugin::ISystemConfigure,
  eduart::simulation::GazeboLightingPlugin::ISystemUpdate
)