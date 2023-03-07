#include "edu_simulation/object_detection_sensor.hpp"

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include <gazebo/sensors/LogicalCameraSensor.hh>

#include <gazebo_ros/node.hpp>

#include <memory>

namespace eduart {
namespace simulation {

ObjectDetectionSensor::ObjectDetectionSensor()
  : gazebo::SensorPlugin()
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
}

ObjectDetectionSensor::~ObjectDetectionSensor()
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;  
}

void ObjectDetectionSensor::Init()
{
  gazebo::SensorPlugin::Init();
  std::cout << __PRETTY_FUNCTION__ << std::endl;
}

void ObjectDetectionSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  // _ros_node = gazebo_ros::Node::Get(_sdf, _worldName);

  auto logical_camera = std::dynamic_pointer_cast<gazebo::sensors::LogicalCameraSensor>(_sensor);

  if (logical_camera == nullptr) {
    std::cout << "nix wars" << std::endl;
    return;
  }

  _on_update_connection = _sensor->ConnectUpdated(std::bind(&ObjectDetectionSensor::OnUpdate, this));
  _logical_camera = logical_camera;
  _logical_camera->SetActive(true);
  std::cout << "finished loading..." << std::endl;
}

void ObjectDetectionSensor::OnUpdate()
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  std::cout << "found objects = " << _logical_camera->Image().model_size() << std::endl;
}

extern "C" GZ_PLUGIN_VISIBLE gazebo::SensorPlugin * RegisterPlugin();
gazebo::SensorPlugin * RegisterPlugin()
{
  return (gazebo::SensorPlugin *)(new ObjectDetectionSensor());
}

} // end namespace simulation
} // end namespace eduart

// GZ_REGISTER_SENSOR_PLUGIN(eduart::simulation::ObjectDetectionSensor)
