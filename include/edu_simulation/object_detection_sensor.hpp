/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <gazebo/sensors/LogicalCameraSensor.hh>

namespace eduart {
namespace simulation {

class ObjectDetectionSensor : public gazebo::sensors::LogicalCameraSensor
{
public:
  ObjectDetectionSensor();
  
  void Init() override;
  void Load(const std::string &_worldName, sdf::ElementPtr _sdf) override;
  void Load(const std::string &_worldName) override;
  bool UpdateImpl(const bool _force) override;
};

} // end namespace simulation
} // end namespace eduart
