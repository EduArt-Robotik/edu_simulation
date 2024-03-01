#include "edu_simulation/gazebo_motor_controller.hpp"

#include <gazebo/physics/World.hh>

namespace eduart {
namespace simulation {

GazeboMotorController::GazeboMotorController(
  const std::string& name, gazebo::physics::ModelPtr model, gazebo::physics::JointPtr joint, const bool is_mecanum)
  : robot::MotorController::HardwareInterface(name, 1)
  , _is_mecanum(is_mecanum)
  , _low_pass_filter({0.2f})
  , _measured_rpm(1, 0.0)
  , _joint(joint)
  , _controller(model)
{
  if (_is_mecanum == false) {
    // _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    //   std::bind(&gazebo::physics::JointController::Update, &_controller)
    // );
    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboMotorController::processController, this)
    );
  }
}

GazeboMotorController::~GazeboMotorController()
{

}

void GazeboMotorController::processSetValue(const std::vector<robot::Rpm>& rpm)
{
  if (rpm.size() != 1) {
    throw std::invalid_argument("GazeboMotorController::processSetValue(): given rpm vector must have size 1.");
  }

  if (_is_mecanum) {
    _measured_rpm[0] = _low_pass_filter(rpm[0]);
    _callback_process_measurement(_measured_rpm, true);
  }
  else {
    _controller.SetVelocityTarget(_joint->GetScopedName(), rpm[0].radps());

    // _joint->SetVelocity(0, _low_pass_filter(rpm[0].radps()));
    _measured_rpm[0] = robot::Rpm::fromRadps(_joint->GetVelocity(0));
    _callback_process_measurement(_measured_rpm, true);
  }
}

void GazeboMotorController::initialize(const robot::Motor::Parameter &parameter)
{
  if (_is_mecanum == false) {
    gazebo::common::PID pid(
       parameter.kp,
       // only work with p controller
       0.0, // parameter.ki,
       0.0, // parameter.kd,
       0.0,
       0.0,
       robot::Rpm(parameter.max_rpm).radps(),
      -robot::Rpm(parameter.max_rpm).radps()
    );
    _controller.AddJoint(_joint);
    _controller.SetVelocityPID(_joint->GetScopedName(), pid);
    _joint->SetVelocityLimit(0, robot::Rpm(parameter.max_rpm).radps());
    // torque 0.9 Nm
    // 62 rpm
  }
  else {
    // motors should run free
    _joint->SetEffortLimit(0, 0.0);
    _joint->SetParam("fmax", 0, 0.0);
  }
}

void GazeboMotorController::processController()
{
  if ((_joint->GetWorld()->SimTime() - _controller.GetLastUpdateTime()).Double() > 0.01) { // Every 10 ms.
    _controller.Update();
  }
}

} // end namespace simulation
} // end namespace eduart
