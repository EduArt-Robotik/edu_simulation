#include "collision_avoidance_lidar_node.hpp"

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <functional>

namespace eduart {
namespace fleet {

static bool is_in_box(
  const sensor_msgs::PointCloud2ConstIterator<float>& point, const Eigen::Vector2f& left_bottom, const Eigen::Vector2f& right_top)
{
  return point[0] >= left_bottom.x() && point[0] <= right_top.x() && point[1] >= right_top.y() && point[1] <= left_bottom.y();
}

CollisionAvoidanceLidar::Parameter CollisionAvoidanceLidar::get_parameter(
  const Parameter &default_parameter, rclcpp::Node &ros_node)
{
  Parameter parameter;

  ros_node.declare_parameter<double>("distance.reduce_velocity", default_parameter.distance_reduce_velocity);
  ros_node.declare_parameter<double>("distance.velocity_zero", default_parameter.distance_velocity_zero);
  ros_node.declare_parameter<double>("size.width", default_parameter.size.width);
  ros_node.declare_parameter<double>("size.length", default_parameter.size.length);
  ros_node.declare_parameter<std::string>("tf_frame_robot", default_parameter.tf_base_link);

  parameter.distance_reduce_velocity = ros_node.get_parameter("distance.reduce_velocity").as_double();
  parameter.distance_velocity_zero = ros_node.get_parameter("distance.velocity_zero").as_double();
  parameter.size.width = ros_node.get_parameter("size.width").as_double();
  parameter.size.length = ros_node.get_parameter("size.length").as_double();
  parameter.tf_base_link = ros_node.get_parameter("tf_frame_robot").as_string();

  return parameter;
}

CollisionAvoidanceLidar::CollisionAvoidanceLidar()
  : rclcpp::Node("collision_avoidance")
  , _parameter(get_parameter({}, *this))
  , _tf_buffer(std::make_unique<tf2_ros::Buffer>(get_clock()))
  , _tf_listener(std::make_shared<tf2_ros::TransformListener>(*_tf_buffer))
  , _laser_projection(std::make_shared<laser_geometry::LaserProjection>())
{
  // ROS Related
  _sub_laser_scan = create_subscription<sensor_msgs::msg::LaserScan>(
    "in/scan",
    rclcpp::QoS(10).best_effort(),
    std::bind(&CollisionAvoidanceLidar::callbackLaserScan, this, std::placeholders::_1)
  );
  _sub_point_cloud = create_subscription<sensor_msgs::msg::PointCloud2>(
    "in/point_cloud", 
    rclcpp::QoS(10).best_effort(), 
    std::bind(&CollisionAvoidanceLidar::callbackPointCloud, this, std::placeholders::_1)
  );
  _sub_velocity = create_subscription<geometry_msgs::msg::Twist>(
    "in/cmd_vel",
    rclcpp::QoS(10).best_effort(),
    std::bind(&CollisionAvoidanceLidar::callbackVelocity, this, std::placeholders::_1)
  );
  _pub_velocity = create_publisher<geometry_msgs::msg::Twist>(
    "out/cmd_vel",
    rclcpp::QoS(10).reliable()
  );
  _pub_debug = create_publisher<sensor_msgs::msg::PointCloud2>(
    "out/debug", 
    rclcpp::QoS(2).reliable()
  );

  // Preparing Processing
  _processing_data.intersection.fill(false);
  _processing_data.reduce_factor.fill(1.0f);

  // Box in front.
  _processing_data.left_bottom[FRONT].x() = _parameter.size.length / 2.0f;
  _processing_data.left_bottom[FRONT].y() = _parameter.size.width / 2.0f;
  _processing_data.right_top[FRONT].x() = _parameter.size.length / 2.0f + _parameter.distance_reduce_velocity;
  _processing_data.right_top[FRONT].y() = -_parameter.size.width / 2.0f;

  // Box in back.
  _processing_data.left_bottom[REAR].x() = -_parameter.size.length / 2.0f - _parameter.distance_reduce_velocity;
  _processing_data.left_bottom[REAR].y() = _parameter.size.width / 2.0f;
  _processing_data.right_top[REAR].x() = -_parameter.size.length / 2.0f;
  _processing_data.right_top[REAR].y() = -_parameter.size.width / 2.0f;

  // Box on left side.
  _processing_data.left_bottom[LEFT].x() = -_parameter.size.length / 2.0f;
  _processing_data.left_bottom[LEFT].y() = _parameter.size.width / 2.0f + _parameter.distance_reduce_velocity;
  _processing_data.right_top[LEFT].x() = _parameter.size.length / 2.0f;
  _processing_data.right_top[LEFT].y() = _parameter.size.width / 2.0f;

  // Box on right side.
  _processing_data.left_bottom[RIGHT].x() = -_parameter.size.length / 2.0f;
  _processing_data.left_bottom[RIGHT].y() = -_parameter.size.width / 2.0f;
  _processing_data.right_top[RIGHT].x() = _parameter.size.length / 2.0f;
  _processing_data.right_top[RIGHT].y() = -_parameter.size.width / 2.0f - _parameter.distance_reduce_velocity;
}

void CollisionAvoidanceLidar::callbackLaserScan(std::shared_ptr<const sensor_msgs::msg::LaserScan> msg)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  sensor_msgs::msg::PointCloud2 points;

  // Transform laser scan into robot frame.
  try {
    _laser_projection->transformLaserScanToPointCloud(
      getFrameIdPrefix() + _parameter.tf_base_link, *msg, points, *_tf_buffer
    );
  }
  catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(
      get_logger(),
      "Could not transform from frame %s to frame %s. what = %s.",
      msg->header.frame_id.c_str(),
      (getFrameIdPrefix() + _parameter.tf_base_link).c_str(),
      ex.what()
    );
    return;
  }

  _pub_debug->publish(points);
  callbackPointCloud(points);
}

void CollisionAvoidanceLidar::callbackPointCloud(const sensor_msgs::msg::PointCloud2& points)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  // Check if an intersection is present.
  // for (const auto& field : points.fields) {
  //   std::cout << "field: "<< field.name << std::endl;
  //   std::cout << "datatype: " << field.datatype << std::endl;
  // }

  // Prepare processing data and estimate new intersections.
  _processing_data.intersection.fill(false);
  _processing_data.reduce_factor.fill(1.0f);

  for (sensor_msgs::PointCloud2ConstIterator<float> point(points, "x"); point != point.end(); ++point) {
    if (is_in_box(point, _processing_data.left_bottom[FRONT], _processing_data.right_top[FRONT])) {
      // Point is in front and is too close.
      _processing_data.intersection[FRONT] = true;
      _processing_data.reduce_factor[FRONT] = std::min(
        calculateReduceFactor(point[0] - _parameter.size.length / 2.0f, _parameter),
        _processing_data.reduce_factor[FRONT]
      );
    }

    // if (point[0] < 0.0 && std::abs(point[0]) <= _parameter.distance_reduce_velocity) {
    if (is_in_box(point, _processing_data.left_bottom[REAR], _processing_data.right_top[REAR])) {
      // Point is in back and is too close.
      _processing_data.intersection[REAR] = true;
      _processing_data.reduce_factor[REAR] = std::min(
        calculateReduceFactor(std::abs(point[0] + _parameter.size.length / 2.0f), _parameter),
        _processing_data.reduce_factor[REAR]
      );
    }

    if (is_in_box(point, _processing_data.left_bottom[LEFT], _processing_data.right_top[LEFT])) {
      // Point is left and is too close.
      _processing_data.intersection[LEFT] = true;
      _processing_data.reduce_factor[LEFT] = std::min(
        calculateReduceFactor(point[1] - _parameter.size.width / 2.0f, _parameter),
        _processing_data.reduce_factor[LEFT]
      );      
    }

    if (is_in_box(point, _processing_data.left_bottom[RIGHT], _processing_data.right_top[RIGHT])) {
      // std::cout << "inside left box: (" << point[0] << ", " << point[1] << ")" << std::endl;
      // Point is right and is too close.
      _processing_data.intersection[RIGHT] = true;
      _processing_data.reduce_factor[RIGHT] = std::min(
        calculateReduceFactor(std::abs(point[1] + _parameter.size.width / 2.0f), _parameter),
        _processing_data.reduce_factor[RIGHT]
      );
    }
  }

  // std::cout << "reduce factors:\n";
  // std::cout << "front = " << _processing_data.reduce_factor[FRONT] << std::endl;
  // std::cout << "rear  = " << _processing_data.reduce_factor[REAR] << std::endl;
  // std::cout << "left  = " << _processing_data.reduce_factor[LEFT] << std::endl;
  // std::cout << "right = " << _processing_data.reduce_factor[RIGHT] << std::endl;
}

void CollisionAvoidanceLidar::callbackVelocity(std::shared_ptr<const geometry_msgs::msg::Twist> msg)
{
  geometry_msgs::msg::Twist msg_out = *msg;
  float reduce_factor_angular = 1.0;

  // Reduce velocity regarding the intersection area.
  // Front
  if (_processing_data.intersection[Area::FRONT]) {
    if (msg->linear.x >= 0.0) {
      // Robot will drive toward intersection... avoid driving into obstacle.
      msg_out.linear.x *= _processing_data.reduce_factor[Area::FRONT];
    }
    if (std::abs(msg->angular.z) >= 0.0) {
      // Robot could rotate into obstacle...
      reduce_factor_angular = std::min(_processing_data.reduce_factor[Area::FRONT], reduce_factor_angular);
    }
  }
  // Rear
  if (_processing_data.intersection[Area::REAR]) {
    if (msg->linear.x < 0.0) {
      // Robot will drive toward intersection... avoid driving into obstacle.
      msg_out.linear.x *= _processing_data.reduce_factor[Area::REAR];      
    }
    if (std::abs(msg->angular.z) >= 0.0) {
      // Robot could rotate into obstacle...
      reduce_factor_angular = std::min(_processing_data.reduce_factor[Area::REAR], reduce_factor_angular);
    }    
  }
  // Left Side
  if (_processing_data.intersection[Area::LEFT]) {
    if (msg->linear.y >= 0.0) {
      // Robot will drive toward intersection... avoid driving into obstacle.
      msg_out.linear.y *= _processing_data.reduce_factor[Area::LEFT];
    }
    if (std::abs(msg->angular.z) >= 0.0) {
      // Robot could rotate into obstacle...
      reduce_factor_angular = std::min(_processing_data.reduce_factor[Area::LEFT], reduce_factor_angular);
    }
  }
  // Right Side
  if (_processing_data.intersection[Area::RIGHT]) {
    if (msg->linear.y < 0.0) {
      // Robot will drive toward intersection... avoid driving into obstacle.
      msg_out.linear.y *= _processing_data.reduce_factor[Area::RIGHT];
    }
    if (std::abs(msg->angular.z) >= 0.0) {
      // Robot could rotate into obstacle...
      reduce_factor_angular = std::min(_processing_data.reduce_factor[Area::RIGHT], reduce_factor_angular);
    }
  }

  // Apply angular velocity reduction
  msg_out.angular.z *= reduce_factor_angular;

  _pub_velocity->publish(msg_out);
}

float CollisionAvoidanceLidar::calculateReduceFactor(const float distance, const Parameter& parameter) const
{
  if (distance <= parameter.distance_velocity_zero) {
    return 0.0f;
  }
  else if (distance >= parameter.distance_reduce_velocity) {
    return 1.0f;
  }
  // else
  // in distance interval for reducing velocity

  // remove offset (zero velocity distance) from value
  float reduce_factor = (distance - parameter.distance_velocity_zero);
  // normalize distance to zero velocity
  reduce_factor /= (parameter.distance_reduce_velocity - parameter.distance_velocity_zero);

  return reduce_factor;
}

std::string CollisionAvoidanceLidar::getFrameIdPrefix() const
{
  // remove slash at the beginning
  std::string frame_id_prefix(get_effective_namespace().begin() + 1, get_effective_namespace().end());
  // add slash at the end if it is missing
  if (frame_id_prefix.back() != '/') {
    frame_id_prefix.push_back('/');
  }
  return frame_id_prefix;
}

} // end namespace fleet
} // end namespace eduart

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eduart::fleet::CollisionAvoidanceLidar>());
  rclcpp::shutdown();

  return 0;
}