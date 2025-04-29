 /**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <laser_geometry/laser_geometry.hpp>

#include <array>
#include <memory>

namespace eduart {
namespace fleet {

class CollisionAvoidanceLidar : public rclcpp::Node
{
public:
  struct Parameter {
    float distance_reduce_velocity = 0.4f;
    float distance_velocity_zero = 0.05;
    struct {
      float width = 0.42f;
      float length = 0.40f;
    } size;
    std::string tf_base_link = "base_link";
  };

  CollisionAvoidanceLidar();
  CollisionAvoidanceLidar(const CollisionAvoidanceLidar &) = delete;
  CollisionAvoidanceLidar(CollisionAvoidanceLidar &&) = delete;
  CollisionAvoidanceLidar &operator=(const CollisionAvoidanceLidar &) = delete;
  CollisionAvoidanceLidar &operator=(CollisionAvoidanceLidar &&) = delete;
  ~CollisionAvoidanceLidar() override = default;

  static Parameter get_parameter(const Parameter& default_parameter, rclcpp::Node& ros_node);

private:
  enum Area {
    FRONT = 0,
    LEFT,
    RIGHT,
    REAR,
    COUNT
  };

  void callbackLaserScan(std::shared_ptr<const sensor_msgs::msg::LaserScan> msg);
  void callbackPointCloud(const sensor_msgs::msg::PointCloud2& msg);
  void callbackVelocity(std::shared_ptr<const geometry_msgs::msg::Twist> msg);
  float calculateReduceFactor(const float distance, const Parameter& parameter) const;
  std::string getFrameIdPrefix() const;

  const Parameter _parameter;
  struct {
    std::array<bool, Area::COUNT> intersection;
    std::array<float, Area::COUNT> reduce_factor;
    std::array<Eigen::Vector2f, Area::COUNT> left_bottom;
    std::array<Eigen::Vector2f, Area::COUNT> right_top;
  } _processing_data;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::LaserScan>> _sub_laser_scan;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> _sub_point_cloud;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> _sub_velocity;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> _pub_velocity;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> _pub_debug;

  std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
  std::shared_ptr<laser_geometry::LaserProjection> _laser_projection;
};

} // end namespace fleet
} // end namespace eduart
