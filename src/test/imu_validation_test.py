#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from std_srvs.srv import Trigger

from edu_robot.srv import SetMode
from edu_robot.msg import Mode

from gz import transport
from gz.msgs.pose_pb2 import Pose

class ImuValidationTest(Node):
  def __init__(self):
    super().__init__("imu_validation_test")

    # getting robot pose from Gazebo as feedback for controlling the robot
    # Note: the robot logs its poses in Gazebo to a file, too. Lets see if there is a difference...
    self.gz_node = transport.Node()
    self.gz_node.subscribe(topic='/world/eduard/pose', msg_type=Pose, callback=self.ground_truth_callback)
    self.log_robot_pose = open('robot_pose_log.csv', 'w', encoding='utf-8')
    self.log_robot_pose.write("time,position_x,position_y,position_z,orientation_x,orientation_y,orientation_z,orientation_w\n")

    # for odometry feedback as alternative to Gazebo pose
    self.sub_odometry = self.create_subscription(Odometry, '/eduard/odometry', self.odometry_callback, QoSProfile(
      depth=10, reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST)
    )

    # for controlling the robot
    self.pub_twist = self.create_publisher(Twist, '/eduard/cmd_vel', 10)
    self.way_points = [(2.0, 0), (2.0, 2.0), (0, 2.0), (0, 0)]
    self.current_way_point_index = 0
    self.velocity = 0.5
    self.way_point_error_threshold = 0.01

    self.srv_set_mode = self.create_client(SetMode,'/eduard/set_mode')
    self.srv_reset_odometry = self.create_client(Trigger, '/eduard/reset_odometry')

    # for validating the IMU data
    self.sub_imu = self.create_subscription(Imu, '/eduard/imu', self.imu_callback, QoSProfile(
      depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST)
    )
    self.log_imu_data = open('imu_data_log.csv', 'w', encoding='utf-8')
    self.log_imu_data.write("time,linear_accel_x,linear_accel_y,linear_accel_z,angular_vel_x,angular_vel_y,angular_vel_z\n")

    # enable the robot and reset odometry before starting the test
    self.reset_odometry()
    self.enable_robot()

  def process_feedback(self, position_x, position_y):
    # controlling the robot to move in a way points
    if self.current_way_point_index < len(self.way_points):
      target_x, target_y = self.way_points[self.current_way_point_index]

      # simple proportional controller where the velocity is the P term
      error_x = target_x - position_x
      error_y = target_y - position_y

      twist_msg = Twist()
      twist_msg.linear.x = self.velocity * max(min(error_x, 1.0), -1.0)
      twist_msg.linear.y = self.velocity * max(min(error_y, 1.0), -1.0)
      self.pub_twist.publish(twist_msg)

      # check if we are close enough to the target way point
      if abs(error_x) < self.way_point_error_threshold and abs(error_y) < self.way_point_error_threshold:
        self.current_way_point_index += 1
        self.get_logger().info(f"Reached way point {self.current_way_point_index}, moving to next")

    # if we have reached all way points, stop the robot
    else:
      self.get_logger().info("All way points reached, stopping the robot")
      self.pub_twist.publish(Twist())  # stop the robot
      rclpy.shutdown()    

  def ground_truth_callback(self, msg):
    # storing for later use, e.g. for plotting the trajectory
    seconds = msg.time_sec + msg.time_nsec * 1e-9
    self.log_robot_pose.write(f"{seconds},{msg.position.x},{msg.position.y},{msg.position.z},{msg.orientation.x},{msg.orientation.y},{msg.orientation.z},{msg.orientation.w}\n")

    # process feedback for controlling the robot
    self.process_feedback(msg.position.x, msg.position.y)

  def odometry_callback(self, msg):
    # process feedback without logging
    self.process_feedback(msg.pose.pose.position.x, msg.pose.pose.position.y)

  def imu_callback(self, msg):
    # log the IMU data for later analysis
    seconds = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    self.log_imu_data.write(f"{seconds},{msg.linear_acceleration.x},{msg.linear_acceleration.y},{msg.linear_acceleration.z},{msg.angular_velocity.x},{msg.angular_velocity.y},{msg.angular_velocity.z}\n")  

  def enable_robot(self):
    set_mode_request = SetMode.Request()
    set_mode_request.mode.mode = Mode.REMOTE_CONTROLLED
    set_mode_request.mode.drive_kinematic = Mode.MECANUM_DRIVE
    set_mode_request.mode.feature_mode = Mode.COLLISION_AVOIDANCE_OVERRIDE

    print('Enabling Eduard')
    assert self.srv_set_mode.service_is_ready() is True
    future = self.srv_set_mode.call_async(set_mode_request)


  def disable_robot(self):
    print('Disabling Eduard')
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    self.pub_twist.publish(twist_msg)

    set_mode_request = SetMode.Request()
    set_mode_request.mode.mode = Mode.INACTIVE
    set_mode_request.mode.drive_kinematic = Mode.MECANUM_DRIVE
    set_mode_request.mode.feature_mode = Mode.COLLISION_AVOIDANCE_OVERRIDE

    assert self.srv_reset_odometry.service_is_ready() is True
    future = self.srv_set_mode.call_async(set_mode_request)
    rclpy.spin_until_future_complete(self.node, future)

    assert future.result().state.mode.mode is Mode.INACTIVE

  def reset_odometry(self):
    print('Resetting Odometry')
    assert self.srv_reset_odometry.service_is_ready() is True
    future = self.srv_reset_odometry.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(self, future)

    assert future.result().success is True


def main():
    rclpy.init()
    node = ImuValidationTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
