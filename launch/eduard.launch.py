import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  # Arguments
  edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
  edu_robot_namespace_arg = DeclareLaunchArgument('edu_robot_namespace', default_value='eduard')

  wheel_type = LaunchConfiguration('wheel_type')
  wheel_type_arg = DeclareLaunchArgument('wheel_type', default_value='mecanum')

  pos_x = LaunchConfiguration('pos_x')
  pos_x_arg = DeclareLaunchArgument('pos_x', default_value='0.0')

  pos_y = LaunchConfiguration('pos_y')
  pos_y_arg = DeclareLaunchArgument('pos_y', default_value='0.0')

  pos_z = LaunchConfiguration('pos_z')
  pos_z_arg = DeclareLaunchArgument('pos_z', default_value='0.07')

  yaw = LaunchConfiguration('yaw')
  yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

  # Eduard
  eduard_launch_file = PathJoinSubstitution([
    FindPackageShare('edu_simulation'),
    'launch',
    'spawn_eduard.launch.py'
  ])

  eduard = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(eduard_launch_file),
    launch_arguments={
      'edu_robot_namespace': edu_robot_namespace,
      'wheel_type': wheel_type,
      "pos_x": pos_x,
      "pos_y": pos_y,
      "pos_z": pos_z,
      "yaw": yaw
    }.items()
  )

  # Lidar    
  tf_laser_eduard = Node(
    package='tf2_ros',
    name='tf_publish_laser',
    executable='static_transform_publisher',
    arguments=[
      '0.11', '0.0', '0.125', '0', '0', '0',
      PathJoinSubstitution([edu_robot_namespace, 'base_link']),
      PathJoinSubstitution([edu_robot_namespace, 'laser'])
    ],
    parameters=[{'use_sim_time': True}]
  )

  # Bridging Topics
  gz_ros_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='topic_bridge',
    output='screen',
    namespace=edu_robot_namespace,
    arguments=[
      ['/', edu_robot_namespace, '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan']
    ],
    parameters=[{
      'use_sim_time': True
    }]
  )

  return LaunchDescription([
    edu_robot_namespace_arg,
    wheel_type_arg,
    pos_x_arg,
    pos_y_arg,
    pos_z_arg,
    yaw_arg,    
    eduard,
    tf_laser_eduard,
    gz_ros_bridge
  ])