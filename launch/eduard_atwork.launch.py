from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  # Args, die du ans spawn-file weiterreichen willst
  wheel_type = LaunchConfiguration('wheel_type')
  wheel_type_arg = DeclareLaunchArgument('wheel_type', default_value='mecanum')

  visualize_rays = LaunchConfiguration('visualize_rays')
  visualize_rays_arg = DeclareLaunchArgument('visualize_rays', default_value='false')

  pos_x = LaunchConfiguration('pos_x')
  pos_x_arg = DeclareLaunchArgument('pos_x', default_value='0.0')

  pos_y = LaunchConfiguration('pos_y')
  pos_y_arg = DeclareLaunchArgument('pos_y', default_value='0.0')

  pos_z = LaunchConfiguration('pos_z')
  pos_z_arg = DeclareLaunchArgument('pos_z', default_value='0.07')

  yaw = LaunchConfiguration('yaw')
  yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

  # Include spawn
  spawn_file = PathJoinSubstitution([
    FindPackageShare('edu_simulation'),
    'launch',
    'spawn_eduard_atwork.launch.py'
  ])

  spawn = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(spawn_file),
    launch_arguments={
      'wheel_type': wheel_type,
      'visualize_rays': visualize_rays,
      'pos_x': pos_x,
      'pos_y': pos_y,
      'pos_z': pos_z,
      'yaw': yaw
    }.items()
  )

  # Bridge both lidars
  # Wenn deine URDF-Sensoren <topic>scan/front</topic> und <topic>scan/back</topic> publizieren,
  # dann bridgen wir genau diese.
  gz_ros_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='topic_bridge',
    output='screen',
    arguments=[
      '/scan/front@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
      '/scan/back@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
    ],
    parameters=[{'use_sim_time': True}]
  )

  return LaunchDescription([
    wheel_type_arg,
    visualize_rays_arg,
    pos_x_arg,
    pos_y_arg,
    pos_z_arg,
    yaw_arg,
    spawn,
    gz_ros_bridge
  ])
