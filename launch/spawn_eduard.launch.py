from launch import LaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, Command
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  # Arguments
  edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
  edu_robot_namespace_arg = DeclareLaunchArgument(
    'edu_robot_namespace', default_value='eduard'
  )

  wheel_type = LaunchConfiguration('wheel_type')
  wheel_type_arg = DeclareLaunchArgument(
    'wheel_type', default_value='mecanum'
  )

  visualize_rays = LaunchConfiguration('visualize_rays')
  visualize_rays_arg = DeclareLaunchArgument(
    'visualize_rays', default_value='false'
  )

  # Spawn Robot based on URDF File
  simulation_package = FindPackageShare('edu_simulation')
  eduard_xacro_file = PathJoinSubstitution([simulation_package, 'model', 'eduard', 'eduard.urdf'])

  spawner = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
      '-name', edu_robot_namespace,
    #   '-x', x,
    #   '-y', y,
    #   '-z', z,
    #   '-Y', yaw,
      '-string', Command([
        'xacro', ' ', eduard_xacro_file, ' ',
        'visualize_rays:=', visualize_rays, ' ',
        'wheel_type:=', wheel_type, ' ',
        'robot_name:=', edu_robot_namespace
      ])
    ]
  )

  return LaunchDescription([
    edu_robot_namespace_arg,
    wheel_type_arg,
    visualize_rays_arg,
    spawner
  ])