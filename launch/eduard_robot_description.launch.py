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

  # Publishing Robot Description
  simulation_package = FindPackageShare('edu_simulation')
  eduard_xacro_file = PathJoinSubstitution([simulation_package, 'model', 'eduard', 'eduard.urdf'])

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    namespace=edu_robot_namespace,
    output='screen',
    parameters=[
      {'use_sim_time': True},
      {'robot_description': Command([
        'xacro', ' ', eduard_xacro_file, ' ',
        'visualize_rays:=', visualize_rays, ' ',
        'wheel_type:=', wheel_type, ' ',
        'robot_name:=', edu_robot_namespace
      ])},
    ],
  )

  joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    namespace=edu_robot_namespace,
    output='screen',
    parameters=[{'use_sim_time': True}],
  )

  return LaunchDescription([
    edu_robot_namespace_arg,
    wheel_type_arg,
    visualize_rays_arg,
    robot_state_publisher,
    joint_state_publisher
  ])