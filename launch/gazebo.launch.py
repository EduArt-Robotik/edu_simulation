import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ros_gz_bridge.actions import RosGzBridge

def generate_launch_description():
  # Launch Arguments
  world = LaunchConfiguration('world')
  world_arg = DeclareLaunchArgument('world', default_value='depot.world')
  
  # Setting GZ Paths
  package_name = 'edu_simulation'
  model_path = os.path.join(get_package_share_directory(package_name), 'model')
  model_path += ':' + os.path.join(get_package_share_directory(package_name), 'world')
  plugin_path = os.path.join(get_package_prefix(package_name), 'lib')

  # Gazebo Harmonic
  gz_sim_launch_file = PathJoinSubstitution([
    FindPackageShare('ros_gz_sim'),
    'launch',
    'gz_sim.launch.py'
  ])

  gz_sim = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(gz_sim_launch_file),
    launch_arguments=[(
      'gz_args', [
        '--render-engine ',
        'ogre2 ',
        world,
        # ' -v 4',
        # ' --gui-config ',
        # PathJoinSubstitution([FindPackageShare(package_name), 'config', 'gazebo.config'])
      ]
    )]
  )

  # Gazebo ROS2 Bridge
  bridge_parameter_file = PathJoinSubstitution([
    FindPackageShare('edu_simulation'),
    'parameter',
    'gz_ros_bridge.yaml'
  ])
  bridge_launch_file = PathJoinSubstitution([
    FindPackageShare('ros_gz_bridge'),
    'launch',
    'ros_gz_bridge.launch.py'
  ])

  # bridge = IncludeLaunchDescription(
  #   PythonLaunchDescriptionSource(bridge_launch_file),
  #   launch_arguments={
  #     'config_file': bridge_parameter_file,
  #     'bridge_name': 'gz_ros_bridge'
  #   }.items()
  # )

  bridge = RosGzBridge(
    bridge_name='clock_bridge',
    config_file=bridge_parameter_file
  )

  # create and return launch description object
  return LaunchDescription([
    world_arg,
    SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=model_path),
    SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=plugin_path),
    gz_sim,
    bridge
  ])
