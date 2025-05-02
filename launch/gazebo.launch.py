import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  # Launch Arguments
  world = LaunchConfiguration('world')
  world_arg = DeclareLaunchArgument('world', default_value='depot.world')
  
  # Setting GZ Paths
  package_name = 'edu_simulation'
  model_path = os.path.join(get_package_share_directory(package_name), 'model')
  model_path += ':' + os.path.join(get_package_share_directory(package_name), 'world')
  plugin_path = os.path.join(get_package_share_directory(package_name), 'lib')

  print('model path: ', model_path)

  # Ignition gazebo
  gz_sim_launch_file = PathJoinSubstitution([
    FindPackageShare('ros_gz_sim'),
    'launch',
    'gz_sim.launch.py'
  ])

  gz_sim = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(gz_sim_launch_file),
    launch_arguments=[(
      'gz_args', [
        # '--render-engine ',
        # 'ogre2 ',
        world,
        ' -v 4',
        ' --gui-config ',
        PathJoinSubstitution([FindPackageShare(package_name), 'config', 'gazebo.config'])
      ]
    )]
  )

  # create and return launch description object
  return LaunchDescription([
    world_arg,
    SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=model_path),
    SetEnvironmentVariable(name='GZ_GUI_PLUGIN_PATH', value=plugin_path),
    gz_sim
  ])
