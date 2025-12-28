from launch import LaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, Command
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  # Arguments
  edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
  edu_robot_namespace_arg = DeclareLaunchArgument('edu_robot_namespace', default_value='eduard')

  wheel_type = LaunchConfiguration('wheel_type')
  wheel_type_arg = DeclareLaunchArgument('wheel_type', default_value='mecanum')

  visualize_rays = LaunchConfiguration('visualize_rays')
  visualize_rays_arg = DeclareLaunchArgument('visualize_rays', default_value='false')

  chassis_mesh = LaunchConfiguration('chassis_mesh')
  chassis_mesh_arg = DeclareLaunchArgument('chassis_mesh', default_value='model://eduard/mesh/eduard-blue-chassis.dae')

  pos_x = LaunchConfiguration('pos_x')
  pos_x_arg = DeclareLaunchArgument('pos_x', default_value='0.0')

  pos_y = LaunchConfiguration('pos_y')
  pos_y_arg = DeclareLaunchArgument('pos_y', default_value='0.0')

  pos_z = LaunchConfiguration('pos_z')
  pos_z_arg = DeclareLaunchArgument('pos_z', default_value='0.07')

  yaw = LaunchConfiguration('yaw')
  yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

  # Spawn Robot based on URDF File
  simulation_package = FindPackageShare('edu_simulation')
  eduard_xacro_file = PathJoinSubstitution([simulation_package, 'model', 'eduard', 'eduard.urdf'])

  spawner = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
      '-name', edu_robot_namespace,
      '-x', pos_x,
      '-y', pos_y,
      '-z', pos_z,
      '-Y', yaw,
      '-string', Command([
        'xacro', ' ', eduard_xacro_file, ' ',
        'visualize_rays:=', visualize_rays, ' ',
        'chassis_mesh:=', chassis_mesh, ' ',
        'wheel_type:=', wheel_type, ' ',
        'robot_name:=', edu_robot_namespace
      ])
    ]
  )

  return LaunchDescription([
    edu_robot_namespace_arg,
    wheel_type_arg,
    visualize_rays_arg,
    pos_x_arg,
    pos_y_arg,
    pos_z_arg,
    yaw_arg,
    chassis_mesh_arg,
    spawner
  ])