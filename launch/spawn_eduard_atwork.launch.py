from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
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

  # fixed robot/entity name (kein Namespace)
  robot_name = 'eduard_atwork'

  simulation_package = FindPackageShare('edu_simulation')
  eduard_xacro_file = PathJoinSubstitution([simulation_package, 'model', 'eduard_atwork', 'eduard_atwork.urdf'])

  spawner = Node(
    package='ros_gz_sim',
    executable='create',
    output='screen',
    arguments=[
      '-name', robot_name,
      '-x', pos_x,
      '-y', pos_y,
      '-z', pos_z,
      '-Y', yaw,
      '-string', Command([
        'xacro', ' ', eduard_xacro_file, ' ',
        'visualize_rays:=', visualize_rays, ' ',
        'wheel_type:=', wheel_type, ' ',
        'robot_name:=', robot_name
      ])
    ]
  )

  return LaunchDescription([
    wheel_type_arg,
    visualize_rays_arg,
    pos_x_arg,
    pos_y_arg,
    pos_z_arg,
    yaw_arg,
    spawner
  ])
