import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution

def generate_launch_description():
  # launch file arguments
  # robot namespace
  edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
  edu_robot_namespace_arg = DeclareLaunchArgument(
      'edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', default='eduard')
  )

  # simulation
  simulation_package_path = FindPackageShare('edu_simulation')
  simulation_launch_file = PathJoinSubstitution([
    simulation_package_path,
    'launch',
    'eduard.launch.py'
  ])
  simulation = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(simulation_launch_file),
    launch_arguments={
      'world': 'eduard_blue_nav2.world'
    }.items()
  )

  # SLAM
  slam_launch_file = PathJoinSubstitution([
    simulation_package_path,
    'launch',
    'slam.launch.py'
  ])
  slam = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(slam_launch_file),
    launch_arguments={
      'use_sim_time': 'True',
      'edu_robot_namespace': 'eduard/blue'
    }.items()
  )

  # robot control
  robot_control_launch_file = PathJoinSubstitution([
    FindPackageShare('edu_robot_control'),
    'launch',
    'robot_remote_control_raspberry.launch.py'
  ])
  robot_control = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(robot_control_launch_file),
    launch_arguments={
      'edu_robot_namespace': 'eduard/blue'      
    }.items()
  )

  # RViz
  rviz_launch_file = PathJoinSubstitution([
    FindPackageShare('edu_robot_control'),
    'launch',
    'eduard_monitor_nav2.launch.py'
  ])
  rviz = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(rviz_launch_file),
    launch_arguments={
      'use_sim_time': 'True',
      'edu_robot_namespace': 'eduard/blue'      
    }.items()
  )

  return LaunchDescription([
    edu_robot_namespace_arg,
    simulation,
    # slam,
    robot_control,
    # rviz
  ])