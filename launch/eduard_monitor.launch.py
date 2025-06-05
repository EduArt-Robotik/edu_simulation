import os

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

    # RViz Config
    package = FindPackageShare('edu_robot_control')
    rviz_config = PathJoinSubstitution([
      package,
      'parameter',
      'eduard.rviz'
    ])

    rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      namespace=edu_robot_namespace,
      arguments=['-d', rviz_config],
      parameters=[
        {'use_sim_time': True}
      ],
      remappings=[
        ('/goal_pose', 'goal_pose')
      ]
    )

    # Robot Description for Eduard
    robot_description_launch_file = PathJoinSubstitution([
      package,
      'launch',
      'eduard_robot_description.launch.py'
    ])
    robot_description = IncludeLaunchDescription(PythonLaunchDescriptionSource(robot_description_launch_file))

    return LaunchDescription([
      edu_robot_namespace_arg,
      rviz_node,
      robot_description
    ])
