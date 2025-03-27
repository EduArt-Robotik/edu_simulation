import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Arguments
    world_name = LaunchConfiguration('world')
    world_name_arg = DeclareLaunchArgument('world', default_value='empty.world')

    # Needed path by Gazebo
    package_path = FindPackageShare('edu_simulation')
    model_path = PathJoinSubstitution([package_path, 'model:/opt/ros/humble/share/turtlebot3_gazebo/models/'])
    plugin_path = PathJoinSubstitution([package_path, 'lib'])
    world = PathJoinSubstitution([package_path, 'world', world_name])
  
    # TF transforms of lidar for different robots    
    tf_laser_eduard_blue = Node(
      package='tf2_ros',
      name='tf_publish_laser',
      executable='static_transform_publisher',
      arguments=[
        '0.11', '0.0', '0.125', '0', '0', '0',
        'eduard/blue/base_link',
        'eduard/blue/laser'
      ],
      parameters=[{'use_sim_time': True}]
    )
    tf_laser_eduard_green = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=[
        '0.11', '0.0', '0.125', '0', '0', '0',
        'eduard/green/base_link',
        'eduard/green/laser'
      ],
      parameters=[{'use_sim_time': True}]
    )
    tf_laser_eduard_red = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=[
        '0.11', '0.0', '0.125', '0', '0', '0',
        'eduard/red/base_link',
        'eduard/red/laser'
      ],
      parameters=[{'use_sim_time': True}]      
    )

    # Gazebo
    gazebo_server_launch_file = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gzserver.launch.py'
    ])
    gazebo_param_file = PathJoinSubstitution([
        package_path,
        'parameter',
        'gazebo.yaml'
    ])
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_server_launch_file),
        launch_arguments={
            'params_file': gazebo_param_file,
            'world': world,
            'extra_gazebo_args': ['--ros-args', '-r', 'cmd_vel:=safety/cmd_vel', '-p', 'hans:=cool']
        }.items()
    )

    gazebo_client_launch_file = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gzclient.launch.py'        
    ])
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_client_launch_file)
    )

    # create and return launch description object
    return LaunchDescription([
        world_name_arg,
        # SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[EnvironmentVariable('GAZEBO_MODEL_PATH'), ':' + model_path]),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=plugin_path),
        # start gazebo, notice we are using libgazebo_ros_factory.so instead of libgazebo_ros_init.so
        # That is because only libgazebo_ros_factory.so contains the service call to /spawn_entity
        # ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '--ros-args', '-p', 'publish_rate:=500.0', '--', world], output='screen'),
        gazebo_server,
        gazebo_client,
        tf_laser_eduard_blue,
        tf_laser_eduard_green,
        tf_laser_eduard_red
    ])