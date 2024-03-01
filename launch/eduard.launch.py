import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution

def generate_launch_description():
    robot_name = 'edu_simulation'
    world_file_name = 'empty.world'

    # full  path to sdf and world file
    
    # world = os.path.join(get_package_share_directory(robot_name), 'world', world_file_name)
    model_path = os.path.join(get_package_share_directory(robot_name), 'model')
    plugin_path = os.path.join(get_package_share_directory(robot_name), 'lib')
    print('plugin path = ', plugin_path)
    world = "worlds/empty.world"
    robot_sdf = os.path.join(model_path, 'eduard/eduard.sdf')
    
    # read sdf contents because to spawn an entity in 
    # gazebo we need to provide entire sdf as string on  command line

    # xml = open(robot_sdf, 'r').read()

    # double quotes need to be with escape sequence
    # xml = xml.replace('"', '\\"')

    # this is argument format for spwan_entity service 
    # spwan_args = '{name: \"eduard\", xml: \"'  +  xml + '\" }'
    
    tf_laser_eduard_blue = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=[
        '0.11', '0.0', '0.125', '3.141592654', '0', '0',
        'eduard/blue/base_link',
        'eduard/blue/laser'
      ]
    )
    tf_laser_eduard_green = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=[
        '0.11', '0.0', '0.125', '3.141592654', '0', '0',
        'eduard/green/base_link',
        'eduard/green/laser'
      ]
    )
    tf_laser_eduard_red = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=[
        '0.11', '0.0', '0.125', '3.141592654', '0', '0',
        'eduard/red/base_link',
        'eduard/red/laser'
      ]
    )

    # create and return launch description object
    return LaunchDescription([
        # SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[EnvironmentVariable('GAZEBO_MODEL_PATH'), ':' + model_path]),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=plugin_path),
        # start gazebo, notice we are using libgazebo_ros_factory.so instead of libgazebo_ros_init.so
        # That is because only libgazebo_ros_factory.so contains the service call to /spawn_entity
        ExecuteProcess(cmd=['gazebo', '--verbose', world ], output='screen'),
        tf_laser_eduard_blue,
        tf_laser_eduard_green,
        tf_laser_eduard_red
    ])