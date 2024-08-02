import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution

def generate_launch_description():
    package_name = 'edu_simulation'
    
    # world = os.path.join(get_package_share_directory(robot_name), 'world', world_file_name)
    model_path = os.path.join(get_package_share_directory(package_name), 'model')
    plugin_path = os.path.join(get_package_share_directory(package_name), 'lib')
    print('plugin path = ', plugin_path)
    world = "worlds/empty.world"
    

    # create and return launch description object
    return LaunchDescription([
        # SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[EnvironmentVariable('GAZEBO_MODEL_PATH'), ':' + model_path]),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=plugin_path),
        # start gazebo, notice we are using libgazebo_ros_factory.so instead of libgazebo_ros_init.so
        # That is because only libgazebo_ros_factory.so contains the service call to /spawn_entity
        ExecuteProcess(cmd=['gazebo', '-s', 'libgazebo_ros_init.so', 'libgazebo_ros_factory.so', '--verbose', world ], output='screen'),
    ])
