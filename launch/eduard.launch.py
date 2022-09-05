import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable

def generate_launch_description():
    robot_name = 'edu_simulation'
    world_file_name = 'empty.world'

    # full  path to sdf and world file
    
    # world = os.path.join(get_package_share_directory(robot_name), 'world', world_file_name)
    model_path = os.path.join(get_package_share_directory(robot_name), 'model')
    world = "worlds/empty.world"
    robot_sdf = os.path.join(model_path, 'eduard/eduard.sdf')
    
    # read sdf contents because to spawn an entity in 
    # gazebo we need to provide entire sdf as string on  command line

    xml = open(robot_sdf, 'r').read()

    # double quotes need to be with escape sequence
    xml = xml.replace('"', '\\"')

    # this is argument format for spwan_entity service 
    spwan_args = '{name: \"eduard\", xml: \"'  +  xml + '\" }'
    
    # create and return launch description object
    return LaunchDescription([
        #SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[EnvironmentVariable('GAZEBO_MODEL_PATH'), ':' + model_path]),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        # start gazebo, notice we are using libgazebo_ros_factory.so instead of libgazebo_ros_init.so
        # That is because only libgazebo_ros_factory.so contains the service call to /spawn_entity
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world ],#'-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        # tell gazebo to spwan your robot in the world by calling service
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spwan_args],
            output='screen'),
    ])