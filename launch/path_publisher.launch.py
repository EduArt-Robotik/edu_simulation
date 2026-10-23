from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    config_file = PathJoinSubstitution([
        FindPackageShare('edu_simulation'),
        'parameter',
        'path_pub.yaml'
    ])

    path_pub_node = Node(
        package='edu_simulation',
        executable='path_publisher',
        name='path_publisher',
        output='screen',
        parameters=[
            config_file
        ],
        # namespace="eduard"
    )

    return LaunchDescription([
        path_pub_node
    ])