from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    simulation_share = get_package_share_directory("edu_simulation")
    robot_control_share = get_package_share_directory("edu_robot_control")

    world_arg = DeclareLaunchArgument("world", default_value="pepperl+fuchs.world", description="Gazebo world to load")
    wheel_type_arg = DeclareLaunchArgument("wheel_type", default_value="mecanum", description="Robot wheel configuration")
    lidar_type_arg = DeclareLaunchArgument("lidar_type", default_value="livox_mid360", description="LiDAR configuration")


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_share,
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
        }.items()
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_share,
                "launch",
                "eduard.launch.py"
            )
        ),
        launch_arguments={
            "wheel_type": LaunchConfiguration("wheel_type"),
            "lidar_type": LaunchConfiguration("lidar_type"),
            "pos_x": '0.85',
            "pos_y": '0.0',
            "pos_z": '1.0',
        }.items()
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                robot_control_share,
                "launch",
                "robot_remote_control_ipc127e.launch.py"
            )
        )
    )

    return LaunchDescription([
        world_arg,
        wheel_type_arg,
        lidar_type_arg,

        gazebo_launch,
        robot_launch,
        control_launch,
    ])