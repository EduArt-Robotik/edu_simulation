import os
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration, NotSubstitution, PathJoinSubstitution)

def generate_launch_description():
    # robot namespace
    edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
    edu_robot_namespace_arg = DeclareLaunchArgument(
        'edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', default='eduard')
    )

    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")    
    use_sim_time = LaunchConfiguration('use_sim_time')
 
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False' if os.environ.get('USE_SIM_TIME') != '1' else 'True',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("slam_toolbox"),
                                   'config', 'mapper_params_online_sync.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')


    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'base_frame': PathJoinSubstitution([edu_robot_namespace, 'base_footprint']),
        'map_frame': PathJoinSubstitution([edu_robot_namespace, 'map']),
        'odom_frame': PathJoinSubstitution([edu_robot_namespace, 'odom']),
        'scan_topic': 'scan'
    }
 
    # RewrittenYaml returns a path to a temporary YAML file with substitutions applied
    parameter_file = PathJoinSubstitution([
        FindPackageShare('edu_simulation'),
        'parameter',
        'slam.yaml'
    ])
    configured_params = RewrittenYaml(
        source_file=parameter_file,
        root_key=edu_robot_namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
 
    start_sync_slam_toolbox_node = LifecycleNode(
        parameters=[
          configured_params,
          {
            'use_sim_time': use_sim_time,
            'use_lifecycle_manager': use_lifecycle_manager,
          }
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace=edu_robot_namespace,
        remappings=[('/map', 'map')],
        output='screen'
      )
 
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_sync_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_sync_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_sync_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    ld = LaunchDescription()
 
    ld.add_action(edu_robot_namespace_arg)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_sync_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)
 
    return ld