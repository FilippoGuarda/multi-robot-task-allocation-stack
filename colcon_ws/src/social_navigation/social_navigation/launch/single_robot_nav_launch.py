import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Package directories
    social_navigation_dir = get_package_share_directory('social_navigation')
    social_navigation_worlds_dir = os.path.join(social_navigation_dir, 'worlds')
    
    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Robot namespace'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(social_navigation_dir, 'configs', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    # Robot State Publisher
    xacro_file = os.path.join(social_navigation_worlds_dir, 'turtlebot3_waffle.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file, ' namespace:=', namespace])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'robot_description': robot_description,
            'frame_prefix': [namespace, '/']
        }],
        output='screen'
    )
    
    # Create custom parameter files for this robot
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'robot_base_frame': [namespace, '/base_link'],
        'global_frame': 'map',
        'odom_frame': [namespace, '/odom']
    }
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True
    )
    
    # AMCL for localization
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('/tf', [namespace, '/tf']),
            ('/tf_static', [namespace, '/tf_static']),
            ('/map', '/map')
        ]
    )
    
    # Controller server (modified to use shared costmap)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('global_costmap/costmap', '/shared_costmap/costmap'),
            ('global_costmap/costmap_updates', '/shared_costmap/costmap_updates'),
            ('/tf', [namespace, '/tf']),
            ('/tf_static', [namespace, '/tf_static'])
        ]
    )
    
    # Planner server (modified to use shared costmap)
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('global_costmap/costmap', '/shared_costmap/costmap'),
            ('global_costmap/costmap_updates', '/shared_costmap/costmap_updates'),
            ('/tf', [namespace, '/tf']),
            ('/tf_static', [namespace, '/tf_static'])
        ]
    )
    
    # Behavior Tree Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('/tf', [namespace, '/tf']),
            ('/tf_static', [namespace, '/tf_static']),
            ('/map', '/map')
        ]
    )
    
    # Lifecycle manager for navigation nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'autostart': ParameterValue(autostart, value_type=bool),
            'node_names': [
                'amcl',
                'controller_server',
                'planner_server',
                'bt_navigator'
            ]
        }]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    
    # Add nodes
    ld.add_action(robot_state_publisher)
    ld.add_action(amcl_node)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(bt_navigator)
    ld.add_action(lifecycle_manager)
    
    return ld