"""
Copyright 2025 Filippo Guarda
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.




WARNING: The namespacing in ROS2 is particularly finnicky, especially around
the nav2 stack, before trying to rewrite parts of this launch file refer to:
https://github.com/ros-navigation/navigation2/issues/2796
https://github.com/ros-navigation/navigation2/blob/galactic/nav2_bringup/bringup/launch/bringup_launch.py#L80-L82

"""


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    social_navigation_dir = get_package_share_directory('social_navigation')
    
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    
    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='robot1', description='Top-level namespace for the robot')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation (Gazebo) clock if true')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(social_navigation_dir, 'configs', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically startup the nav2 stack')
    
    declare_initial_pose_x_cmd = DeclareLaunchArgument(
        'initial_pose_x', default_value='0.0', description='Initial pose X coordinate')
    
    declare_initial_pose_y_cmd = DeclareLaunchArgument(
        'initial_pose_y', default_value='0.0', description='Initial pose Y coordinate')
    
    declare_initial_pose_yaw_cmd = DeclareLaunchArgument(
        'initial_pose_yaw', default_value='0.0', description='Initial pose Yaw angle')

    # This forces correct substitutions in the various parameter files
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'robot_base_frame': [namespace, '/base_footprint'], 
        'odom_frame_id': [namespace, '/odom'],
        'base_frame_id': [namespace, '/base_footprint'],
        'robot_base_frame': [namespace, '/base_footprint'], 
        'global_frame': [namespace, '/odom'],
        'topic': ['/', namespace, '/scan'],
        'costmap_topic': [namespace, '/local_costmap/costmap_raw'],
        'footprint_topic': [namespace, '/local_costmap/published_footprint'],
        'robot_namespace': namespace
    }
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    # Create a different param substitution set since the global costmap
    # uses map as the global frame
    param_substitutions_global_cost = param_substitutions.copy()
    param_substitutions_global_cost['global_frame'] = 'map'
    configured_params_global_cost = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions_global_cost,
        convert_types=True)


    remappings = [('map', '/map')]

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            configured_params,
            {'set_initial_pose': True,
             'initial_pose.x': initial_pose_x,
             'initial_pose.y': initial_pose_y,
             'initial_pose.yaw': initial_pose_yaw}
        ],
        remappings=remappings)
    
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)
    
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            configured_params_global_cost
            # {
            # 'multi_robot_layer.robot_namespace': namespace,
            # 'multi_robot_layer.enabled': True,
            # 'multi_robot_layer.shared_grid_topic': '/shared_obstacles_grid',
            # 'multi_robot_layer.robot_radius': 0.3,
            # 'multi_robot_layer.exclusion_buffer': 0.5
            # }
        ],
        remappings=remappings)
    
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params_global_cost],
        remappings=remappings)
    
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server', 
        name='behavior_server',
        output='screen',
        parameters=[configured_params_global_cost],
        remappings=remappings)
    
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': autostart,
                     'node_names': [
                         'amcl',
                         'controller_server',
                         'planner_server',
                         'behavior_server',
                         'bt_navigator'
                         ]}])
    
    ld = LaunchDescription()
    
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_initial_pose_x_cmd)
    ld.add_action(declare_initial_pose_y_cmd)
    ld.add_action(declare_initial_pose_yaw_cmd)
    
    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))
    
    nav2_bringup_group = GroupAction([
        PushRosNamespace(namespace=namespace),
        
        # Add all the nodes that need to be namespaced
        amcl_node,
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        lifecycle_manager_node
    ])
    
    ld.add_action(nav2_bringup_group)
    
    return ld
