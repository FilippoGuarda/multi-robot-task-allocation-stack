import os
import sys
import launch
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
import argparse
import json

def generate_launch_description():
    social_navigation_dir = get_package_share_directory('social_navigation')
    world_file_name = "aws_hospital_no_humans.world"
    world = os.path.join(social_navigation_dir, 'worlds', world_file_name)


    arguments = parse_arguments(sys.argv)
    scenario_file = get_scenario_file_from_arguments(arguments)
    if not scenario_file or not os.path.exists(scenario_file):
        print(f"Error: Scenario file not found at '{scenario_file}'")
        sys.exit(1)
        
    positions = get_robot_positions(scenario_file)
    robot_names = [f'robot{i + 1}' for i in range(len(positions))]


    declare_robot_urdf_cmd = DeclareLaunchArgument(
        'robot_urdf',
        default_value=os.path.join(social_navigation_dir, 'worlds', 'turtlebot3_waffle.urdf.xacro'),
        description='Full path to robot URDF file to spawn the robot in gazebo')


    robot_urdf = LaunchConfiguration('robot_urdf')
    
    gazebo_params_path = os.path.join(social_navigation_dir, 'configs', 'gazebo_params.yml')
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'params_file': gazebo_params_path,
                              'world': world,
                              'gui': 'true'}.items(),
    )

    ld = launch.LaunchDescription()
    ld.add_action(declare_robot_urdf_cmd)
    ld.add_action(launch_gazebo)

    for i, robot_name in enumerate(robot_names):
        namespace = robot_name
        pose = {
            'x': str(positions[i][0]),
            'y': str(positions[i][1]),
            'z': '0.01',
            'R': '0.00',
            'P': '0.00',
            'Y': '0.00'
        }

        # Create frame tree for the nth robot
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                # Process the URDF file with the current robot's namespace
                'robot_description': Command(['xacro ', robot_urdf, ' namespace:=', f"{namespace}/"]),
                'frame_prefix': f"{namespace}/"
            }]
        )

        # Spawn model of the robot in gazebo
        spawn_entity_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', robot_name,
                # The spawner listens to the 'robot_description' topic in its namespace
                '-topic', 'robot_description',
                '-robot_namespace', namespace,
                '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
                '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']
            ]
        )

        namespaced_group = GroupAction(
            actions=[
                # Push the robot's namespace to all nodes within this group
                PushRosNamespace(namespace),
                robot_state_publisher_node,
                spawn_entity_node
            ]
        )
        
        ld.add_action(namespaced_group)

    return ld

def get_robot_positions(file):
    """Load robot starting positions from scenario file"""
    with open(file, 'r') as f:
        scenario_setup = json.load(f) 
    positions = [] 
    for robot in scenario_setup["agents"].values():
        print(f"Robot: {robot}")
        positions.append(robot["start"])
    return positions

def get_scenario_file_from_arguments(argv):
    """Parse scenario file from command line arguments"""
    print(f"Arguments: {argv}")
    parser = argparse.ArgumentParser(
        description='Start multi-robot hospital simulation'
    )
    parser.add_argument('-input_file', type=str, help='Scenario file', required=True)
    args = parser.parse_args(argv)
    return args.input_file

def parse_arguments(argv):
    """Parse launch arguments"""
    args = []
    for arg in argv:
        if ":=" in arg:
            parsed_arg = arg.split(':')
            print(parsed_arg)
            if not parsed_arg[0] == 'use_sim_time' and not parsed_arg[0] == 'autostart':
                args.append(f"-{parsed_arg[0]}")
                args.append(parsed_arg[1][1:])
    return args

if __name__ == '__main__':
    generate_launch_description()
