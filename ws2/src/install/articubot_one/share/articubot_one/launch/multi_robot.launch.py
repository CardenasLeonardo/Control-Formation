import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def spawn_robots(context, *args, **kwargs):

    num = int(context.launch_configurations['num_robots'])
    actions = []

    for i in range(num):

        namespace = f'robot{i}'

        # Lanzar RSP
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('articubot_one'),
                        'launch',
                        'rsp.launch.py'
                    ])
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': 'true'
                }.items()
            )
        )

        # Spawn en Gazebo
        actions.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', f'/{namespace}/robot_description',
                    '-entity', namespace,
                    '-x', str(i * 1.5),
                    '-y', '0.0',
                    '-z', '0.1'
                ],
                output='screen'
            )
        )

    return actions


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'num_robots',
            default_value='1',
            description='Number of robots'
        ),

        # Lanzar Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            )
        ),

        OpaqueFunction(function=spawn_robots)

    ])