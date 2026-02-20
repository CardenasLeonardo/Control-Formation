import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node


def spawn_robots(context, *args, **kwargs):

    n = int(LaunchConfiguration('n_robots').perform(context))

    pkg = get_package_share_directory('articubot_one')

    rsp_launch = os.path.join(pkg, 'launch', 'rsp.launch.py')

    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    actions = []

    # Launch Gazebo once
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'verbose': 'false',
            'pause': 'false'
        }.items()
    )
    actions.append(gazebo)

    # Spawn each robot with a small delay between each
    for i in range(n):
        ns = f"robot{i}"

        # robot_state_publisher
        rsp_action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch),
            launch_arguments={
                'namespace': ns,
                'use_sim_time': 'true'
            }.items()
        )
        
        # spawn_entity
        spawn_action = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            namespace=ns,
            arguments=[
                '-topic', f'/{ns}/robot_description',
                '-entity', ns,
                '-x', str(i * 1.5),  # 1.5m spacing between robots
                '-y', '0.0',
                '-z', '0.1',  # Slightly above ground to avoid spawn collision
                '-R', '0.0',
                '-P', '0.0',
                '-Y', '0.0'
            ],
            output='screen'
        )

        # Add a small delay for Gazebo to process each robot
        # First robot spawns immediately after Gazebo (3s)
        # Each subsequent robot waits an additional 2s
        #delay = 3.0 + (i * 2.0)
        delay = 0.1
        
        
        actions.append(TimerAction(
            period=delay,
            actions=[rsp_action, spawn_action]
        ))

    return actions


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'n_robots',
            default_value='3',
            description='Number of robots to spawn in simulation'),
        
        OpaqueFunction(function=spawn_robots)
    ])