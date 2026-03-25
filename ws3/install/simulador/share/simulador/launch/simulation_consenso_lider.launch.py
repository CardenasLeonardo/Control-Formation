import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    n = int(LaunchConfiguration('n_robots').perform(context))

    articubot_pkg = get_package_share_directory('articubot_one')

    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    rsp_launch = os.path.join(
        articubot_pkg,
        'launch',
        'rsp.launch.py'
    )

    actions = []

    # -------------------------------------------------
    # 1. Lanzar Gazebo
    # -------------------------------------------------

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        )
    )

    # -------------------------------------------------
    # 2. Nodo AIRE (relay de comunicación)
    # -------------------------------------------------

    actions.append(
        Node(
            package='control_nodes',
            executable='aire',
            output='screen'
        )
    )

    # -------------------------------------------------
    # 3. Spawn robots
    # -------------------------------------------------

    for i in range(n):

        ns = f'robot{i}'

        # robot_state_publisher

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rsp_launch),
                launch_arguments={
                    'namespace': ns,
                    'use_sim_time': 'true'
                }.items()
            )
        )

        # spawn_entity

        actions.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', f'/{ns}/robot_description',
                    '-entity', ns,
                    '-x', str(i * 1.5),
                    '-y', '0.0',
                    '-z', '0.1'
                ],
                output='screen'
            )
        )

    # -------------------------------------------------
    # 4. Controladores
    # -------------------------------------------------

    for i in range(n):

        ns = f'robot{i}'

        # robot0 → líder

        if i == 0:

            actions.append(
                Node(
                    package='control_nodes',
                    executable='navigate_individual_pva',
                    namespace=ns,
                    output='screen'
                )
            )

        # robot1..N → consenso líder

        else:

            actions.append(
                Node(
                    package='control_nodes',
                    executable='consenso_lider_node',
                    namespace=ns,
                    output='screen'
                )
            )

    return actions


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'n_robots',
            default_value='10',
            description='Number of robots'
        ),

        OpaqueFunction(function=launch_setup)
    ])