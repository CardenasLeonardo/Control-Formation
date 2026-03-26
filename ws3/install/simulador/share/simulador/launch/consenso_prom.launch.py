import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    n = int(LaunchConfiguration('n_robots').perform(context))
    R = float(LaunchConfiguration('neighbor_radius').perform(context))
    leader_id = LaunchConfiguration('leader_id').perform(context)

    articubot_pkg = get_package_share_directory('articubot_one')
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )
    rsp_launch = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')

    actions = []

    # Lanzar Gazebo
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        )
    )

    # Spawn robots + nodo de consenso con líder
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

        # consenso con líder — vecinos filtrados por AIRE
        actions.append(
            Node(
                package='control_nodes',
                executable='consenso_lider_node',
                name='consenso_lider',
                namespace=ns,
                parameters=[{
                    'leader_id': leader_id,
                    'use_sim_time': True
                }],
                output='screen'
            )
        )

    # AIRE
    actions.append(
        Node(
            package='simulador',
            executable='aire',
            name='aire',
            parameters=[{
                'neighbor_radius': R
            }],
            output='screen'
        )
    )

    # States plotter
    actions.append(
        Node(
            package='simulador',
            executable='states_plotter_v2',
            name='states_plotter_v2',
            output='screen'
        )
    )

    # PVA plotter
    actions.append(
        Node(
            package='simulador',
            executable='pva_plotter',
            name='pva_plotter',
            output='screen'
        )
    )

    return actions


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'n_robots',
            default_value='3',
            description='Número de robots'
        ),
        DeclareLaunchArgument(
            'neighbor_radius',
            default_value='3.0',
            description='Radio de percepción (metros)'
        ),
        DeclareLaunchArgument(
            'leader_id',
            default_value='robot0',
            description='ID del robot líder'
        ),
        OpaqueFunction(function=launch_setup)
    ])