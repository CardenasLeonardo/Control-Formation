import os
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    R   = float(LaunchConfiguration('neighbor_radius').perform(context))
    sep = float(LaunchConfiguration('separation').perform(context))

    articubot_pkg = get_package_share_directory('articubot_one')
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )
    rsp_launch = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')

    # Cuadrícula 3x3 centrada en el origen
    grid = [
        (col * sep, row * sep)
        for row in range(3)
        for col in range(3)
    ]

    # Robot suelto a la derecha al centro
    x_right = 2 * sep + sep          # columna 3 + separación extra
    y_right  = sep                    # fila del medio
    extra = [(x_right, y_right)]

    positions = grid + extra
    n = len(positions)               # 10 robots

    actions = []

    # Lanzar Gazebo
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        )
    )

    # Spawn robots + nodo de consenso
    for i, (x0, y0) in enumerate(positions):

        ns = f'robot{i}'

        # robot_state_publisher
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rsp_launch),
                launch_arguments={
                    'namespace': ns,
                    'use_sim_time': 'false'
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
                    '-x', str(round(x0, 4)),
                    '-y', str(round(y0, 4)),
                    '-z', '0.1'
                ],
                output='screen'
            )
        )

        # consenso promedio
        actions.append(
            Node(
                package='control_nodes',
                executable='consenso_prom_err',
                name='consenso_prom_err',
                namespace=ns,
                parameters=[{
                    'use_sim_time': False
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

    return actions


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'neighbor_radius',
            default_value='15.0',
            description='Radio de percepción (metros)'
        ),
        DeclareLaunchArgument(
            'separation',
            default_value='10.0',
            description='Separación entre robots (metros)'
        ),
        OpaqueFunction(function=launch_setup)
    ])