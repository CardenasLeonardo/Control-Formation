import os
import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


# Layout 8x9 m sin robot en el centro.
# Solo robot0 y robot7 apuntan HACIA el centroide.
# Los otros 6 apuntan LEJOS → movimiento en reversa.
# robot3 y robot4 apuntan exactamente opuestos a la dirección al centroide.
# Conectividad garantizada con R=5.0 m.
POSES_8 = [
    (0.0, 0.0,   math.pi / 4),       # robot0 — esquina BL → hacia centro  (único hacia adelante)
    (4.0, 1.0,  -math.pi / 2),       # robot1 — borde inf  → sur  (reversa)
    (8.0, 0.0,  -math.pi / 4),       # robot2 — esquina BR → sureste (reversa)
    (1.0, 4.0,   math.pi),           # robot3 — borde izq  → oeste (reversa exacta)
    (8.0, 4.0,   0.0),               # robot4 — borde der  → este  (reversa exacta)
    (0.0, 8.0,   3*math.pi / 4),     # robot5 — esquina TL → noroeste (reversa)
    (4.0, 9.0,   math.pi / 2),       # robot6 — borde sup  → norte (reversa)
    (8.0, 8.0,  -3*math.pi / 4),     # robot7 — esquina TR → hacia centro (único hacia adelante)
]


def launch_setup(context, *args, **kwargs):

    n              = int(LaunchConfiguration('n_robots').perform(context))
    R              = float(LaunchConfiguration('neighbor_radius').perform(context))
    t_final        = float(LaunchConfiguration('t_final').perform(context))
    spawn_obstacle = LaunchConfiguration('spawn_obstacle').perform(context).lower() == 'true'
    obs_x          = float(LaunchConfiguration('obstacle_x').perform(context))
    obs_y          = float(LaunchConfiguration('obstacle_y').perform(context))

    poses = POSES_8[:n]

    simulador_pkg  = get_package_share_directory('simulador')
    obstacle_sdf   = os.path.join(simulador_pkg, 'models', 'obstacle_cylinder.sdf')

    articubot_pkg = get_package_share_directory('articubot_one')
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )
    rsp_launch = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')

    actions = []

    actions.append(
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch))
    )

    if spawn_obstacle:
        actions.append(TimerAction(period=3.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', obstacle_sdf,
                    '-entity', 'obstacle_cylinder',
                    '-x', str(obs_x), '-y', str(obs_y), '-z', '0.5',
                ],
                output='screen'
            )
        ]))

    # Stagger each robot's RSP + spawn + control node to avoid overloading Gazebo
    for i, (x0, y0, yaw) in enumerate(poses):

        ns = f'robot{i}'
        delay = 3.0 + i * 1.5   # robot0 at 3s, robot1 at 4.5s, ..., robot7 at 13.5s

        rsp_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch),
            launch_arguments={
                'namespace': ns,
                'use_sim_time': 'false'
            }.items()
        )

        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', f'/{ns}/robot_description',
                '-entity', ns,
                '-x', str(round(x0, 4)),
                '-y', str(round(y0, 4)),
                '-z', '0.1',
                '-Y', str(round(yaw, 4)),
            ],
            output='screen'
        )

        control_node = Node(
            package='control_nodes',
            executable='consenso_node',
            name='consenso',
            namespace=ns,
            parameters=[{
                'consensus_type': 'prom_err',
                'k1':             0.8,
                'k2':             1.0,
                'vmax':           1.0,
                'wmax':           1.0,
            }],
            output='screen'
        )

        actions.append(TimerAction(period=delay, actions=[rsp_node]))
        actions.append(TimerAction(period=delay + 0.5, actions=[spawn_node]))
        actions.append(TimerAction(period=delay + 1.5, actions=[control_node]))

    # AIRE and plotters after all robots are launched
    late_delay = 3.0 + n * 1.5 + 2.0

    actions.append(
        TimerAction(period=late_delay, actions=[
            Node(
                package='simulador',
                executable='aire',
                name='aire',
                parameters=[{'neighbor_radius': R}],
                output='screen'
            )
        ])
    )

    actions.append(
        TimerAction(period=late_delay, actions=[
            Node(
                package='simulador',
                executable='states_plotter_v2',
                name='states_plotter_v2',
                parameters=[{
                    'save_dir':          'figuras/consenso_prom',
                    't_final':           t_final,
                    'n_robots_expected': n,
                }],
                output='screen'
            )
        ])
    )

    actions.append(
        TimerAction(period=late_delay, actions=[
            Node(
                package='simulador',
                executable='pva_plotter',
                name='pva_plotter',
                parameters=[{
                    'save_dir': 'figuras/consenso_prom',
                    't_final':  t_final,
                }],
                output='screen'
            )
        ])
    )

    return actions


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'n_robots',
            default_value='8',
            description='Número de robots (máx 8)'
        ),
        DeclareLaunchArgument(
            'neighbor_radius',
            default_value='5.0',
            description='Radio de percepción AIRE (metros)'
        ),
        DeclareLaunchArgument(
            't_final',
            default_value='60.0',
            description='Duración de la simulación en segundos'
        ),
        DeclareLaunchArgument(
            'spawn_obstacle',
            default_value='false',
            description='Poner true para agregar un cilindro de prueba'
        ),
        DeclareLaunchArgument(
            'obstacle_x',
            default_value='4.0',
            description='Posición X del obstáculo de prueba'
        ),
        DeclareLaunchArgument(
            'obstacle_y',
            default_value='4.5',
            description='Posición Y del obstáculo de prueba'
        ),
        OpaqueFunction(function=launch_setup)
    ])
