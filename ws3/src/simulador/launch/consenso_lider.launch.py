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

# Grabación de GIF (cámara cenital) — se detiene sola al llegar a t_final
# (ConsensusLeader no tiene señal de "meta alcanzada" propia).
RECORD_GIF    = True
CAMERA_HEIGHT = 9.0   # estándar del proyecto — "más cerca, robots se ven más grandes"
GIF_FPS       = 10.0


def launch_setup(context, *args, **kwargs):

    n             = int(LaunchConfiguration('n_robots').perform(context))
    R             = float(LaunchConfiguration('neighbor_radius').perform(context))
    leader_id     = LaunchConfiguration('leader_id').perform(context)
    angle_v       = float(LaunchConfiguration('angle_v').perform(context))
    d             = float(LaunchConfiguration('d').perform(context))
    waypoints_str = LaunchConfiguration('waypoints').perform(context)
    t_final       = float(LaunchConfiguration('t_final').perform(context))
    save_dir      = LaunchConfiguration('save_dir').perform(context)
    waypoints     = [float(v) for v in waypoints_str.split(',')]

    articubot_pkg = get_package_share_directory('articubot_one')
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )
    rsp_launch = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')
    world_path = os.path.join(                                  # mundo con piso transparente
        get_package_share_directory('simulador'), 'worlds', 'demo.world'
    )
    camera_sdf = os.path.join(
        get_package_share_directory('simulador'), 'models', 'overhead_camera.sdf'
    )

    # Escenario común a todas las experimentaciones (consenso_lider,
    # consenso_formacion, vs_formacion): líder/robot0 en el origen mirando
    # al este (theta=0), seguidores ya en su ancla dentro de la V
    # (angle_v, d), en vez de una grilla arbitraria — así el punto de
    # partida es idéntico entre experimentos y las gráficas son comparables.
    n_followers = n - 1
    n_left      = n_followers // 2

    def rot(theta, vx, vy):
        c, s = math.cos(theta), math.sin(theta)
        return c * vx - s * vy, s * vx + c * vy

    leader_num = int(leader_id.replace('robot', ''))
    positions  = {leader_num: (0.0, 0.0)}
    idx = 1
    for k in range(1, n_left + 1):
        rx, ry = rot(angle_v, -k * d, 0.0)
        positions[idx] = (round(rx, 4), round(ry, 4))
        idx += 1
    for k in range(1, n_followers - n_left + 1):
        rx, ry = rot(-angle_v, -k * d, 0.0)
        positions[idx] = (round(rx, 4), round(ry, 4))
        idx += 1

    # Encuadre de cámara: bounding box de las posiciones de spawn + la ruta del líder
    wp_pairs = [(waypoints[i], waypoints[i + 1]) for i in range(0, len(waypoints), 2)]
    all_x = [p[0] for p in positions.values()] + [p[0] for p in wp_pairs]
    all_y = [p[1] for p in positions.values()] + [p[1] for p in wp_pairs]
    camera_x = (max(all_x) + min(all_x)) / 2.0
    camera_y = (max(all_y) + min(all_y)) / 2.0

    late_delay = 3.0 + n * 1.5 + 2.0

    actions = []

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world_path}.items()
        )
    )

    if RECORD_GIF:
        actions.append(TimerAction(period=2.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', camera_sdf,
                    '-entity', 'overhead_camera',
                    '-x', str(camera_x), '-y', str(camera_y), '-z', str(CAMERA_HEIGHT),
                    '-P', '1.5708',
                ],
                output='screen'
            )
        ]))

    # Staggered: robot i spawns at t = 3 + i*1.5 s
    for i in range(n):
        ns    = f'robot{i}'
        x0, y0 = positions[i]
        delay = 3.0 + i * 1.5

        rsp_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch),
            launch_arguments={'namespace': ns, 'use_sim_time': 'false'}.items()
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
                '-Y', '0.0',
            ],
            output='screen'
        )

        if ns == leader_id:
            control_node = Node(
                package='control_nodes',
                executable='navigate_waypoints_pva',
                name='navigate_waypoints',
                namespace=ns,
                parameters=[{
                    'waypoints': waypoints,
                    'vmax': 0.25,
                    'wmax': 0.5,
                }],
                output='screen'
            )
        else:
            control_node = Node(
                package='control_nodes',
                executable='consenso_node',
                name='consenso',
                namespace=ns,
                parameters=[{
                    'consensus_type': 'lider',
                    'leader_id':      leader_id,
                    'k_leader':       2.0,
                    'k1':             1.0,
                    'k2':             1.5,
                    'vmax':           0.8,
                    'wmax':           0.8,
                }],
                output='screen'
            )

        actions.append(TimerAction(period=delay,       actions=[rsp_node]))
        actions.append(TimerAction(period=delay + 0.5, actions=[spawn_node]))

        # El líder espera a que todos los robots hayan spawneado antes de moverse
        control_delay = late_delay if ns == leader_id else delay + 1.5
        actions.append(TimerAction(period=control_delay, actions=[control_node]))

    # AIRE y plotters después de que todos los robots estén activos
    actions.append(TimerAction(period=late_delay, actions=[
        Node(
            package='simulador',
            executable='aire',
            name='aire',
            parameters=[{'neighbor_radius': R}],
            output='screen'
        )
    ]))

    actions.append(TimerAction(period=late_delay, actions=[
        Node(
            package='simulador',
            executable='states_plotter_v2',
            name='states_plotter_v2',
            parameters=[{
                'save_dir':          save_dir,
                't_final':           t_final,
                'n_robots_expected': n,
                # ConsensusLeader no tiene offset de formación: todos los
                # seguidores convergen exactamente a la posición del líder.
                # formation_d=0.0 colapsa el offset ideal a (0,0) para todos.
                'error_reference':   'leader_robot',
                'formation_leader_id': leader_id,
                'formation_d':        0.0,
            }],
            output='screen'
        )
    ]))

    if RECORD_GIF:
        actions.append(TimerAction(period=late_delay, actions=[
            Node(
                package='simulador',
                executable='gif_recorder',
                name='gif_recorder',
                parameters=[{
                    'save_dir':      save_dir,
                    'gif_name':      'consenso_lider.gif',
                    'fps':           GIF_FPS,
                    't_final':       t_final,
                    'auto_shutdown': True,
                }],
                output='screen'
            )
        ]))

    # actions.append(TimerAction(period=late_delay, actions=[
    #     Node(
    #         package='simulador',
    #         executable='pva_plotter',
    #         name='pva_plotter',
    #         parameters=[{
    #             'save_dir': save_dir,
    #             't_final':  t_final,
    #         }],
    #         output='screen'
    #     )
    # ]))

    return actions


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('n_robots',
            default_value='5',
            description='Total de robots (líder + seguidores)'),
        DeclareLaunchArgument('neighbor_radius',
            default_value='10.0',
            description='Radio de percepción AIRE (m)'),
        DeclareLaunchArgument('leader_id',
            default_value='robot0',
            description='Namespace del robot líder'),
        DeclareLaunchArgument('angle_v',
            default_value='0.7854',
            description='Ángulo de apertura de la V (rad), default π/4 = 45°'),
        DeclareLaunchArgument('d',
            default_value='1.0',
            description='Separación entre robots en el brazo de la V (m)'),
        DeclareLaunchArgument('waypoints',
            # Mismo cuadrado 5x5m que consenso_formacion.launch.py y
            # vs_formacion.launch.py: escenario común para las cuatro
            # experimentaciones.
            default_value='0.0,0.0,5.0,0.0,5.0,5.0,0.0,5.0,0.0,0.0',
            description='Waypoints del líder: x0,y0,x1,y1,...'),
        DeclareLaunchArgument('t_final',
            default_value='90.0',
            description='Duración de la simulación (s)'),
        DeclareLaunchArgument('save_dir',
            default_value='figuras/consenso_lider',
            description='Directorio donde se guardan las figuras'),
        OpaqueFunction(function=launch_setup)
    ])
