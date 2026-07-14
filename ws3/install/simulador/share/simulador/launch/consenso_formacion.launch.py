import os
import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

# Posición de spawn del líder — inicio y cierre de la trayectoria por defecto
LEADER_X = 0.0
LEADER_Y = 0.0


def compute_initial_positions(n, leader_id, angle_v, d):
    """
    Posiciones iniciales en formación V con theta_leader=0 (líder mirando al este).
    Líder en (LEADER_X, LEADER_Y). Seguidores en sus anclas virtuales:
    r_i = R(θ_L ± angle_v) * [-k*d, 0].
    """

    n_followers = n - 1
    n_left  = n_followers // 2
    n_right = n_followers - n_left

    def rot(theta, vx, vy):
        c, s = math.cos(theta), math.sin(theta)
        return c * vx - s * vy, s * vx + c * vy

    leader_num = int(leader_id.replace('robot', ''))
    positions  = {leader_num: (LEADER_X, LEADER_Y)}

    idx = 1
    for k in range(1, n_left + 1):
        rx, ry = rot(angle_v, -k * d, 0.0)
        positions[idx] = (round(LEADER_X + rx, 4), round(LEADER_Y + ry, 4))
        idx += 1

    for k in range(1, n_right + 1):
        rx, ry = rot(-angle_v, -k * d, 0.0)
        positions[idx] = (round(LEADER_X + rx, 4), round(LEADER_Y + ry, 4))
        idx += 1

    return positions


def launch_setup(context, *args, **kwargs):

    n             = int(LaunchConfiguration('n_robots').perform(context))
    R             = float(LaunchConfiguration('neighbor_radius').perform(context))
    leader_id     = LaunchConfiguration('leader_id').perform(context)
    angle_v       = float(LaunchConfiguration('angle_v').perform(context))
    d             = float(LaunchConfiguration('d').perform(context))
    waypoints_str = LaunchConfiguration('waypoints').perform(context)
    t_final       = float(LaunchConfiguration('t_final').perform(context))
    save_dir      = LaunchConfiguration('save_dir').perform(context)
    record_gif    = LaunchConfiguration('record_gif').perform(context).lower() == 'true'
    camera_height = float(LaunchConfiguration('camera_height').perform(context))
    gif_fps       = float(LaunchConfiguration('gif_fps').perform(context))
    waypoints     = [float(v) for v in waypoints_str.split(',')]

    init_pos = compute_initial_positions(n, leader_id, angle_v, d)

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

    # Encuadre de cámara: bounding box de las posiciones de spawn + la ruta del líder
    wp_pairs = [(waypoints[i], waypoints[i + 1]) for i in range(0, len(waypoints), 2)]
    all_x = [p[0] for p in init_pos.values()] + [p[0] for p in wp_pairs]
    all_y = [p[1] for p in init_pos.values()] + [p[1] for p in wp_pairs]
    camera_x = (max(all_x) + min(all_x)) / 2.0
    camera_y = (max(all_y) + min(all_y)) / 2.0

    actions = []

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world_path}.items()
        )
    )

    if record_gif:
        actions.append(TimerAction(period=2.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', camera_sdf,
                    '-entity', 'overhead_camera',
                    '-x', str(camera_x), '-y', str(camera_y), '-z', str(camera_height),
                    '-P', '1.5708',
                ],
                output='screen'
            )
        ]))

    # Staggered: robot i arranca en t = 3 + i*1.5 s
    for i in range(n):
        ns    = f'robot{i}'
        x0, y0 = init_pos.get(i, (0.0, i * 1.5))
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
                '-x', str(x0), '-y', str(y0), '-z', '0.1',
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
                    'consensus_type': 'formacion',
                    'leader_id':      leader_id,
                    'n_robots':       n,
                    'angle_v':        angle_v,
                    'd':              d,
                    'beta':           2.0,
                    'k1':             1.0,
                    'k2':             1.5,
                    'vmax':           0.8,
                    'wmax':           0.8,
                    # PVA solo como saturación (sin evasión por LiDAR),
                    # como en el nodo de formación original
                    'd_safe':         0.0,
                    'd_influence':    0.0,
                }],
                output='screen'
            )

        actions.append(TimerAction(period=delay,       actions=[rsp_node]))
        actions.append(TimerAction(period=delay + 0.5, actions=[spawn_node]))
        actions.append(TimerAction(period=delay + 1.5, actions=[control_node]))

    # AIRE y plotter después de que todos los robots estén activos
    late_delay = 3.0 + n * 1.5 + 2.0

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
                't_final':           t_final,   # respaldo de seguridad
                'n_robots_expected': n,
                # Líder físico real (navigate_waypoints_pva) + offset de V:
                # el ideal de cada robot es la pose del líder más su offset,
                # misma fórmula que usa consenso_node (ConsensusFormation).
                'error_reference':      'leader_robot',
                'formation_leader_id':  leader_id,
                'formation_angle_v':    angle_v,
                'formation_d':          d,
                # Guarda al completar el loop (antes de que gif_recorder mate
                # este proceso con su auto_shutdown/pkill al terminar antes).
                'stop_mode':    'goal',
                'goal_topic':   f'/{leader_id}/final_goal_reached',
                'stop_delay':   1.0,
            }],
            output='screen'
        )
    ]))

    if record_gif:
        actions.append(TimerAction(period=late_delay, actions=[
            Node(
                package='simulador',
                executable='gif_recorder',
                name='gif_recorder',
                parameters=[{
                    'save_dir':      save_dir,
                    't_final':       t_final,   # respaldo de seguridad
                    'fps':           gif_fps,
                    'gif_name':      'formacion.gif',
                    # Cierra al completar el loop (navigate_waypoints_pva
                    # publica esto al llegar al último waypoint), no por
                    # tiempo fijo — mismo criterio que vs_formacion*.
                    'stop_mode':     'goal',
                    'goal_topic':    f'/{leader_id}/final_goal_reached',
                    'stop_delay':    2.0,
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
            # Loop simple (cuadrado 5x5 m) que cierra exactamente en el spawn
            # del líder (LEADER_X, LEADER_Y) = (0.0, 0.0) — mismo criterio de
            # inicio/cierre que vs_formacion / vs_formacion_noble.
            default_value='0.0,0.0, 5.0,0.0, 5.0,5.0, 0.0,5.0, 0.0,0.0',
            description='Waypoints del líder: x0,y0,x1,y1,... (navegación punto a punto, no spline)'),
        DeclareLaunchArgument('t_final',
            default_value='120.0',
            description='Duración de la simulación (s)'),
        DeclareLaunchArgument('save_dir',
            default_value='figuras/formacion',
            description='Directorio donde se guardan las figuras'),
        DeclareLaunchArgument('record_gif',
            default_value='true',
            description='Si es true, spawnea la cámara cenital y graba un GIF de la simulación'),
        DeclareLaunchArgument('camera_height',
            default_value='9.0',
            description='Altura de la cámara cenital (m) — estándar del proyecto'),
        DeclareLaunchArgument('gif_fps',
            default_value='10.0',
            description='Fotogramas por segundo del GIF grabado'),
        OpaqueFunction(function=launch_setup)
    ])
