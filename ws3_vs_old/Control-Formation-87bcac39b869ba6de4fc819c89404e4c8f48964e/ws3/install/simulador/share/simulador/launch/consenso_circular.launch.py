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


def compute_initial_positions(n, leader_id, angle_v, d):
    """
    Posiciones iniciales en formación V con theta_leader=0 (líder mirando al este).
    Líder en (0, 0). Seguidores en sus anclas virtuales: r_i = R(θ_L ± angle_v) * [-k*d, 0].
    """

    n_followers = n - 1
    n_left  = n_followers // 2
    n_right = n_followers - n_left

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

    for k in range(1, n_right + 1):
        rx, ry = rot(-angle_v, -k * d, 0.0)
        positions[idx] = (round(rx, 4), round(ry, 4))
        idx += 1

    return positions


def _circle_waypoints(cx, cy, radius, n_wp, n_circles, start_angle=0):
    """
    Genera waypoints para n_circles vueltas en círculo CCW.
    start_angle determina el ángulo inicial (rad).
    Con start_angle=-π/2 arranca desde abajo.
    Retorna lista plana [x0, y0, x1, y1, ...].
    """
    pts = []
    total = n_wp * n_circles + 1  # +1 para cerrar el último arco
    for i in range(total):
        angle = start_angle + 2 * math.pi * i / n_wp
        pts += [round(cx + radius * math.cos(angle), 4),
                round(cy + radius * math.sin(angle), 4)]
    return pts


def generate_circular_waypoints():
    """
    Círculo centrado en (0, RADIUS) = (0, 4), radio 4.0.
    El líder arranca desde ABAJO (ángulo -π/2), justo en (0,0).
    Gira en sentido ANTIHORARIO: abajo → derecha → arriba → izquierda → abajo.
    Da 2 vueltas completas con 48 waypoints por vuelta.
    """
    RADIUS      = 4.0
    N_WP        = 48
    N_CIRCLES   = 2
    CX, CY      = 0.0, RADIUS

    return _circle_waypoints(CX, CY, RADIUS, N_WP, N_CIRCLES,
                              start_angle=-math.pi / 2)


def launch_setup(context, *args, **kwargs):

    n             = int(LaunchConfiguration('n_robots').perform(context))
    R             = float(LaunchConfiguration('neighbor_radius').perform(context))
    leader_id     = LaunchConfiguration('leader_id').perform(context)
    angle_v       = float(LaunchConfiguration('angle_v').perform(context))
    d             = float(LaunchConfiguration('d').perform(context))
    # Si se pasan waypoints personalizados, usarlos; si no, generar círculo
    waypoints_str = LaunchConfiguration('waypoints').perform(context)
    t_final       = float(LaunchConfiguration('t_final').perform(context))
    save_dir      = LaunchConfiguration('save_dir').perform(context)

    # Generar waypoints: si el default está vacío, generar círculo
    if waypoints_str and waypoints_str != '__default__':
        waypoints = [float(v) for v in waypoints_str.split(',')]
    else:
        waypoints = generate_circular_waypoints()

    init_pos = compute_initial_positions(n, leader_id, angle_v, d)

    articubot_pkg = get_package_share_directory('articubot_one')
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )
    rsp_launch = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')

    actions = []

    actions.append(
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch))
    )

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
                '-Y', '0.0',  # todos miran al este (heading = 0)
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
                    'vmax': 0.5,
                    'wmax': 0.6,
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
                    'consensus_type': 'formacion_vs',
                    'leader_id':      leader_id,
                    'n_robots':       n,
                    'angle_v':        angle_v,
                    'd':              d,
                    'beta':           2.0,
                    'k1':             1.0,
                    'k2':             1.5,
                    'vmax':           0.8,
                    'wmax':           0.8,
                    'anchor_speed':   0.5,
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
                't_final':           t_final,
                'n_robots_expected': n,
            }],
            output='screen'
        )
    ]))

    return actions


def generate_launch_description():

    # Generar los waypoints del círculo por defecto como string
    circle_wps = generate_circular_waypoints()
    default_wps = ','.join(str(v) for v in circle_wps)

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
            default_value=default_wps,
            description='Waypoints del líder (default: círculo 2 vueltas)'),
        DeclareLaunchArgument('t_final',
            default_value='160.0',
            description='Duración de la simulación (s)'),
        DeclareLaunchArgument('save_dir',
            default_value='figuras/circular',
            description='Directorio donde se guardan las figuras'),
        OpaqueFunction(function=launch_setup)
    ])
