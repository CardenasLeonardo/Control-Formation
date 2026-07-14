"""
circulo_pivote.launch.py

Experimento S04 en ROS2 / Gazebo:
  Círculo completo con pivote en el extremo del brazo interior.

El líder traza N_CIRCLES vueltas completas alrededor del robot extremo del
brazo izquierdo (robot n_left, índice n_left dentro de la formación), que
actúa como centro de giro de la estructura virtual.

Radios alrededor del pivote (parámetros por defecto, n=5, d=1.0, angle_v=π/4, CCW):
  líder       2.000 m   v = 0.500 m/s
  robot 1     1.000 m   v = 0.250 m/s
  robot 2     2.828 m   v = 0.707 m/s  (< vmax 0.8)  ← EXTERIOR
  robot 3     2.236 m   v = 0.559 m/s
  robot 4     0.000 m   ← PIVOTE, quasi-estático (brazo interior para CCW)

Seguidores: FormacionVSPPC  (estructura virtual + control de rendimiento prescrito)

Uso:
  ros2 launch simulador circulo_pivote.launch.py
  ros2 launch simulador circulo_pivote.launch.py n_circles:=1 rho_0:=3.0
  ros2 launch simulador circulo_pivote.launch.py turn_dir:=-1  # giro CW
"""

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


# ── Helpers geométricos ───────────────────────────────────────────────────────

def _compute_initial_positions(n, angle_v, d):
    """
    Posiciones iniciales en formación V, líder en (0,0) mirando al este.
    Retorna dict {robot_index: (x, y)}.
    """
    n_followers = n - 1
    n_left  = n_followers // 2
    n_right = n_followers - n_left

    def rot(theta, vx, vy):
        c, s = math.cos(theta), math.sin(theta)
        return c * vx - s * vy, s * vx + c * vy

    positions = {0: (0.0, 0.0)}
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


def _pivot_arc_waypoints(lx, ly, lth, n_left, n_right, d, angle_v,
                          turn_angle, n_wp, turn_dir=+1):
    """
    Waypoints del líder en un arco/círculo alrededor del robot extremo
    del brazo INTERIOR.

    turn_dir = +1 (CCW, giro izquierda): pivote en brazo derecho (−angle_v)
    turn_dir = −1 (CW,  giro derecha):   pivote en brazo izquierdo (+angle_v)

    Retorna lista plana [x0, y0, x1, y1, ...].
    """
    if turn_dir > 0:
        k_pivot   = n_right
        theta_arm = lth - angle_v
    else:
        k_pivot   = n_left
        theta_arm = lth + angle_v

    Px = lx - k_pivot * d * math.cos(theta_arm)
    Py = ly - k_pivot * d * math.sin(theta_arm)

    vx = lx - Px
    vy = ly - Py

    pts = []
    for i in range(n_wp):
        delta = turn_dir * turn_angle * i / (n_wp - 1)
        c, s  = math.cos(delta), math.sin(delta)
        rx    = vx * c - vy * s
        ry    = vx * s + vy * c
        pts  += [round(Px + rx, 4), round(Py + ry, 4)]
    return pts


def _generate_pivot_circle_waypoints(n_robots, angle_v, d, n_circles, turn_dir):
    """
    Calcula los waypoints del líder para N_CIRCLES vueltas alrededor del pivote
    (robot extremo del brazo INTERIOR según turn_dir).

    El número de waypoints es adaptativo: cuerda ≈ 0.25 m para que
    navigate_waypoints_pva (tolerance=0.15 m) no salte ninguno.
    """
    n_followers = n_robots - 1
    n_left  = n_followers // 2
    n_right = n_followers - n_left
    # El radio es siempre la distancia del líder al extremo del brazo interior
    k_pivot = n_right if turn_dir > 0 else n_left
    r_pivot = k_pivot * d

    chord_target  = 0.25
    circumference = 2 * math.pi * r_pivot
    n_wp_per_rev  = max(int(circumference / chord_target) + 1, 12)
    n_wp_total    = n_wp_per_rev * n_circles + 1

    turn_angle = 2 * math.pi * n_circles

    return _pivot_arc_waypoints(
        0.0, 0.0, 0.0,
        n_left, n_right, d, angle_v,
        turn_angle, n_wp_total, turn_dir
    )


# ── launch_setup ──────────────────────────────────────────────────────────────

def launch_setup(context, *args, **kwargs):

    n           = int(LaunchConfiguration('n_robots').perform(context))
    nbr_R       = float(LaunchConfiguration('neighbor_radius').perform(context))
    angle_v     = float(LaunchConfiguration('angle_v').perform(context))
    d           = float(LaunchConfiguration('d').perform(context))
    n_circles   = int(LaunchConfiguration('n_circles').perform(context))
    turn_dir    = int(LaunchConfiguration('turn_dir').perform(context))
    t_final     = float(LaunchConfiguration('t_final').perform(context))
    save_dir    = LaunchConfiguration('save_dir').perform(context)
    rho_0       = float(LaunchConfiguration('rho_0').perform(context))
    rho_inf     = float(LaunchConfiguration('rho_inf').perform(context))
    l_rate      = float(LaunchConfiguration('l_rate').perform(context))
    leader_id   = LaunchConfiguration('leader_id').perform(context)
    r_pivot_max = float(LaunchConfiguration('r_pivot_max').perform(context))
    v_leader    = float(LaunchConfiguration('v_leader').perform(context))

    waypoints = _generate_pivot_circle_waypoints(n, angle_v, d, n_circles, turn_dir)
    init_pos  = _compute_initial_positions(n, angle_v, d)

    n_followers = n - 1
    n_left      = n_followers // 2
    n_right     = n_followers - n_left
    # Índice del robot pivote (extremo del brazo interior)
    idx_pivot   = (n_left + n_right) if turn_dir > 0 else n_left  # robot 4 para CCW
    k_pivot     = n_right if turn_dir > 0 else n_left
    r_pivot     = k_pivot * d
    arc_len     = 2 * math.pi * r_pivot * n_circles

    # anchor_speed del pivote: r_pivot_max = anchor_speed × k_pivot × d / v_leader
    anchor_speed_pivot = r_pivot_max * v_leader / (k_pivot * d)

    # ── Paquetes ──────────────────────────────────────────────────────────────
    articubot_pkg = get_package_share_directory('articubot_one')
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )
    rsp_launch = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')

    actions = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch))
    ]

    # ── Robots escalonados ────────────────────────────────────────────────────
    for i in range(n):
        ns     = f'robot{i}'
        x0, y0 = init_pos.get(i, (0.0, i * 1.5))
        delay  = 3.0 + i * 1.5

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
                    'vmax': v_leader,
                    'wmax': 0.6,
                }],
                output='screen'
            )
        else:
            asp = anchor_speed_pivot if i == idx_pivot else 0.5
            control_node = Node(
                package='control_nodes',
                executable='consenso_node',
                name='consenso',
                namespace=ns,
                parameters=[{
                    'consensus_type': 'formacion_vs_ppc',
                    'leader_id':      leader_id,
                    'n_robots':       n,
                    'angle_v':        angle_v,
                    'd':              d,
                    'beta':           2.0,
                    'k1':             1.0,
                    'k2':             1.5,
                    'vmax':           0.8,
                    'wmax':           0.8,
                    'anchor_speed':   asp,
                    'rho_0':          rho_0,
                    'rho_inf':        rho_inf,
                    'l_rate':         l_rate,
                }],
                output='screen'
            )

        actions.append(TimerAction(period=delay,       actions=[rsp_node]))
        actions.append(TimerAction(period=delay + 0.5, actions=[spawn_node]))
        actions.append(TimerAction(period=delay + 1.5, actions=[control_node]))

    # ── AIRE y plotter ────────────────────────────────────────────────────────
    late_delay = 3.0 + n * 1.5 + 2.0

    actions.append(TimerAction(period=late_delay, actions=[
        Node(
            package='simulador',
            executable='aire',
            name='aire',
            parameters=[{'neighbor_radius': nbr_R}],
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


# ── generate_launch_description ───────────────────────────────────────────────

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
            description='Apertura de la V (rad), default π/4'),
        DeclareLaunchArgument('d',
            default_value='1.0',
            description='Separación entre robots en el brazo (m)'),
        DeclareLaunchArgument('n_circles',
            default_value='2',
            description='Número de vueltas completas'),
        DeclareLaunchArgument('turn_dir',
            default_value='1',
            description='Dirección del giro: +1=CCW (izquierda), -1=CW (derecha)'),
        DeclareLaunchArgument('rho_0',
            default_value='4.0',
            description='PPC: cota de error inicial (m)'),
        DeclareLaunchArgument('rho_inf',
            default_value='0.15',
            description='PPC: cota de error final (m)'),
        DeclareLaunchArgument('l_rate',
            default_value='0.04',
            description='PPC: tasa de convergencia de la envolvente'),
        DeclareLaunchArgument('v_leader',
            default_value='0.5',
            description='Velocidad máxima del líder (m/s)'),
        DeclareLaunchArgument('r_pivot_max',
            default_value='0.10',
            description='Desplazamiento máximo del pivote (m). '
                        'Fórmula: anchor_speed_pivot = r_pivot_max × v_leader / (k_pivot × d)'),
        DeclareLaunchArgument('t_final',
            default_value='90.0',
            description='Duración de la simulación (s)'),
        DeclareLaunchArgument('save_dir',
            default_value='figuras/circulo_pivote',
            description='Directorio donde se guardan las figuras'),
        OpaqueFunction(function=launch_setup)
    ])
