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


# ------------------------------------------------------------------
# Variante de pivot_giro_waypoints_v2.launch.py CON robots físicos:
# además de las esferas de referencia (estructura virtual que alcanza
# waypoints girando y avanzando A LA PAR, ver pivot_giro.py
# PivotGiroWaypointsV2 / pivot_giro_waypoints_v2_node.py), se despliegan
# N_ROBOTS robots reales en Gazebo que siguen esas referencias vía
# consenso_node (consensus_type='formacion'), igual patrón que
# vs_formacion_noble.launch.py pero con pivot_giro_waypoints_v2 como
# generador de /virtual_structure en vez de vs_node (spline).
# ------------------------------------------------------------------

N_ROBOTS = 5
ANGLE_V  = math.pi / 4
D        = 1.0

THETA_0  = math.pi / 2.0   # heading inicial: norte

# Centroide inicial de la estructura virtual
VS_X = 0.0
VS_Y = 0.0

OMEGA = 0.5    # rad/s, para el giro de orientación (V2: corre en
               # paralelo con el avance, no lo bloquea, por eso puede
               # ser mayor que en la V1 secuencial)
V_MAX = 0.25   # m/s, para el avance en línea recta

# Waypoints a alcanzar, uno por uno (rombos dorados en Gazebo)
WAYPOINTS = [3.0, 3.0,   3.0, -3.0,   -3.0, -3.0]

# Robots spawnean 2.5 m detrás de la estructura virtual (mismo criterio
# que vs_formacion_noble.launch.py)
# round() evita residuos de punto flotante tipo -1.53e-16 (cos(pi/2) no
# da exactamente 0.0) que spawn_entity.py's argparse no sabe parsear
# como argumento de -x/-y (confunde el '-' del signo con otro flag).
ROBOT_OFFSET = 2.5
ROBOT_X = round(VS_X - math.cos(THETA_0) * ROBOT_OFFSET, 6) + 0.0
ROBOT_Y = round(VS_Y - math.sin(THETA_0) * ROBOT_OFFSET, 6) + 0.0

# Timing
T_START      = 3.0
T_ROBOT_STEP = 1.5
T_AIRE       = T_START + N_ROBOTS * T_ROBOT_STEP + 1.0
T_CONTROL    = T_AIRE + 1.0
START_DELAY  = T_CONTROL - T_START + 2.0   # la VS espera a que los robots ya tengan control activo

RECORD_GIF    = True
GIF_FPS       = 10.0
STOP_DELAY    = 2.0

# Margen generoso: giro completo (pi) + tramo recto por cada waypoint
_WP_PTS     = [(WAYPOINTS[i], WAYPOINTS[i+1]) for i in range(0, len(WAYPOINTS), 2)]
_dist_total = 0.0
_prev = (VS_X, VS_Y)
for _wp in _WP_PTS:
    _dist_total += math.hypot(_wp[0]-_prev[0], _wp[1]-_prev[1])
    _prev = _wp
T_FINAL = START_DELAY + len(_WP_PTS) * (math.pi / OMEGA) + _dist_total / V_MAX + 30.0

# Encuadre de cámara: bounding box de waypoints + posición inicial de la VS
_ALL_X = [VS_X] + [p[0] for p in _WP_PTS]
_ALL_Y = [VS_Y] + [p[1] for p in _WP_PTS]
CAMERA_X      = (max(_ALL_X) + min(_ALL_X)) / 2.0
CAMERA_Y      = (max(_ALL_Y) + min(_ALL_Y)) / 2.0
CAMERA_HEIGHT = 9.0


def _v_positions(n, cx, cy, theta, angle_v, d):
    n_followers = n - 1
    n_left  = n_followers // 2
    n_right = n_followers - n_left

    def rot(th, vx, vy):
        c, s = math.cos(th), math.sin(th)
        return c*vx - s*vy, s*vx + c*vy

    pos = {0: (cx, cy)}
    for k in range(1, n_left + 1):
        ox, oy = rot(theta + angle_v, -k*d, 0.0)
        pos[k] = (round(cx + ox, 4), round(cy + oy, 4))
    for k in range(1, n_right + 1):
        ox, oy = rot(theta - angle_v, -k*d, 0.0)
        pos[n_left + k] = (round(cx + ox, 4), round(cy + oy, 4))
    return pos


def _v_positions_flat(n, cx, cy, theta, angle_v, d):
    pos = _v_positions(n, cx, cy, theta, angle_v, d)
    flat = []
    for i in range(n):
        flat.extend(pos[i])
    return flat


INITIAL_POSITIONS = _v_positions_flat(N_ROBOTS, VS_X, VS_Y, THETA_0, ANGLE_V, D)


def launch_setup(context, *args, **kwargs):

    save_dir = LaunchConfiguration('save_dir').perform(context)

    articubot_pkg = get_package_share_directory('articubot_one')
    rsp_launch    = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')
    camera_sdf    = os.path.join(
        get_package_share_directory('simulador'), 'models', 'overhead_camera.sdf'
    )

    robot_pos = _v_positions(N_ROBOTS, ROBOT_X, ROBOT_Y, THETA_0, ANGLE_V, D)

    actions = []

    if RECORD_GIF:
        actions.append(TimerAction(period=T_START, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', camera_sdf,
                    '-entity', 'overhead_camera',
                    '-x', str(CAMERA_X), '-y', str(CAMERA_Y), '-z', str(CAMERA_HEIGHT),
                    '-P', '1.5708',
                ],
                output='screen'
            )
        ]))

        actions.append(TimerAction(period=T_AIRE, actions=[
            Node(
                package='simulador',
                executable='gif_recorder',
                name='gif_recorder',
                parameters=[{
                    'save_dir':      save_dir,
                    'gif_name':      'pivot_giro_waypoints_v2_formacion.gif',
                    'fps':           GIF_FPS,
                    'stop_mode':     'goal',
                    'goal_topic':    '/final_goal_reached',
                    'stop_delay':    STOP_DELAY,
                    't_final':       300.0,
                    'auto_shutdown': True,
                }],
                output='screen'
            )
        ]))

    # --- Spawn staggered de cada robot ---
    for i in range(N_ROBOTS):
        ns     = f'robot{i}'
        x0, y0 = robot_pos[i]
        t_rsp  = T_START + i * T_ROBOT_STEP

        actions.append(TimerAction(period=t_rsp, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rsp_launch),
                launch_arguments={'namespace': ns, 'use_sim_time': 'false'}.items()
            )
        ]))

        actions.append(TimerAction(period=t_rsp + 0.5, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', f'/{ns}/robot_description',
                    '-entity', ns,
                    '-x', str(x0), '-y', str(y0), '-z', '0.1',
                    '-Y', str(THETA_0),
                ],
                output='screen'
            )
        ]))

    # --- AIRE ---
    actions.append(TimerAction(period=T_AIRE, actions=[
        Node(
            package='simulador',
            executable='aire',
            name='aire',
            parameters=[{'neighbor_radius': 15.0}],
            output='screen'
        )
    ]))

    # --- Gráficas: trayectoria + error de formación vs offset ideal ---
    actions.append(TimerAction(period=T_AIRE, actions=[
        Node(
            package='simulador',
            executable='states_plotter_v2',
            name='states_plotter_v2',
            parameters=[{
                'save_dir':                   save_dir,
                't_final':                    T_FINAL,
                'n_robots_expected':          N_ROBOTS,
                'error_vs_virtual_structure': True,
                'formation_angle_v':          ANGLE_V,
                'formation_d':                D,
                'stop_mode':    'goal',
                'goal_topic':   '/final_goal_reached',
                'stop_delay':   1.0,
            }],
            output='screen'
        )
    ]))

    # --- Nodo de consenso para cada robot (todos siguen la estructura virtual) ---
    for i in range(N_ROBOTS):
        ns = f'robot{i}'
        actions.append(TimerAction(period=T_CONTROL, actions=[
            Node(
                package='control_nodes',
                executable='consenso_node',
                name='consenso',
                namespace=ns,
                parameters=[{
                    'consensus_type': 'formacion',
                    'n_robots':       N_ROBOTS,
                    'angle_v':        ANGLE_V,
                    'd':              D,
                    'beta':           2.0,
                    'k1':             1.0,
                    'k2':             1.5,
                    'vmax':           0.5,
                    'wmax':           0.6,
                    'd_safe':         0.35,
                    'd_influence':    0.60,
                }],
                output='screen'
            )
        ]))

    return actions


def generate_launch_description():

    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )
    world_path = os.path.join(
        get_package_share_directory('simulador'), 'worlds', 'demo.world'
    )

    return LaunchDescription([

        DeclareLaunchArgument('save_dir',
            default_value='figuras/pivot_giro_waypoints_formacion',
            description='Directorio donde se guardan gifs/ y frames/'),

        # --- Gazebo con plugin de estado ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world_path}.items()
        ),

        # --- Estructura virtual: esferas + PivotGiroWaypointsV2 + waypoints dorados ---
        TimerAction(period=T_START, actions=[
            Node(
                package='control_nodes',
                executable='pivot_giro_waypoints_v2_node',
                name='pivot_giro_waypoints_v2_node',
                parameters=[{
                    'n_robots':           N_ROBOTS,
                    'initial_positions':  INITIAL_POSITIONS,
                    'waypoints':          WAYPOINTS,
                    'omega':              OMEGA,
                    'v_max':              V_MAX,
                    'theta_0':            THETA_0,
                    'start_delay':        START_DELAY,
                }],
                output='screen',
            )
        ]),

        # --- Robots ---
        OpaqueFunction(function=launch_setup),

    ])
