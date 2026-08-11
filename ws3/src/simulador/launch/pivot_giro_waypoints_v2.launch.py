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
# Variante V2 de pivot_giro_waypoints.launch.py: en vez de girar
# primero (orientarse hacia el waypoint) y avanzar después en dos
# etapas secuenciales, el giro (control proporcional del error angular,
# pivote relativo que se traslada junto con la formación) y el avance
# en línea recta (v_max constante) ocurren A LA PAR — la formación
# nunca se queda quieta girando, avanza mientras se orienta.
# Algoritmo: control_nodes/algorithms/pivot_giro.py (PivotGiroWaypointsV2)
# Nodo: pivot_giro_waypoints_v2_node.py
# ------------------------------------------------------------------

N_ROBOTS = 5
ANGLE_V  = math.pi / 4
D        = 1.0

THETA_0  = math.pi / 2.0   # heading inicial: norte

VS_X = 0.0
VS_Y = 0.0

OMEGA = 0.5    # rad/s, para el giro de orientación (más rápido que V1
               # porque ahora corre en paralelo con el avance, no lo
               # bloquea)
V_MAX = 0.25   # m/s, para el avance en línea recta

# Waypoints a alcanzar, uno por uno (rombos dorados en Gazebo)
WAYPOINTS = [3.0, 3.0,   3.0, -3.0,   -3.0, -3.0]

T_START = 3.0
T_AIRE  = T_START + 1.0

RECORD_GIF    = True
GIF_FPS       = 10.0
STOP_DELAY    = 2.0

# Margen generoso: al correr giro y avance en paralelo, el tiempo real
# es menor que la suma secuencial — se usa la misma estimación
# conservadora (cota superior) que V1.
_WP_PTS   = [(WAYPOINTS[i], WAYPOINTS[i+1]) for i in range(0, len(WAYPOINTS), 2)]
_dist_total = 0.0
_prev = (VS_X, VS_Y)
for _wp in _WP_PTS:
    _dist_total += math.hypot(_wp[0]-_prev[0], _wp[1]-_prev[1])
    _prev = _wp
T_FINAL = len(_WP_PTS) * (math.pi / OMEGA) + _dist_total / V_MAX + 30.0

CAMERA_X      = VS_X
CAMERA_Y      = VS_Y
CAMERA_HEIGHT = 9.0


def _v_positions_flat(n, cx, cy, theta, angle_v, d):
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

    flat = []
    for i in range(n):
        flat.extend(pos[i])
    return flat


INITIAL_POSITIONS = _v_positions_flat(N_ROBOTS, VS_X, VS_Y, THETA_0, ANGLE_V, D)


def launch_setup(context, *args, **kwargs):

    save_dir   = LaunchConfiguration('save_dir').perform(context)
    camera_sdf = os.path.join(
        get_package_share_directory('simulador'), 'models', 'overhead_camera.sdf'
    )

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
                    'gif_name':      'pivot_giro_waypoints_v2.gif',
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

    actions.append(TimerAction(period=T_AIRE, actions=[
        Node(
            package='simulador',
            executable='states_plotter_v2',
            name='states_plotter_v2',
            parameters=[{
                'save_dir':           save_dir,
                't_final':            T_FINAL,
                'n_robots_expected':  0,
                'trajectory_only':    True,
                'stop_mode':    'goal',
                'goal_topic':   '/final_goal_reached',
                'stop_delay':   1.0,
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
            default_value='figuras/pivot_giro_waypoints_v2',
            description='Directorio donde se guardan gifs/ y frames/'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world_path}.items()
        ),

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
                    'start_delay':        3.0,
                }],
                output='screen',
            )
        ]),

        OpaqueFunction(function=launch_setup),

    ])
