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
# Variante de trayectoria_vs.launch.py sin waypoints: la estructura
# virtual arranca en un set de posiciones iniciales fijo (V rígida) y
# ejecuta una secuencia de 3 fases como cuerpo rígido: gira -180° sobre
# la punta de un brazo, gira +180° sobre la punta del otro, y luego
# avanza en línea recta una distancia DISTANCE en la dirección hacia la
# que queda apuntando la formación (heading acumulado tras los giros).
# Preserva todo lo demás (Gazebo, cámara cenital, GIF, gráficas); solo
# cambia el módulo de navegación:
# control_nodes/algorithms/pivot_giro.py + pivot_giro_node.py.
# ------------------------------------------------------------------

N_ROBOTS = 5
ANGLE_V  = math.pi / 4
D        = 1.0

THETA_0  = math.pi / 2.0   # heading inicial: norte

# Centroide/posición inicial de la V (no se mueve como en trayectoria_vs;
# aquí gira sobre uno de sus brazos)
VS_X = 0.0
VS_Y = 0.0

# Secuencia fija: giro -180°, giro +180°, avanzar DISTANCE en línea recta
# en la dirección hacia la que queda apuntando la formación
OMEGA     = 0.3    # rad/s, para ambos giros
V_MAX     = 0.25   # m/s, para el avance final
DISTANCE  = 5.0    # m

T_START = 3.0
T_AIRE  = T_START + 1.0

RECORD_GIF    = True
GIF_FPS       = 10.0
STOP_DELAY    = 2.0

# Margen generoso sobre el tiempo estimado: dos giros de 180° + avance
_T_GIROS  = 2 * (math.pi / OMEGA)
_T_AVANCE = DISTANCE / V_MAX
T_FINAL   = _T_GIROS + _T_AVANCE + 30.0

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
                    'gif_name':      'pivot_giro.gif',
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
            default_value='figuras/pivot_giro',
            description='Directorio donde se guardan gifs/ y frames/'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world_path}.items()
        ),

        TimerAction(period=T_START, actions=[
            Node(
                package='control_nodes',
                executable='pivot_giro_node',
                name='pivot_giro_node',
                parameters=[{
                    'n_robots':           N_ROBOTS,
                    'initial_positions':  INITIAL_POSITIONS,
                    'omega':              OMEGA,
                    'v_max':              V_MAX,
                    'distance':           DISTANCE,
                    'theta_0':            THETA_0,
                    'start_delay':        3.0,
                }],
                output='screen',
            )
        ]),

        OpaqueFunction(function=launch_setup),

    ])
