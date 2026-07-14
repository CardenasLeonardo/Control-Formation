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
# Parámetros globales
# ------------------------------------------------------------------

N_ROBOTS = 5
ANGLE_V  = math.pi / 4
D        = 1.0

THETA_0  = math.pi / 2.0   # heading inicial: norte

# Centroide de la estructura virtual
VS_X = 3.0
VS_Y = 0.0

# Robots spawnean 2.5 m detrás de la estructura virtual
ROBOT_OFFSET = 2.5
ROBOT_X = VS_X - math.cos(THETA_0) * ROBOT_OFFSET   # 3.0
ROBOT_Y = VS_Y - math.sin(THETA_0) * ROBOT_OFFSET   # -2.5

# waypoints[0] es la posición inicial del centroide; la curva pasa por todos ellos.
# Ruta: cuadrado con esquinas redondeadas (Catmull-Rom suaviza cada esquina al
# tener dos puntos cercanos alrededor de ella; los tramos entre esquinas quedan
# prácticamente rectos). El cuadrado se arma de modo que:
#   - el borde derecho (x = SQ_RIGHT) pasa por (VS_X, VS_Y), el inicio real
#   - la esquina inferior derecha coincide con (ROBOT_X, ROBOT_Y), el spawn de
#     los robots, así el GIF cierra el ciclo exactamente donde arrancaron
SQ_RIGHT  = VS_X                 # 3.0
SQ_BOTTOM = ROBOT_Y              # -2.5  (== esquina inferior derecha)
SQ_LEFT   = SQ_RIGHT  - 7.0      # -4.0
SQ_TOP    = SQ_BOTTOM + 7.0      # 4.5
CORNER_R  = 1.3                  # radio de recorte de cada esquina

WAYPOINTS = [
    VS_X, VS_Y,                                    # inicio: sobre el borde derecho
    SQ_RIGHT,            SQ_TOP - CORNER_R,         # llega a la esquina sup. derecha
    SQ_RIGHT - CORNER_R, SQ_TOP,                    # sale de la esquina sup. derecha
    SQ_LEFT + CORNER_R,  SQ_TOP,                    # llega a la esquina sup. izquierda
    SQ_LEFT,             SQ_TOP - CORNER_R,         # sale de la esquina sup. izquierda
    SQ_LEFT,             SQ_BOTTOM + CORNER_R,       # llega a la esquina inf. izquierda
    SQ_LEFT + CORNER_R,  SQ_BOTTOM,                 # sale de la esquina inf. izquierda
    SQ_RIGHT - CORNER_R, SQ_BOTTOM,                 # llega a la esquina inf. derecha...
    ROBOT_X, ROBOT_Y-2,                               # ...y termina justo en ella
]

# Timing
T_START      = 3.0
T_ROBOT_STEP = 1.5
T_AIRE       = T_START + N_ROBOTS * T_ROBOT_STEP + 1.0   # 11.5 s
T_CONTROL    = T_AIRE + 1.0                               # 12.5 s
START_DELAY  = T_CONTROL - T_START + 2.0                 # 11.5 s

# Grabación de GIF (cámara cenital) — se detiene sola al completar la
# trayectoria de la estructura virtual, no por tiempo fijo
RECORD_GIF    = True
CAMERA_HEIGHT = 9.0     # antes 16.0: más cerca, robots se ven más grandes
CAMERA_X      = (SQ_RIGHT + SQ_LEFT) / 2.0    # centro del cuadrado de la ruta
CAMERA_Y      = (SQ_TOP + SQ_BOTTOM) / 2.0
GIF_FPS       = 10.0
STOP_DELAY    = 2.0

# Cierre de las gráficas (states_plotter_v2): margen generoso sobre el tiempo
# estimado para recorrer toda la trayectoria a v_max=0.25 m/s.
T_FINAL = 150.0


# ------------------------------------------------------------------
# Posiciones iniciales en V
# ------------------------------------------------------------------

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


# ------------------------------------------------------------------
# Spawns de robots
# ------------------------------------------------------------------

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
                    'gif_name':      'vs_formacion.gif',
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


# ------------------------------------------------------------------
# Launch description
# ------------------------------------------------------------------

def generate_launch_description():

    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )
    world_path = os.path.join(
        get_package_share_directory('simulador'), 'worlds', 'demo.world'
    )

    return LaunchDescription([

        DeclareLaunchArgument('save_dir',
            default_value='figuras/vs_formacion',
            description='Directorio donde se guardan gifs/ y frames/'),

        # --- Gazebo con plugin de estado ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world_path}.items()
        ),

        # --- Estructura virtual: esferas + trayectoria + publicación ---
        TimerAction(period=T_START, actions=[
            Node(
                package='simulador',
                executable='vs_node',
                name='vs_node',
                parameters=[{
                    'n_robots':    N_ROBOTS,
                    'angle_v':     ANGLE_V,
                    'd':           D,
                    'waypoints':   WAYPOINTS,
                    'v_max':       0.25,
                    'start_delay': START_DELAY,
                }],
                output='screen',
            )
        ]),

        # --- Robots ---
        OpaqueFunction(function=launch_setup),

    ])
