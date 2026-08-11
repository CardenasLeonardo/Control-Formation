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
# Variante de vs_formacion.launch.py con la trayectoria ORIGINAL
# (Control-Formation-87bcac3, experimentación previa) en vez del
# cuadrado con esquinas redondeadas. vs_formacion.launch.py se deja
# intacto a propósito: con el cuadrado se reproduce el efecto látigo
# en algunos robots por no ser una trayectoria "noble" respecto a las
# capacidades cinemáticas individuales — eso se documenta tal cual.
# Esta variante usa una ruta que sí respeta esas capacidades.
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

# waypoints[0] es la posición inicial del centroide; la curva pasa por todos
# ellos. Ruta de ESTRÉS (deliberadamente exigente, a diferencia de la
# trayectoria "noble" original de la experimentación previa): muchos
# waypoints, giros cerrados entre segmentos consecutivos (71°-152°,
# incluido un giro casi en U) y tramos de longitud muy dispar (2.5m a
# 9m), para ver hasta dónde aguanta la formación sin inducir el efecto
# látigo del cuadrado de vs_formacion.launch.py. El último punto sigue
# siendo (ROBOT_X, ROBOT_Y) —el spawn físico de los robots—, no
# (VS_X, VS_Y), para que el GIF cierre el ciclo exactamente donde
# arrancaron.
WAYPOINTS = [
    3.0, 0.0,      # inicio (VS_X, VS_Y)
    3.0, 4.0,      # tramo largo al norte
    0.5, 4.3,      # giro cerrado (~83°)
    0.2, 1.0,      # vuelta abrupta hacia el sur (~92°)
    3.5, 1.3,      # zigzag: gira al este (~100°)
    3.8, -2.0,     # otra vuelta cerrada hacia el sur (~90°)
    0.0, -2.3,     # gira hacia el oeste (~91°)
    -3.5, 6.0,     # tramo largo diagonal (9m)
    -4.0, 0.0,     # giro muy cerrado, casi en U (~152°)
    -1.0, -0.3,    # zigzag corto tras el tramo largo (~89°)
    -1.0, -3.0,    # baja (~84°)
    ROBOT_X, ROBOT_Y - 2,  # cierre del loop
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
GIF_FPS       = 10.0
STOP_DELAY    = 2.0

# Cierre de las gráficas (states_plotter_v2): margen generoso sobre el tiempo
# estimado para recorrer toda la trayectoria a v_max=0.25 m/s. Ruta de
# estrés: ~45m de distancia poligonal entre waypoints, pero la curva
# Catmull-Rom real es más larga en los giros cerrados (puede formar
# bucles locales) — se sube el margen respecto a la ruta original.
T_FINAL = 260.0

# Encuadre de cámara: misma condición de toma que vs_formacion.launch.py
# (CAMERA_HEIGHT=9.0 fijo, "más cerca, robots se ven más grandes"). El centro
# se calcula del bounding box real de los waypoints —aquí no hay un cuadrado
# de referencia como en la otra trayectoria— y a 9m de altura (fov=1.3 rad)
# cubre de sobra los ~10m del lado más largo de esta ruta (~13.7m de diámetro
# visible, ~3.7m de margen).
_WP_X = WAYPOINTS[0::2]
_WP_Y = WAYPOINTS[1::2]
CAMERA_X      = (max(_WP_X) + min(_WP_X)) / 2.0
CAMERA_Y      = (max(_WP_Y) + min(_WP_Y)) / 2.0
CAMERA_HEIGHT = 9.0


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
                    'gif_name':      'vs_formacion_noble.gif',
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
                't_final':                    T_FINAL,   # respaldo de seguridad
                'n_robots_expected':          N_ROBOTS,
                'error_vs_virtual_structure': True,
                'formation_angle_v':          ANGLE_V,
                'formation_d':                D,
                # Guarda al completar la trayectoria (antes de que
                # gif_recorder mate este proceso con su auto_shutdown/pkill
                # si termina antes de T_FINAL, que es lo normal).
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
            default_value='figuras/vs_formacion_noble',
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
