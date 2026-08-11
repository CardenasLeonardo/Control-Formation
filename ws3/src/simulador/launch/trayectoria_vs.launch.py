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
# Launch para probar SOLO la trayectoria de la estructura virtual, sin
# desplegar robots. Preserva todo lo demás de vs_formacion_noble.launch.py
# (Gazebo, cámara cenital, GIF, gráficas) para poder observar cómo se
# comporta la ruta del centroide al cambiar de estrategia de navegación.
# La generación de la trayectoria vive en un módulo intercambiable:
# control_nodes/algorithms/trayectoria_vs.py (actualmente spline
# Catmull-Rom); para probar otra estrategia basta con escribir una clase
# alterna con la misma interfaz ahí y apuntar trayectoria_vs_node.py a ella.
# ------------------------------------------------------------------

THETA_0  = math.pi / 2.0   # heading inicial: norte

# Centroide de la estructura virtual
VS_X = 3.0
VS_Y = 0.0

# Formación (solo para visualizar las anclas de referencia; no hay robots)
N_ROBOTS = 5
ANGLE_V  = math.pi / 4
D        = 1.0

# waypoints[0] es la posición inicial del centroide; la curva pasa por todos
# ellos. Misma trayectoria original de vs_formacion_noble.launch.py.
WAYPOINTS = [3.0, 0.0,   3.0, 4.0,   0.0, 7.0,  -4.0, 5.0,
             -4.0, 0.0,  -1.0, -3.0,  3.0, -2.5]

# Timing
T_START      = 3.0
T_AIRE       = T_START + 1.0                     # 4.0 s

# Grabación de GIF (cámara cenital) — se detiene sola al completar la
# trayectoria de la estructura virtual, no por tiempo fijo
RECORD_GIF    = True
GIF_FPS       = 10.0
STOP_DELAY    = 2.0

# Cierre de las gráficas (states_plotter_v2): margen generoso sobre el tiempo
# estimado para recorrer toda la trayectoria a v_max=0.25 m/s.
T_FINAL = 150.0

# Encuadre de cámara: mismo criterio que vs_formacion_noble.launch.py
# (CAMERA_HEIGHT=9.0 fijo). El centro se calcula del bounding box real de
# los waypoints.
_WP_X = WAYPOINTS[0::2]
_WP_Y = WAYPOINTS[1::2]
CAMERA_X      = (max(_WP_X) + min(_WP_X)) / 2.0
CAMERA_Y      = (max(_WP_Y) + min(_WP_Y)) / 2.0
CAMERA_HEIGHT = 9.0


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
                    'gif_name':      'trayectoria_vs.gif',
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

    # --- Gráfica: trayectoria del centroide de la estructura virtual ---
    actions.append(TimerAction(period=T_AIRE, actions=[
        Node(
            package='simulador',
            executable='states_plotter_v2',
            name='states_plotter_v2',
            parameters=[{
                'save_dir':           save_dir,
                't_final':            T_FINAL,   # respaldo de seguridad
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
            default_value='figuras/trayectoria_vs',
            description='Directorio donde se guardan gifs/ y frames/'),

        # --- Gazebo con plugin de estado ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world_path}.items()
        ),

        # --- Estructura virtual: esfera del centroide + anclas + trayectoria ---
        TimerAction(period=T_START, actions=[
            Node(
                package='control_nodes',
                executable='trayectoria_vs_node',
                name='trayectoria_vs_node',
                parameters=[{
                    'n_robots':    N_ROBOTS,
                    'angle_v':     ANGLE_V,
                    'd':           D,
                    'waypoints':   WAYPOINTS,
                    'v_max':       0.25,
                    'start_delay': 3.0,
                }],
                output='screen',
            )
        ]),

        # --- Cámara + GIF + gráfica ---
        OpaqueFunction(function=launch_setup),

    ])
