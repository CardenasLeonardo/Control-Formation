import os
import math
import tempfile

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
# Escenario de UN solo robot con el mismo curso de obstáculos y estética
# de siempre (pared ancha + slalom de pilares, cámara cenital, GIFs). El
# módulo de bordeo de obstáculos ("boundary following") fue retirado de
# control_nodes/algorithms/pva.py: el PVA ahora solo hace evasión
# instantánea vía QP (módulo 1). Frente a un obstáculo que bloquee por
# completo la línea recta a la meta, el robot ya no lo rodea — frena.
# Se conserva el escenario y las gráficas tal como estaban.
#
# Se graban/guardan, mismo patrón de vs_formacion_noble.launch.py:
#   1. GIF de la cámara cenital de Gazebo (gif_recorder) — la escena real.
#   2. GIF de la evolución del polígono FVP (pva_plotter, record_gif) — el
#      espacio de velocidades tal como en las Figuras 6/8/10 del paper.
#   3. GIF combinado (gif_combiner) — cada frame es la unión lado a lado
#      (más ancho, no más alto) del frame i de la cámara con el frame i
#      del PVA, sincronizados por índice.
#   4. Gráfica de trayectoria (states_plotter_v2, trajectory_only) — la
#      evolución de posición del robot (inicio/fin/orientación), sin
#      cálculo de error de formación porque aquí no hay estructura virtual.
# Todo se detiene solo al llegar a la meta (stop_mode='goal'). gif_combiner
# usa un stop_delay MENOR que gif_recorder para terminar de escribir su
# GIF antes de que el auto_shutdown de gif_recorder mate la simulación.
# ------------------------------------------------------------------

# Robot
ROBOT_X  =  0.0
ROBOT_Y  = -3.0
THETA_0  = math.pi / 2.0   # heading norte

# Pared original (se conserva): 4m de ancho, centrada en x=1.0 -> deja
# un hueco a la izquierda (x < -1.0) por el que el robot puede escapar
# rodeando. A partir de ahí, MUCHOS obstáculos más (pilares de tamaños
# variados) formando un curso tipo slalom hasta una meta mucho más
# lejana — para forzar VARIOS episodios de deadlock/bordeo sucesivos, no
# solo uno, y ejercitar el rastreo de restricción con múltiples
# obstáculos distintos en escena simultáneamente.
WALL_X  = 1.0
WALL_Y  = 2.0
WALL_Z  = 0.5

# (nombre, x, y, ancho_x, ancho_y) — z se fija a 0.5 (mismo alto que la
# pared original, 1.0m) para todos.
OBSTACLES = [
    ('wall_0', WALL_X, WALL_Y, 4.0, 0.3),
    ('obs_1', -2.5, 4.0,  1.0, 1.0),
    ('obs_2',  2.2, 4.6,  0.8, 1.6),
    ('obs_3', -1.0, 6.0,  1.4, 0.6),
    ('obs_4',  3.0, 7.0,  0.6, 0.6),
    ('obs_5', -3.2, 7.8,  0.6, 2.4),
    ('obs_6',  0.5, 9.0,  2.2, 0.5),
    ('obs_7', -1.6, 10.4, 1.0, 1.0),
    ('obs_8',  2.0, 11.4, 0.7, 0.7),
    ('obs_9', -0.3, 12.6, 1.8, 0.5),
]
OBSTACLE_Z = 0.5

# Meta mucho más lejana, al final del curso de obstáculos — la línea
# recta robot->meta cruza varios de ellos, no solo la primera pared.
WAYPOINTS = [0.0, 14.0]

# Rayos LIDAR usados por PVA (0 = todos los 360)
N_RAYS_USED = 72

# PVA — d_influence generoso para detectar la pared con margen.
D_SAFE      = 0.45
D_INFLUENCE = 2.0

VMAX = 0.5
WMAX = 0.6

# Timing
T_RSP       = 3.0
T_SPAWN_BOT = T_RSP + 0.5
T_SPAWN_OBS = T_RSP + 1.0   # primer obstáculo; el resto se escalona 0.2s c/u
OBS_SPAWN_STEP = 0.2
T_CONTROL   = T_SPAWN_OBS + len(OBSTACLES) * OBS_SPAWN_STEP + 1.5

GOAL_TOPIC = '/robot0/final_goal_reached'

# Grabación de GIF (cámara cenital) — se detiene sola al llegar a la meta
RECORD_GIF    = True
GIF_FPS       = 10.0
STOP_DELAY    = 2.0
# Curso mucho más largo (10 obstáculos, meta a 14m en vez de 6m) y con
# varios episodios de bordeo esperados -> respaldo de tiempo más generoso.
T_FINAL       = 240.0

# gif_combiner debe terminar de leer/combinar los PNG de ambos ANTES del
# pkill -9 de gif_recorder (auto_shutdown, dispara 1s después de su propio
# stop_delay=STOP_DELAY). Con stop_delay más chico se dispara primero.
COMBINE_STOP_DELAY = STOP_DELAY - 0.5   # 1.5s: sobra margen frente al 1s de gif_recorder

# Encuadre de cámara: cubre robot, TODOS los obstáculos y la meta —
# curso mucho más largo ahora (y de -3 a 14), se sube la altura para que
# el fov (1.3 rad) siga cubriendo todo el recorrido.
CAMERA_X      = 0.0
CAMERA_Y      = 5.5
CAMERA_HEIGHT = 13.0


# ------------------------------------------------------------------
# Nodos que dependen de save_dir (resuelto en runtime)
# ------------------------------------------------------------------

def launch_setup(context, *args, **kwargs):

    save_dir = LaunchConfiguration('save_dir').perform(context)

    camera_sdf = os.path.join(
        get_package_share_directory('simulador'), 'models', 'overhead_camera.sdf'
    )

    actions = []

    if RECORD_GIF:
        actions.append(TimerAction(period=T_RSP, actions=[
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

        actions.append(TimerAction(period=T_CONTROL, actions=[
            Node(
                package='simulador',
                executable='gif_recorder',
                name='gif_recorder',
                parameters=[{
                    'save_dir':      save_dir,
                    'gif_name':      'single_robot_pva_deadlock.gif',
                    'fps':           GIF_FPS,
                    'stop_mode':     'goal',
                    'goal_topic':    GOAL_TOPIC,
                    'stop_delay':    STOP_DELAY,
                    't_final':       T_FINAL,
                    'auto_shutdown': True,
                }],
                output='screen'
            )
        ]))

    # --- Navegación con PVA (evasión instantánea vía QP) ---
    actions.append(TimerAction(period=T_CONTROL, actions=[
        Node(
            package='control_nodes',
            executable='navigate_waypoints_pva',
            name='navigate',
            namespace='robot0',
            parameters=[{
                'waypoints':    WAYPOINTS,
                'vmax':         VMAX,
                'wmax':         WMAX,
                'n_rays_used':  N_RAYS_USED,
                'loop':         False,
                'd_safe':       D_SAFE,
                'd_influence':  D_INFLUENCE,
            }],
            output='screen'
        )
    ]))

    # --- Visualizador PVA en tiempo real (polígono FVP + vértice elegido),
    # grabado a GIF con el mismo criterio de parada que gif_recorder ---
    actions.append(TimerAction(period=T_CONTROL, actions=[
        Node(
            package='simulador',
            executable='pva_plotter',
            name='pva_plotter',
            parameters=[{
                'vmax':        VMAX,
                'wmax':        WMAX,
                'record_gif':  RECORD_GIF,
                'save_dir':    save_dir,
                'gif_name':    'single_robot_pva_deadlock_fvp.gif',
                'record_fps':  GIF_FPS,
                'stop_mode':   'goal',
                'goal_topic':  GOAL_TOPIC,
                'stop_delay':  STOP_DELAY,
                't_final':     T_FINAL,
            }],
            output='screen'
        )
    ]))

    # --- Gráfica de trayectoria (posición del robot) — un solo robot,
    # sin estructura virtual, así que trajectory_only en vez de
    # error_vs_virtual_structure ---
    actions.append(TimerAction(period=T_CONTROL, actions=[
        Node(
            package='simulador',
            executable='states_plotter_v2',
            name='states_plotter_v2',
            parameters=[{
                'save_dir':          save_dir,
                't_final':           T_FINAL,   # respaldo de seguridad
                'n_robots_expected': 1,
                'trajectory_only':   True,
                'stop_mode':         'goal',
                'goal_topic':        GOAL_TOPIC,
                'stop_delay':        STOP_DELAY,
            }],
            output='screen'
        )
    ]))

    # --- GIF combinado: cada frame = frame cámara + frame PVA, lado a
    # lado, sincronizados por índice. stop_delay MENOR que gif_recorder
    # para escribir antes del auto_shutdown (pkill -9) ---
    if RECORD_GIF:
        actions.append(TimerAction(period=T_CONTROL, actions=[
            Node(
                package='simulador',
                executable='gif_combiner',
                name='gif_combiner',
                parameters=[{
                    'save_dir':     save_dir,
                    'gif_name':     'single_robot_pva_deadlock_combined.gif',
                    'fps':          GIF_FPS,
                    'stop_mode':    'goal',
                    'goal_topic':   GOAL_TOPIC,
                    'stop_delay':   COMBINE_STOP_DELAY,
                    't_final':      T_FINAL,
                }],
                output='screen'
            )
        ]))

    return actions


# ------------------------------------------------------------------
# Launch description
# ------------------------------------------------------------------

def _obstacle_sdf(name, sx, sy, sz=1.0, color='0.8 0.2 0.2 1.0'):
    # Caja estática genérica (mismo patrón que wall_obstacle.sdf, pero
    # con tamaño parametrizable) — una por obstáculo del curso.
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="col">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
      <visual name="vis">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def generate_launch_description():

    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )
    world_path = os.path.join(
        get_package_share_directory('simulador'), 'worlds', 'demo.world'
    )
    articubot_pkg = get_package_share_directory('articubot_one')
    rsp_launch    = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')

    # SDF de cada obstáculo del curso, escrito a un archivo temporal
    # (spawn_entity.py solo acepta '-file <ruta>', no XML inline) —
    # generados una vez al construir la descripción de lanzamiento.
    obstacles_tmp_dir = tempfile.mkdtemp(prefix='pva_multi_obstacles_')
    obstacle_sdf_paths = {}
    for name, ox, oy, sx, sy in OBSTACLES:
        path = os.path.join(obstacles_tmp_dir, f'{name}.sdf')
        with open(path, 'w') as f:
            f.write(_obstacle_sdf(name, sx, sy, OBSTACLE_Z * 2.0))
        obstacle_sdf_paths[name] = path

    obstacle_spawn_actions = []
    for i, (name, ox, oy, sx, sy) in enumerate(OBSTACLES):
        obstacle_spawn_actions.append(TimerAction(
            period=T_SPAWN_OBS + i * OBS_SPAWN_STEP,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-file',   obstacle_sdf_paths[name],
                        '-entity', name,
                        '-x', str(ox),
                        '-y', str(oy),
                        '-z', str(OBSTACLE_Z),
                    ],
                    output='screen'
                )
            ]
        ))

    return LaunchDescription([

        DeclareLaunchArgument('save_dir',
            default_value='figuras/single_robot_pva_deadlock',
            description='Directorio donde se guardan gifs/ y frames/'),

        # --- Gazebo ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world_path}.items()
        ),

        # --- Robot: robot_state_publisher ---
        TimerAction(period=T_RSP, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rsp_launch),
                launch_arguments={
                    'namespace': 'robot0',
                    'use_sim_time': 'false'
                }.items()
            )
        ]),

        # --- Spawn robot ---
        TimerAction(period=T_SPAWN_BOT, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', '/robot0/robot_description',
                    '-entity', 'robot0',
                    '-x', str(ROBOT_X),
                    '-y', str(ROBOT_Y),
                    '-z', '0.1',
                    '-Y', str(THETA_0),
                ],
                output='screen'
            )
        ]),

        # --- Spawn de TODOS los obstáculos del curso, escalonado
        # (OBS_SPAWN_STEP entre cada uno, mismo patrón que vs_node.py
        # para no saturar /spawn_entity con llamadas simultáneas) ---
        *obstacle_spawn_actions,

        # --- Robot + cámara + gif_recorder + pva_plotter (dependen de save_dir) ---
        OpaqueFunction(function=launch_setup),

    ])
