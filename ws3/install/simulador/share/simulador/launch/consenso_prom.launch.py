import os
import math
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


POSES_8 = [
    (0.0, 0.0,   math.pi / 4),       # robot0
    (4.0, 1.0,  -math.pi / 2),       # robot1
    (8.0, 0.0,  -math.pi / 4),       # robot2
    (1.0, 4.0,   math.pi),           # robot3
    (8.0, 4.0,   0.0),               # robot4
    (0.0, 8.0,   3*math.pi / 4),     # robot5
    (4.0, 9.0,   math.pi / 2),       # robot6
    (8.0, 8.0,  -3*math.pi / 4),     # robot7
]

# Grabación de GIF (cámara cenital) — se detiene sola al llegar a t_final
# (no hay señal de "meta alcanzada" en consenso promedio, solo t_final).
RECORD_GIF    = True
CAMERA_HEIGHT = 9.0   # estándar del proyecto — "más cerca, robots se ven más grandes"
GIF_FPS       = 10.0


def launch_setup(context, *args, **kwargs):
    # Leer parámetros del launch #
    n        = int(LaunchConfiguration('n_robots').perform(context))           # número de robots activos
    R        = float(LaunchConfiguration('neighbor_radius').perform(context))  # radio de percepción AIRE (m)
    t_final  = float(LaunchConfiguration('t_final').perform(context))          # duración de la simulación (s)
    save_dir = LaunchConfiguration('save_dir').perform(context)                # carpeta de destino de figuras
    poses    = POSES_8[:n]                                                     # recortar poses a los n robots usados

    # Rutas de paquetes y archivos de launch #
    articubot_pkg = get_package_share_directory('articubot_one')              # path del paquete articubot_one
    gazebo_launch = os.path.join(                                             # launch file principal de Gazebo
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )
    rsp_launch = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')   # launch del robot state publisher
    world_path = os.path.join(                                            # mundo con piso transparente
        get_package_share_directory('simulador'), 'worlds', 'demo.world'
    )
    camera_sdf = os.path.join(
        get_package_share_directory('simulador'), 'models', 'overhead_camera.sdf'
    )

    # Encuadre de cámara: bounding box de las poses iniciales usadas
    wp_x = [p[0] for p in poses]
    wp_y = [p[1] for p in poses]
    camera_x = (max(wp_x) + min(wp_x)) / 2.0
    camera_y = (max(wp_y) + min(wp_y)) / 2.0

    # Arranque de Gazebo (con piso transparente) #
    actions = []
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world_path}.items()
        )
    )

    if RECORD_GIF:
        actions.append(TimerAction(period=2.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', camera_sdf,
                    '-entity', 'overhead_camera',
                    '-x', str(camera_x), '-y', str(camera_y), '-z', str(CAMERA_HEIGHT),
                    '-P', '1.5708',
                ],
                output='screen'
            )
        ]))

    # Spawn escalonado: RSP → robot → nodo de control (uno por uno para no sobrecargar Gazebo) #
    for i, (x0, y0, yaw) in enumerate(poses):

        ns    = f'robot{i}'                  # namespace del robot (robot0, robot1, ...)
        delay = 3.0 + i * 1.5               # robot0 a 3 s, robot1 a 4.5 s, ..., robot7 a 13.5 s

        rsp_node = IncludeLaunchDescription(                    # publica robot_description en /{ns}/robot_description
            PythonLaunchDescriptionSource(rsp_launch),
            launch_arguments={
                'namespace':    ns,
                'use_sim_time': 'false'
            }.items()
        )

        spawn_node = Node(                                       # inserta el modelo URDF en Gazebo
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', f'/{ns}/robot_description',           # lee el URDF desde el topic
                '-entity', ns,                                  # nombre de la entidad en Gazebo
                '-x', str(round(x0, 4)),                        # posición inicial X
                '-y', str(round(y0, 4)),                        # posición inicial Y
                '-z', '0.1',                                    # altura sobre el suelo
                '-Y', str(round(yaw, 4)),                       # orientación inicial (yaw)
            ],
            output='screen'
        )

        control_node = Node(                                     # nodo de consenso para este robot
            package='control_nodes',
            executable='consenso_node',
            name='consenso',
            namespace=ns,
            parameters=[{
                'consensus_type': 'prom_err',                   # tipo: consenso por promedio de errores
                'k1':             0.8,                          # ganancia lineal
                'k2':             1.0,                          # ganancia angular
                'vmax':           1.0,                          # velocidad lineal máx (m/s)
                'wmax':           1.0,                          # velocidad angular máx (rad/s)
            }],
            output='screen'
        )

        actions.append(TimerAction(period=delay,       actions=[rsp_node]))      # publica URDF
        actions.append(TimerAction(period=delay + 0.5, actions=[spawn_node]))    # spawn en Gazebo
        actions.append(TimerAction(period=delay + 1.5, actions=[control_node]))  # arranca control

    # Nodos globales: AIRE + plotters (esperan a que todos los robots estén listos) #
    late_delay = 3.0 + n * 1.5 + 2.0                            # margen de 2 s tras el último robot

    actions.append(
        TimerAction(period=late_delay, actions=[
            Node(
                package='simulador',
                executable='aire',
                name='aire',
                parameters=[{'neighbor_radius': R}],             # publica vecindades dentro del radio R
                output='screen'
            )
        ])
    )

    actions.append(
        TimerAction(period=late_delay, actions=[
            Node(
                package='simulador',
                executable='states_plotter_v2',
                name='states_plotter_v2',
                parameters=[{
                    'save_dir':          save_dir,   # carpeta de destino de las figuras
                    't_final':           t_final,    # cierra y guarda al llegar a t_final
                    'n_robots_expected': n,          # espera a tener datos de n robots
                    # Sin líder ni formación: el "ideal" de cada robot es el
                    # centroide instantáneo del grupo, sin offset — todos
                    # deben converger al mismo punto.
                    'error_reference':   'centroid',
                }],
                output='screen'
            )
        ])
    )

    actions.append(
        TimerAction(period=late_delay, actions=[
            Node(
                package='simulador',
                executable='pva_plotter',
                name='pva_plotter',
                parameters=[{
                    'save_dir': save_dir,          # carpeta de destino
                    't_final':  t_final,                          # cierra al llegar a t_final
                }],
                output='screen'
            )
        ])
    )

    if RECORD_GIF:
        actions.append(TimerAction(period=late_delay, actions=[
            Node(
                package='simulador',
                executable='gif_recorder',
                name='gif_recorder',
                parameters=[{
                    'save_dir':      save_dir,
                    'gif_name':      'consenso_prom.gif',
                    'fps':           GIF_FPS,
                    't_final':       t_final,
                    'auto_shutdown': True,
                }],
                output='screen'
            )
        ]))

    return actions


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'n_robots',
            default_value='8',
            description='Número de robots (máx 8)'
        ),
        DeclareLaunchArgument(
            'neighbor_radius',
            default_value='5.0',
            description='Radio de percepción AIRE (metros)'
        ),
        DeclareLaunchArgument(
            't_final',
            default_value='60.0',
            description='Duración de la simulación en segundos'
        ),
        DeclareLaunchArgument(
            'save_dir',
            default_value='figuras/consenso_prom',
            description='Directorio donde guardar figuras (relativo al cwd)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
