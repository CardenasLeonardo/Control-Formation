import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ------------------------------------------------------------------
# Demo didáctico previo a la estructura virtual: spawnea UN cubo con
# física completa (masa, inercia, colisión) en Gazebo, para explicar
# de forma genérica qué campos definen un modelo SDF (pose, geometría,
# color, inercia) antes de mostrar la variante "sin física" (kinematic,
# sin colisión) usada por las esferas de referencia de la estructura
# virtual. Mismos estándares visuales del resto del proyecto: piso
# transparente (demo.world), cámara cenital fija, GIF + frames.
# ------------------------------------------------------------------

CUBE_X = 0.0
CUBE_Y = 0.0
CUBE_Z = 1.0   # cae desde 1 m de altura, para que se note la física real

T_START = 3.0
T_CUBE  = T_START + 1.0

RECORD_GIF    = True
GIF_FPS       = 10.0
T_FINAL       = 8.0   # tiempo fijo: no hay meta que alcanzar en esta demo

CAMERA_X      = CUBE_X
CAMERA_Y      = CUBE_Y
CAMERA_HEIGHT = 4.0    # más baja que en los demos de formación: un solo
                       # cubo pequeño no necesita el encuadre amplio de 9m


def generate_launch_description():

    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )
    world_path = os.path.join(
        get_package_share_directory('simulador'), 'worlds', 'demo.world'
    )
    camera_sdf = os.path.join(
        get_package_share_directory('simulador'), 'models', 'overhead_camera.sdf'
    )

    return LaunchDescription([

        DeclareLaunchArgument('save_dir',
            default_value='figuras/spawn_cube_demo',
            description='Directorio donde se guardan gifs/ y frames/'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world_path}.items()
        ),

        TimerAction(period=T_START, actions=[
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
        ]),

        TimerAction(period=T_CUBE, actions=[
            Node(
                package='simulador',
                executable='spawn_cube_demo',
                name='spawn_cube_demo',
                parameters=[{
                    'x': CUBE_X,
                    'y': CUBE_Y,
                    'z': CUBE_Z,
                }],
                output='screen',
            )
        ]),

        TimerAction(period=T_CUBE, actions=[
            Node(
                package='simulador',
                executable='gif_recorder',
                name='gif_recorder',
                parameters=[{
                    'save_dir':      LaunchConfiguration('save_dir'),
                    'gif_name':      'spawn_cube_demo.gif',
                    'fps':           GIF_FPS,
                    'stop_mode':     'time',
                    't_final':       T_FINAL,
                    'auto_shutdown': True,
                }],
                output='screen'
            )
        ]),

    ])
