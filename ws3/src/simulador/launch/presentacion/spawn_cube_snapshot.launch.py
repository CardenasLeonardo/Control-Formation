import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ------------------------------------------------------------------
# Launch de PRESENTACIÓN: spawnea un cubo con física completa y toma
# UNA sola captura fija de la cámara cenital (no un GIF), para usar
# como imagen estática en las diapositivas. El cubo cae ~1s antes de
# la captura para que se vea asentado sobre el piso, no en el aire.
# Mismos estándares visuales del proyecto: piso transparente
# (demo.world), cámara cenital fija.
# ------------------------------------------------------------------

CUBE_X = 0.0
CUBE_Y = 0.0
CUBE_Z = 1.0   # cae desde 1 m de altura

T_START    = 3.0
T_CUBE     = T_START + 1.0
T_SNAPSHOT = T_CUBE + 2.0   # margen para que el cubo caiga y se asiente

CAMERA_X      = CUBE_X
CAMERA_Y      = CUBE_Y
CAMERA_HEIGHT = 4.0


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
            default_value='figuras/presentacion',
            description='Directorio donde se guarda la captura'),

        DeclareLaunchArgument('file_name',
            default_value='spawn_cube_demo.png',
            description='Nombre del archivo de la captura'),

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

        TimerAction(period=T_SNAPSHOT, actions=[
            Node(
                package='simulador',
                executable='snapshot_camera',
                name='snapshot_camera',
                parameters=[{
                    'save_dir':  LaunchConfiguration('save_dir'),
                    'file_name': LaunchConfiguration('file_name'),
                    'delay':     0.0,
                }],
                output='screen',
            )
        ]),

    ])
