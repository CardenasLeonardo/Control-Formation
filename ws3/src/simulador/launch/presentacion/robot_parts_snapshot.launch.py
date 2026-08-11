import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ------------------------------------------------------------------
# Launch de PRESENTACIÓN: spawnea el robot real completo (articubot_one)
# JUNTO a sus piezas sueltas (chasis, 2 ruedas motrices, rueda castor,
# LiDAR) alineadas en fila frente a él, para mostrar que el robot es un
# ensamble de componentes configurables, no un modelo monolítico.
# Cámara isométrica reencuadrada (más lejos que en robot_isometric_snapshot)
# para cubrir robot + fila de piezas en la misma toma.
# ------------------------------------------------------------------

ROBOT_X = 0.0
ROBOT_Y = 0.0
ROBOT_Z = 0.1
ROBOT_YAW = 0.0

# Fila de piezas: delante del robot en +X, centrada en Y respecto a él.
# Piezas juntas (spacing chico) para que la captura no se alargue, pero
# con suficiente separación del chasis del robot (0.3x0.3) para que no
# se solapen visualmente en la vista isométrica.
PARTS_START_X = ROBOT_X + 0.9
PARTS_START_Y = ROBOT_Y - 0.3
PARTS_SPACING = 0.15

# Offset acumulado de la última pieza (lidar), calculado con la misma
# lógica borde a borde de spawn_parts_demo.py — ver ese archivo.
# 4 pasos: chasis->wheel_l, wheel_l->wheel_r, wheel_r->caster, caster->lidar
_PARTS_LAST_OFFSET = (0.15 + PARTS_SPACING + 0.05) + 3 * (0.05 + PARTS_SPACING + 0.05)

# Punto medio de la escena (robot + fila de piezas) para apuntar la cámara.
SCENE_CX = (ROBOT_X + PARTS_START_X) / 2.0
SCENE_CY = (ROBOT_Y + (PARTS_START_Y + _PARTS_LAST_OFFSET)) / 2.0

# Cámara isométrica: misma orientación que robot_isometric_snapshot.
CAM_OFFSET = 1.6
CAMERA_X = SCENE_CX + CAM_OFFSET
CAMERA_Y = SCENE_CY + CAM_OFFSET
CAMERA_Z = ROBOT_Z + CAM_OFFSET

CAM_ROLL  = 0.0
CAM_PITCH = 0.6154797086703874   # ≈ 35.264°, arctan(1/sqrt(2))
CAM_YAW   = -2.356194490192345   # ≈ -135°

T_START = 3.0
T_ROBOT = T_START + 0.5
T_PARTS = T_START + 0.5
T_CAM   = T_START + 0.5
T_SNAPSHOT = T_CAM + 2.0


def generate_launch_description():

    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )
    world_path = os.path.join(
        get_package_share_directory('simulador'), 'worlds', 'demo.world'
    )
    camera_sdf = os.path.join(
        get_package_share_directory('simulador'), 'models', 'isometric_camera.sdf'
    )
    articubot_pkg = get_package_share_directory('articubot_one')
    rsp_launch    = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')

    return LaunchDescription([

        DeclareLaunchArgument('save_dir',
            default_value='figuras/presentacion',
            description='Directorio donde se guarda la captura'),

        DeclareLaunchArgument('file_name',
            default_value='robot_parts.png',
            description='Nombre del archivo de la captura'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world_path}.items()
        ),

        TimerAction(period=T_START, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rsp_launch),
                launch_arguments={'namespace': 'robot0', 'use_sim_time': 'false'}.items()
            )
        ]),

        TimerAction(period=T_ROBOT, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', '/robot0/robot_description',
                    '-entity', 'robot0',
                    '-x', str(ROBOT_X), '-y', str(ROBOT_Y), '-z', str(ROBOT_Z),
                    '-Y', str(ROBOT_YAW),
                ],
                output='screen'
            )
        ]),

        TimerAction(period=T_PARTS, actions=[
            Node(
                package='simulador',
                executable='spawn_parts_demo',
                name='spawn_parts_demo',
                parameters=[{
                    'start_x': PARTS_START_X,
                    'start_y': PARTS_START_Y,
                    'spacing': PARTS_SPACING,
                }],
                output='screen',
            )
        ]),

        TimerAction(period=T_CAM, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', camera_sdf,
                    '-entity', 'isometric_camera',
                    '-x', str(CAMERA_X), '-y', str(CAMERA_Y), '-z', str(CAMERA_Z),
                    '-R', str(CAM_ROLL), '-P', str(CAM_PITCH), '-Y', str(CAM_YAW),
                ],
                output='screen'
            )
        ]),

        TimerAction(period=T_SNAPSHOT, actions=[
            Node(
                package='simulador',
                executable='snapshot_camera',
                name='snapshot_camera',
                parameters=[{
                    'image_topic': '/isometric_camera/image_raw',
                    'save_dir':    LaunchConfiguration('save_dir'),
                    'file_name':   LaunchConfiguration('file_name'),
                    'delay':       0.0,
                }],
                output='screen',
            )
        ]),

    ])
