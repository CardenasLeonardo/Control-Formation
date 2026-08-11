import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ------------------------------------------------------------------
# Launch de PRESENTACIÓN: spawnea UN robot real (articubot_one) y toma
# una captura fija desde una cámara en ángulo ISOMÉTRICO (no cenital):
# posicionada en diagonal (X, Y, Z iguales respecto al robot) y
# orientada para mirar exactamente hacia él, para ver el robot en
# perspectiva 3D en vez de la vista de planta usada en el resto del
# proyecto. Mismos estándares visuales: piso transparente (demo.world).
# ------------------------------------------------------------------

ROBOT_X = 0.0
ROBOT_Y = 0.0
ROBOT_Z = 0.1
ROBOT_YAW = 0.0

# Cámara equidistante en X, Y, Z respecto al robot (vista isométrica
# clásica: yaw=45°, pitch=arctan(1/sqrt(2))≈35.26°). CAM_OFFSET se
# deriva de una distancia diagonal cámara-robot de 1.3 m.
CAM_OFFSET = 0.751
CAMERA_X = ROBOT_X + CAM_OFFSET
CAMERA_Y = ROBOT_Y + CAM_OFFSET
CAMERA_Z = ROBOT_Z + CAM_OFFSET

# Orientación de la cámara (roll, pitch, yaw) para que mire al robot:
# yaw = -135° (apunta hacia -x,-y desde +x,+y), pitch = 35.264° hacia abajo.
CAM_ROLL  = 0.0
CAM_PITCH = 0.6154797086703874   # ≈ 35.264°, arctan(1/sqrt(2))
CAM_YAW   = -2.356194490192345   # ≈ -135°

T_START = 3.0
T_ROBOT = T_START + 0.5
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
            default_value='robot_isometric.png',
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
