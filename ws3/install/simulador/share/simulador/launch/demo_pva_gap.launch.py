import os
import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ------------------------------------------------------------------
# Escenario: robot avanzando en línea recta entre dos obstáculos
# ------------------------------------------------------------------

# Robot
ROBOT_X = 0.0
ROBOT_Y = -3.0
THETA_0 = math.pi / 2.0   # Heading norte

# Dos obstáculos cilíndricos formando una brecha centrada en x=0
GAP_Y     = 1.0
OBS_LEFT  = ('obs_left',  -1.5, GAP_Y)
OBS_RIGHT = ('obs_right',  1.5, GAP_Y)

# Meta al otro lado de la brecha
WAYPOINTS = [0.0, 7.0]

# Cámara cenital centrada sobre la brecha
CAMERA_X, CAMERA_Y, CAMERA_HEIGHT = 0.0, 2.0, 8.0

# Timing
T_RSP       = 3.0
T_SPAWN_BOT = T_RSP + 0.5
T_SPAWN_OBS = T_RSP + 1.0
T_CAMERA    = T_RSP + 1.0
T_CONTROL   = T_RSP + 5.0
T_RECORD    = T_CONTROL + 0.5


def generate_launch_description():

    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )
    world_path = os.path.join(
        get_package_share_directory('simulador'), 'worlds', 'demo.world'
    )
    obs_sdf = os.path.join(
        get_package_share_directory('simulador'), 'models', 'obstacle_cylinder.sdf'
    )
    camera_sdf = os.path.join(
        get_package_share_directory('simulador'), 'models', 'overhead_camera.sdf'
    )
    articubot_pkg = get_package_share_directory('articubot_one')
    rsp_launch    = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')

    return LaunchDescription([

        DeclareLaunchArgument(
            'save_dir',
            default_value='figuras/demo_pva_gap',
            description='Directorio donde se guardan gifs/ y frames/ (cámara y gráfica)'
        ),

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

        # --- Spawn de los dos obstáculos que forman la brecha ---
        TimerAction(period=T_SPAWN_OBS, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file',   obs_sdf,
                    '-entity', OBS_LEFT[0],
                    '-x', str(OBS_LEFT[1]),
                    '-y', str(OBS_LEFT[2]),
                    '-z', '0.5',
                ],
                output='screen'
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file',   obs_sdf,
                    '-entity', OBS_RIGHT[0],
                    '-x', str(OBS_RIGHT[1]),
                    '-y', str(OBS_RIGHT[2]),
                    '-z', '0.5',
                ],
                output='screen'
            ),
        ]),

        # --- Cámara cenital sobre la brecha ---
        TimerAction(period=T_CAMERA, actions=[
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

        # --- Navegación con PVA ---
        TimerAction(period=T_CONTROL, actions=[
            Node(
                package='control_nodes',
                executable='navigate_waypoints_pva',
                name='navigate',
                namespace='robot0',
                parameters=[{
                    'waypoints':   WAYPOINTS,
                    'vmax':        0.5,
                    'wmax':        0.6,
                    'n_rays_used': 36,
                }],
                output='screen'
            )
        ]),

        # --- Visualizador PVA en tiempo real, grabando la evolución del polígono ---
        TimerAction(period=T_RECORD, actions=[
            Node(
                package='simulador',
                executable='pva_plotter',
                name='pva_plotter',
                parameters=[{
                    'vmax':          0.5,
                    'wmax':          0.6,
                    'record_gif':    True,
                    'save_dir':      LaunchConfiguration('save_dir'),
                    'gif_name':      'pva_evolution.gif',
                    'record_fps':    5.0,
                    'stop_mode':     'goal',
                    'goal_topic':    '/robot0/final_goal_reached',
                    'stop_delay':    1.5,
                    't_final':       90.0,
                }],
                output='screen'
            )
        ]),

        # --- Grabador de la cámara cenital, cierra la simulación al terminar ---
        TimerAction(period=T_RECORD, actions=[
            Node(
                package='simulador',
                executable='gif_recorder',
                name='gif_recorder',
                parameters=[{
                    'save_dir':      LaunchConfiguration('save_dir'),
                    'gif_name':      'demo_pva_gap.gif',
                    'fps':           10.0,
                    'stop_mode':     'goal',
                    'goal_topic':    '/robot0/final_goal_reached',
                    'stop_delay':    3.0,
                    't_final':       90.0,
                    'auto_shutdown': True,
                }],
                output='screen'
            )
        ]),

        # --- Cámara dinámica: se acerca al robot al entrar en el radio de
        #     cualquiera de los dos obstáculos, vuelve a la vista amplia
        #     al alejarse. Deja ver el corte antes/durante/después de la brecha. ---
        TimerAction(period=T_RECORD, actions=[
            Node(
                package='simulador',
                executable='camera_zoom',
                name='camera_zoom',
                parameters=[{
                    'entity_name': 'overhead_camera',
                    'odom_topic':  '/robot0/odom',
                    'obstacles':   [OBS_LEFT[1], OBS_LEFT[2], OBS_RIGHT[1], OBS_RIGHT[2]],
                    'wide_x':      CAMERA_X,
                    'wide_y':      CAMERA_Y,
                    'wide_z':      CAMERA_HEIGHT,
                    'zoom_z':      2.5,
                    'zoom_radius': 3.5,
                    'update_rate': 2.0,
                }],
                output='screen'
            )
        ]),

    ])
