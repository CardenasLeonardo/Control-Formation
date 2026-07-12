import os
import math

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ------------------------------------------------------------------
# Escenario
# ------------------------------------------------------------------

# Robot
ROBOT_X  =  0.0
ROBOT_Y  = -2.0
THETA_0  = math.pi / 2.0   # Heading norte

# Obstáculo: caja roja 2 m de ancho directo en la trayectoria
OBS_X    =  0.0
OBS_Y    =  4.0
OBS_Z    =  0.5             # Mitad de la altura (caja de 1 m)

# Meta al otro lado del obstáculo
WAYPOINTS = [0.0, 10.0]

# Rayos LIDAR usados por PVA (0 = todos los 360)
N_RAYS_USED = 36

# Timing
T_RSP       = 3.0
T_SPAWN_BOT = T_RSP + 0.5
T_SPAWN_OBS = T_RSP + 1.0
T_CONTROL   = T_RSP + 5.0


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
    obs_sdf = os.path.join(
        get_package_share_directory('simulador'), 'models', 'box_obstacle.sdf'
    )
    articubot_pkg = get_package_share_directory('articubot_one')
    rsp_launch    = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')

    return LaunchDescription([

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

        # --- Spawn obstáculo (caja roja estática) ---
        TimerAction(period=T_SPAWN_OBS, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file',   obs_sdf,
                    '-entity', 'obstacle_0',
                    '-x', str(OBS_X),
                    '-y', str(OBS_Y),
                    '-z', str(OBS_Z),
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
                    'n_rays_used': N_RAYS_USED,
                }],
                output='screen'
            )
        ]),

        # --- Visualizador PVA en tiempo real ---
        TimerAction(period=T_CONTROL, actions=[
            Node(
                package='simulador',
                executable='pva_plotter',
                name='pva_plotter',
                parameters=[{
                    'vmax': 0.5,
                    'wmax': 0.6,
                }],
                output='screen'
            )
        ]),

    ])
