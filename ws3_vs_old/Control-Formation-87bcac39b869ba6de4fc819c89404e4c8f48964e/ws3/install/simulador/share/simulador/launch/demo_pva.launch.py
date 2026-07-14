import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


# Circuito cuadrado: (0,0)→(4,0)→(4,4)→(0,4)→(0,0)
# Obstáculos a 2 m de cada segmento, escalonados para no solaparse.
# Forman un diamante interior que el robot esquiva en cada vuelta.
#
#   (0,4)──────────────────(4,4)
#     │      obs_4(2,3)      │
#  obs_1(1,2)    ◆    obs_3(3,2)
#     │      obs_2(2,1)      │
#   (0,0)──────────────────(4,0)
#
OBSTACLES = [
    ('obs_1', 1.0, 2.0),   # sur:   2m al norte del segmento sur,   en x=1
    ('obs_2', 2.0, 1.0),   # este:  2m al oeste del segmento este,  en y=1
    ('obs_3', 3.0, 2.0),   # norte: 2m al sur  del segmento norte,  en x=3
    ('obs_4', 2.0, 3.0),   # oeste: 2m al este del segmento oeste,  en y=3
]


def launch_setup(context, *args, **kwargs):

    waypoints_str = LaunchConfiguration('waypoints').perform(context)
    vmax          = float(LaunchConfiguration('vmax').perform(context))
    wmax          = float(LaunchConfiguration('wmax').perform(context))
    n_rays_used   = int(LaunchConfiguration('n_rays_used').perform(context))
    waypoints     = [float(v) for v in waypoints_str.split(',')]

    simulador_pkg = get_package_share_directory('simulador')
    obstacle_sdf  = os.path.join(simulador_pkg, 'models', 'obstacle_cylinder.sdf')

    articubot_pkg = get_package_share_directory('articubot_one')
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )
    rsp_launch = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')

    actions = []

    actions.append(
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch))
    )

    # RSP
    actions.append(TimerAction(period=3.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch),
            launch_arguments={'namespace': 'robot0', 'use_sim_time': 'false'}.items()
        )
    ]))

    # Spawn robot0 en el origen
    actions.append(TimerAction(period=3.5, actions=[
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', '/robot0/robot_description',
                '-entity', 'robot0',
                '-x', '0.0', '-y', '0.0', '-z', '0.1', '-Y', '0.0',
            ],
            output='screen'
        )
    ]))

    # Spawn obstáculos
    for name, ox, oy in OBSTACLES:
        actions.append(TimerAction(period=3.5, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', obstacle_sdf,
                    '-entity', name,
                    '-x', str(ox), '-y', str(oy), '-z', '0.5',
                ],
                output='screen'
            )
        ]))

    # Nodo de navegación por waypoints
    actions.append(TimerAction(period=5.0, actions=[
        Node(
            package='control_nodes',
            executable='navigate_waypoints_pva',
            name='navigate_waypoints',
            namespace='robot0',
            parameters=[{
                'waypoints':   waypoints,
                'vmax':        vmax,
                'wmax':        wmax,
                'n_rays_used': n_rays_used,
            }],
            output='screen'
        )
    ]))

    # PVA plotter — corre hasta Ctrl+C, guarda PDF al cerrar
    actions.append(TimerAction(period=6.0, actions=[
        Node(
            package='simulador',
            executable='pva_plotter',
            name='pva_plotter',
            parameters=[{
                'vmax': vmax,
                'wmax': wmax,
            }],
            output='screen'
        )
    ]))

    return actions


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoints',
            default_value='4.0,0.0,4.0,4.0,0.0,4.0,0.0,0.0',
            description='Waypoints en bucle: x0,y0,x1,y1,...'
        ),
        DeclareLaunchArgument(
            'vmax',
            default_value='0.5',
            description='Velocidad lineal máxima (m/s)'
        ),
        DeclareLaunchArgument(
            'wmax',
            default_value='1.0',
            description='Velocidad angular máxima (rad/s)'
        ),
        DeclareLaunchArgument(
            'n_rays_used',
            default_value='0',
            description='Rayos LIDAR a usar (0 = todos los 360)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
