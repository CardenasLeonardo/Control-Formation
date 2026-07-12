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
    save_dir      = LaunchConfiguration('save_dir').perform(context)
    waypoints     = [float(v) for v in waypoints_str.split(',')]

    simulador_pkg = get_package_share_directory('simulador')
    obstacle_sdf  = os.path.join(simulador_pkg, 'models', 'obstacle_cylinder.sdf')
    camera_sdf    = os.path.join(simulador_pkg, 'models', 'overhead_camera.sdf')

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

    # Cámara cenital sobre el centro del circuito (2.0, 2.0)
    actions.append(TimerAction(period=3.5, actions=[
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', camera_sdf,
                '-entity', 'overhead_camera',
                '-x', '2.0', '-y', '2.0', '-z', '7.0',
                '-P', '1.5708',
            ],
            output='screen'
        )
    ]))

    # Grabador de GIF — cierra solo al completar una vuelta del circuito
    actions.append(TimerAction(period=5.5, actions=[
        Node(
            package='simulador',
            executable='gif_recorder',
            name='gif_recorder',
            parameters=[{
                'save_dir':      save_dir,
                'gif_name':      'demo_pva.gif',
                'fps':           10.0,
                'stop_mode':     'goal',
                'goal_topic':    '/robot0/final_goal_reached',
                'stop_delay':    2.0,
                't_final':       180.0,
                'auto_shutdown': True,
            }],
            output='screen'
        )
    ]))

    # Cámara dinámica: se acerca al robot cuando entra al radio de un
    # obstáculo, vuelve a la vista amplia al alejarse.
    obstacles_flat = [v for _, ox, oy in OBSTACLES for v in (ox, oy)]
    actions.append(TimerAction(period=5.5, actions=[
        Node(
            package='simulador',
            executable='camera_zoom',
            name='camera_zoom',
            parameters=[{
                'entity_name': 'overhead_camera',
                'odom_topic':  '/robot0/odom',
                'obstacles':   obstacles_flat,
                'wide_x':      2.0,
                'wide_y':      2.0,
                'wide_z':      7.0,
                'zoom_z':      2.5,
                'zoom_radius': 1.8,
            }],
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

    # PVA plotter — corre hasta Ctrl+C, guarda PDF al cerrar; además graba
    # la evolución del polígono como GIF, sincronizado con la misma meta
    # que gif_recorder (stop_delay más corto para que termine primero).
    actions.append(TimerAction(period=6.0, actions=[
        Node(
            package='simulador',
            executable='pva_plotter',
            name='pva_plotter',
            parameters=[{
                'vmax':        vmax,
                'wmax':        wmax,
                'record_gif':  True,
                'save_dir':    save_dir,
                'gif_name':    'pva_evolution.gif',
                'record_fps':  10.0,
                'stop_mode':   'goal',
                'goal_topic':  '/robot0/final_goal_reached',
                'stop_delay':  1.0,
                't_final':     180.0,
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
        DeclareLaunchArgument(
            'save_dir',
            default_value='figuras/demo_pva',
            description='Directorio donde se guardan gifs/ y frames/'
        ),
        OpaqueFunction(function=launch_setup)
    ])
