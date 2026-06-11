import os
import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    n             = int(LaunchConfiguration('n_robots').perform(context))
    R             = float(LaunchConfiguration('neighbor_radius').perform(context))
    leader_id     = LaunchConfiguration('leader_id').perform(context)
    waypoints_str = LaunchConfiguration('waypoints').perform(context)
    t_final       = float(LaunchConfiguration('t_final').perform(context))
    save_dir      = LaunchConfiguration('save_dir').perform(context)
    waypoints     = [float(v) for v in waypoints_str.split(',')]

    sep = 2.0  # separación entre seguidores en la grilla inicial

    articubot_pkg = get_package_share_directory('articubot_one')
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )
    rsp_launch = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')

    # Líder en el origen; seguidores en grilla detrás (x negativo)
    n_followers = n - 1
    cols = math.ceil(math.sqrt(n_followers)) if n_followers > 0 else 1
    rows = math.ceil(n_followers / cols)     if n_followers > 0 else 1

    follower_positions = []
    for row in range(rows):
        for col in range(cols):
            if len(follower_positions) >= n_followers:
                break
            x = -(col + 1) * sep
            y = (row - rows // 2) * sep
            follower_positions.append((x, y))

    leader_num = int(leader_id.replace('robot', ''))
    positions  = {leader_num: (0.0, 0.0)}
    seg_idx    = 0
    for i in range(n):
        if i != leader_num:
            positions[i] = follower_positions[seg_idx]
            seg_idx += 1

    late_delay = 3.0 + n * 1.5 + 2.0

    actions = []

    actions.append(
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch))
    )

    # Staggered: robot i spawns at t = 3 + i*1.5 s
    for i in range(n):
        ns    = f'robot{i}'
        x0, y0 = positions[i]
        delay = 3.0 + i * 1.5

        rsp_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch),
            launch_arguments={'namespace': ns, 'use_sim_time': 'false'}.items()
        )

        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', f'/{ns}/robot_description',
                '-entity', ns,
                '-x', str(round(x0, 4)),
                '-y', str(round(y0, 4)),
                '-z', '0.1',
                '-Y', '0.0',
            ],
            output='screen'
        )

        if ns == leader_id:
            control_node = Node(
                package='control_nodes',
                executable='navigate_waypoints_pva',
                name='navigate_waypoints',
                namespace=ns,
                parameters=[{
                    'waypoints': waypoints,
                    'vmax': 0.25,
                    'wmax': 0.5,
                }],
                output='screen'
            )
        else:
            control_node = Node(
                package='control_nodes',
                executable='consenso_node',
                name='consenso',
                namespace=ns,
                parameters=[{
                    'consensus_type': 'lider',
                    'leader_id':      leader_id,
                    'k_leader':       2.0,
                    'k1':             1.0,
                    'k2':             1.5,
                    'vmax':           0.8,
                    'wmax':           0.8,
                }],
                output='screen'
            )

        actions.append(TimerAction(period=delay,       actions=[rsp_node]))
        actions.append(TimerAction(period=delay + 0.5, actions=[spawn_node]))

        # El líder espera a que todos los robots hayan spawneado antes de moverse
        control_delay = late_delay if ns == leader_id else delay + 1.5
        actions.append(TimerAction(period=control_delay, actions=[control_node]))

    # AIRE y plotters después de que todos los robots estén activos
    actions.append(TimerAction(period=late_delay, actions=[
        Node(
            package='simulador',
            executable='aire',
            name='aire',
            parameters=[{'neighbor_radius': R}],
            output='screen'
        )
    ]))

    actions.append(TimerAction(period=late_delay, actions=[
        Node(
            package='simulador',
            executable='states_plotter_v2',
            name='states_plotter_v2',
            parameters=[{
                'save_dir':          save_dir,
                't_final':           t_final,
                'n_robots_expected': n,
            }],
            output='screen'
        )
    ]))

    # actions.append(TimerAction(period=late_delay, actions=[
    #     Node(
    #         package='simulador',
    #         executable='pva_plotter',
    #         name='pva_plotter',
    #         parameters=[{
    #             'save_dir': save_dir,
    #             't_final':  t_final,
    #         }],
    #         output='screen'
    #     )
    # ]))

    return actions


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('n_robots',
            default_value='5',
            description='Total de robots (líder + seguidores)'),
        DeclareLaunchArgument('neighbor_radius',
            default_value='10.0',
            description='Radio de percepción AIRE (m)'),
        DeclareLaunchArgument('leader_id',
            default_value='robot0',
            description='Namespace del robot líder'),
        DeclareLaunchArgument('waypoints',
            default_value='5.0,0.0,5.0,5.0,0.0,5.0,0.0,0.0',
            description='Waypoints del líder: x0,y0,x1,y1,...'),
        DeclareLaunchArgument('t_final',
            default_value='90.0',
            description='Duración de la simulación (s)'),
        DeclareLaunchArgument('save_dir',
            default_value='figuras/consenso_lider',
            description='Directorio donde se guardan las figuras'),
        OpaqueFunction(function=launch_setup)
    ])
