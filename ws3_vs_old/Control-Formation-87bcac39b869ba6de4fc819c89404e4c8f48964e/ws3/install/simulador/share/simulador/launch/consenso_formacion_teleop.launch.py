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


def compute_initial_positions(n, leader_id, angle_v, d):
    n_followers = n - 1
    n_left  = n_followers // 2
    n_right = n_followers - n_left

    def rot(theta, vx, vy):
        c, s = math.cos(theta), math.sin(theta)
        return c * vx - s * vy, s * vx + c * vy

    leader_num = int(leader_id.replace('robot', ''))
    positions  = {leader_num: (0.0, 0.0)}

    idx = 1
    for k in range(1, n_left + 1):
        rx, ry = rot(angle_v, -k * d, 0.0)
        positions[idx] = (round(rx, 4), round(ry, 4))
        idx += 1
    for k in range(1, n_right + 1):
        rx, ry = rot(-angle_v, -k * d, 0.0)
        positions[idx] = (round(rx, 4), round(ry, 4))
        idx += 1

    return positions


def launch_setup(context, *args, **kwargs):

    n          = int(LaunchConfiguration('n_robots').perform(context))
    R          = float(LaunchConfiguration('neighbor_radius').perform(context))
    leader_id  = LaunchConfiguration('leader_id').perform(context)
    angle_v    = float(LaunchConfiguration('angle_v').perform(context))
    d          = float(LaunchConfiguration('d').perform(context))

    init_pos = compute_initial_positions(n, leader_id, angle_v, d)

    articubot_pkg = get_package_share_directory('articubot_one')
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )
    rsp_launch = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')

    actions = []

    actions.append(
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch))
    )

    for i in range(n):
        ns      = f'robot{i}'
        x0, y0  = init_pos.get(i, (0.0, i * 1.5))
        delay   = 3.0 + i * 1.5

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
                '-x', str(x0), '-y', str(y0), '-z', '0.1',
                '-Y', '0.0',
            ],
            output='screen'
        )

        if ns == leader_id:
            # Relay: publica estado para que los seguidores lo vean.
            # El usuario controla cmd_vel con teleop en otra terminal.
            control_node = Node(
                package='control_nodes',
                executable='consenso_node',
                name='consenso',
                namespace=ns,
                parameters=[{'consensus_type': 'relay'}],
                output='screen'
            )
        else:
            control_node = Node(
                package='control_nodes',
                executable='consenso_node',
                name='consenso',
                namespace=ns,
                parameters=[{
                    'consensus_type': 'formacion',
                    'leader_id':      leader_id,
                    'n_robots':       n,
                    'angle_v':        angle_v,
                    'd':              d,
                    'beta':           2.0,
                    'k1':             1.0,
                    'k2':             1.5,
                    'vmax':           0.8,
                    'wmax':           0.8,
                    'anchor_speed':   0.5,
                    'forward_only':   True,
                }],
                output='screen'
            )

        actions.append(TimerAction(period=delay,       actions=[rsp_node]))
        actions.append(TimerAction(period=delay + 0.5, actions=[spawn_node]))
        actions.append(TimerAction(period=delay + 1.5, actions=[control_node]))

    late_delay = 3.0 + n * 1.5 + 2.0

    actions.append(TimerAction(period=late_delay, actions=[
        Node(
            package='simulador',
            executable='aire',
            name='aire',
            parameters=[{'neighbor_radius': R}],
            output='screen'
        )
    ]))

    return actions


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('n_robots',
            default_value='5'),
        DeclareLaunchArgument('neighbor_radius',
            default_value='10.0'),
        DeclareLaunchArgument('leader_id',
            default_value='robot0'),
        DeclareLaunchArgument('angle_v',
            default_value='0.7854',
            description='Apertura de la V (rad), π/4 = 45°'),
        DeclareLaunchArgument('d',
            default_value='1.0',
            description='Separación entre robots en el brazo (m)'),
        OpaqueFunction(function=launch_setup)
    ])
