import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    goal_x = LaunchConfiguration('goal_x').perform(context)
    goal_y = LaunchConfiguration('goal_y').perform(context)

    articubot_pkg = get_package_share_directory('articubot_one')
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )
    rsp_launch = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')

    actions = []

    # Lanzar Gazebo
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        )
    )

    # robot_state_publisher
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch),
            launch_arguments={
                'namespace': 'robot0',
                'use_sim_time': 'true'
            }.items()
        )
    )

    # spawn_entity
    actions.append(
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', '/robot0/robot_description',
                '-entity', 'robot0',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1'
            ],
            output='screen'
        )
    )

    # navegación individual con PVA
    actions.append(
        Node(
            package='control_nodes',
            executable='navigate_individual_pva',
            name='navigate_individual_pva',
            namespace='robot0',
            parameters=[{
                'goal_x': float(goal_x),
                'goal_y': float(goal_y),
                'use_sim_time': True
            }],
            output='screen'
        )
    )

    # States plotter
    actions.append(
        Node(
            package='simulador',
            executable='states_plotter_v2',
            name='states_plotter_v2',
            output='screen'
        )
    )

    # PVA plotter
    actions.append(
        Node(
            package='simulador',
            executable='pva_plotter',
            name='pva_plotter',
            output='screen'
        )
    )

    return actions


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'goal_x',
            default_value='5.0',
            description='Coordenada x del objetivo'
        ),
        DeclareLaunchArgument(
            'goal_y',
            default_value='5.0',
            description='Coordenada y del objetivo'
        ),
        OpaqueFunction(function=launch_setup)
    ])