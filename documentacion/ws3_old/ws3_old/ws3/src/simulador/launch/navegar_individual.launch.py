import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='-5.0')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='5.0')
    t_final_arg = DeclareLaunchArgument('t_final', default_value='25.0')
    save_dir_arg = DeclareLaunchArgument('save_dir', default_value='figures_navegar_individual')

    articubot_pkg = get_package_share_directory('articubot_one')
    gazebo_launch  = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    rsp_launch     = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch))

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch),
        launch_arguments={'namespace': 'robot0', 'use_sim_time': 'false'}.items()
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot0/robot_description',
            '-entity', 'robot0',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
            '-Y', '0.0',   # facing east
        ],
        output='screen'
    )

    controller = Node(
        package='control_nodes',
        executable='navigate_individual_pva',
        name='navigate_individual_pva',
        namespace='robot0',
        parameters=[{
            'goal_x':      LaunchConfiguration('goal_x'),
            'goal_y':      LaunchConfiguration('goal_y'),
            'v_max':       0.5,
            'w_max':       0.5,
            'use_sim_time': False,
        }],
        output='screen'
    )

    plotter = Node(
        package='simulador',
        executable='states_plotter_v2',
        name='states_plotter_v2',
        parameters=[{
            'save_dir':          LaunchConfiguration('save_dir'),
            't_final':           LaunchConfiguration('t_final'),
            'n_robots_expected': 1,
            'trajectory_only':   True,
        }],
        output='screen'
    )

    return LaunchDescription([
        goal_x_arg, goal_y_arg, t_final_arg, save_dir_arg,
        gazebo, rsp, spawn, controller, plotter,
    ])
