import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory('articubot_one')
    xacro_file = os.path.join(pkg, 'description', 'robot.urdf.xacro')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(
    Command(['xacro ', xacro_file]),
    value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='robot0',
            description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
                'frame_prefix': [namespace, '/']
            }],
            output='screen'
        )
    ])