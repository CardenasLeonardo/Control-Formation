from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='control_algorithms',
            executable='multi_controller',
            name='multi_controller',
            output='screen'
        )
    ])
