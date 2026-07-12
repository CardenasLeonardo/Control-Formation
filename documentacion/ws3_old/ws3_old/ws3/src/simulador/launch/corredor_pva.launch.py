import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


CYLINDER_SDF = """\
<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <pose>0 0 0.5 0 0 0</pose>
      <collision name='collision'>
        <geometry>
          <cylinder><radius>0.2</radius><length>1.0</length></cylinder>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <cylinder><radius>0.2</radius><length>1.0</length></cylinder>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def make_cylinder_node(name, x, y):
    tmp = tempfile.NamedTemporaryFile(suffix='.sdf', delete=False, mode='w')
    tmp.write(CYLINDER_SDF.format(name=name))
    tmp.close()
    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', tmp.name, '-entity', name,
                   '-x', str(x), '-y', str(y), '-z', '0.0'],
        output='screen'
    )


def launch_setup(context, *args, **kwargs):

    goal_x  = float(LaunchConfiguration('goal_x').perform(context))
    goal_y  = float(LaunchConfiguration('goal_y').perform(context))
    gap     = float(LaunchConfiguration('gap').perform(context))
    obs_x   = goal_x / 2.0

    articubot_pkg = get_package_share_directory('articubot_one')
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )
    rsp_launch = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')

    actions = []

    actions.append(
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch))
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch),
            launch_arguments={'namespace': 'robot0', 'use_sim_time': 'false'}.items()
        )
    )

    actions.append(
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', '/robot0/robot_description', '-entity', 'robot0',
                       '-x', '0.0', '-y', '0.0', '-z', '0.1'],
            output='screen'
        )
    )

    actions.append(make_cylinder_node('obs_izq',  obs_x,  gap / 2.0))
    actions.append(make_cylinder_node('obs_der',  obs_x, -gap / 2.0))

    actions.append(
        Node(
            package='control_nodes',
            executable='navigate_individual_pva',
            name='navigate_individual_pva',
            namespace='robot0',
            parameters=[{
                'goal_x':      goal_x,
                'goal_y':      goal_y,
                'd_safe':      0.3,
                'd_influence': 2.5,
                'xi':          1.0,
                'v_max':       0.5,
                'w_max':       0.5,
                'use_sim_time': False
            }],
            output='screen'
        )
    )

    actions.append(
        Node(
            package='simulador',
            executable='states_plotter_v2',
            name='states_plotter_v2',
            parameters=[{'save_dir': 'figures_corredor'}],
            output='screen'
        )
    )

    actions.append(
        Node(
            package='simulador',
            executable='pva_plotter',
            name='pva_plotter',
            parameters=[{'save_dir': 'figures_corredor'}],
            output='screen'
        )
    )

    return actions


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'goal_x',
            default_value='8.0',
            description='Coordenada x del objetivo'
        ),
        DeclareLaunchArgument(
            'goal_y',
            default_value='0.0',
            description='Coordenada y del objetivo'
        ),
        DeclareLaunchArgument(
            'gap',
            default_value='2.0',
            description='Separación total entre centros de cilindros (m)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
