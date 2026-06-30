import os
import math

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ------------------------------------------------------------------
# Parámetros globales
# ------------------------------------------------------------------

N_ROBOTS = 5
ANGLE_V  = math.pi / 4
D        = 1.0

THETA_0  = math.pi / 2.0   # heading inicial: norte

# Centroide de la estructura virtual
VS_X = 3.0
VS_Y = 0.0

# Robots spawnean 2.5 m detrás de la estructura virtual
ROBOT_OFFSET = 2.5
ROBOT_X = VS_X - math.cos(THETA_0) * ROBOT_OFFSET   # 3.0
ROBOT_Y = VS_Y - math.sin(THETA_0) * ROBOT_OFFSET   # -2.5

# waypoints[0] es la posición inicial del centroide; la curva pasa por todos ellos
WAYPOINTS = [3.0, 0.0,   3.0, 4.0,   0.0, 7.0,  -4.0, 5.0,
             -4.0, 0.0,  -1.0, -3.0,  3.0, 0.0]

# Timing
T_START      = 3.0
T_ROBOT_STEP = 1.5
T_AIRE       = T_START + N_ROBOTS * T_ROBOT_STEP + 1.0   # 11.5 s
T_CONTROL    = T_AIRE + 1.0                               # 12.5 s
START_DELAY  = T_CONTROL - T_START + 2.0                 # 11.5 s


# ------------------------------------------------------------------
# Posiciones iniciales en V
# ------------------------------------------------------------------

def _v_positions(n, cx, cy, theta, angle_v, d):
    n_followers = n - 1
    n_left  = n_followers // 2
    n_right = n_followers - n_left

    def rot(th, vx, vy):
        c, s = math.cos(th), math.sin(th)
        return c*vx - s*vy, s*vx + c*vy

    pos = {0: (cx, cy)}
    for k in range(1, n_left + 1):
        ox, oy = rot(theta + angle_v, -k*d, 0.0)
        pos[k] = (round(cx + ox, 4), round(cy + oy, 4))
    for k in range(1, n_right + 1):
        ox, oy = rot(theta - angle_v, -k*d, 0.0)
        pos[n_left + k] = (round(cx + ox, 4), round(cy + oy, 4))
    return pos


# ------------------------------------------------------------------
# Spawns de robots
# ------------------------------------------------------------------

def launch_setup(context, *args, **kwargs):

    articubot_pkg = get_package_share_directory('articubot_one')
    rsp_launch    = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')

    robot_pos = _v_positions(N_ROBOTS, ROBOT_X, ROBOT_Y, THETA_0, ANGLE_V, D)

    actions = []

    # --- Spawn staggered de cada robot ---
    for i in range(N_ROBOTS):
        ns     = f'robot{i}'
        x0, y0 = robot_pos[i]
        t_rsp  = T_START + i * T_ROBOT_STEP

        actions.append(TimerAction(period=t_rsp, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rsp_launch),
                launch_arguments={'namespace': ns, 'use_sim_time': 'false'}.items()
            )
        ]))

        actions.append(TimerAction(period=t_rsp + 0.5, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', f'/{ns}/robot_description',
                    '-entity', ns,
                    '-x', str(x0), '-y', str(y0), '-z', '0.1',
                    '-Y', str(THETA_0),
                ],
                output='screen'
            )
        ]))

    # --- AIRE ---
    actions.append(TimerAction(period=T_AIRE, actions=[
        Node(
            package='simulador',
            executable='aire',
            name='aire',
            parameters=[{'neighbor_radius': 15.0}],
            output='screen'
        )
    ]))

    # --- Nodo de consenso para cada robot (todos siguen la estructura virtual) ---
    for i in range(N_ROBOTS):
        ns = f'robot{i}'
        actions.append(TimerAction(period=T_CONTROL, actions=[
            Node(
                package='control_nodes',
                executable='consenso_node',
                name='consenso',
                namespace=ns,
                parameters=[{
                    'consensus_type': 'formacion',
                    'n_robots':       N_ROBOTS,
                    'angle_v':        ANGLE_V,
                    'd':              D,
                    'beta':           2.0,
                    'k1':             1.0,
                    'k2':             1.5,
                    'vmax':           0.5,
                    'wmax':           0.6,
                    'd_safe':         0.35,
                    'd_influence':    0.60,
                }],
                output='screen'
            )
        ]))

    return actions


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

    return LaunchDescription([

        # --- Gazebo con plugin de estado ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world_path}.items()
        ),

        # --- Estructura virtual: esferas + trayectoria + publicación ---
        TimerAction(period=T_START, actions=[
            Node(
                package='simulador',
                executable='vs_node',
                name='vs_node',
                parameters=[{
                    'n_robots':    N_ROBOTS,
                    'angle_v':     ANGLE_V,
                    'd':           D,
                    'waypoints':   WAYPOINTS,
                    'v_max':       0.25,
                    'start_delay': START_DELAY,
                }],
                output='screen',
            )
        ]),

        # --- Robots ---
        OpaqueFunction(function=launch_setup),

    ])
