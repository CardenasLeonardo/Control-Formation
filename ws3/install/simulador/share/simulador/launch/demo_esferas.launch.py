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


# ------------------------------------------------------------------
# Parámetros globales del demo
# ------------------------------------------------------------------

N_ROBOTS  = 5
ANGLE_V   = math.pi / 4
D         = 1.0

# Heading inicial = norte (π/2), apuntando al primer waypoint
THETA_0   = math.pi / 2.0

# Posición del punto virtual (centro de la V de esferas)
SPHERE_X  = 3.0
SPHERE_Y  = 0.0

# Posición del robot0 (centro de la V de robots) — detrás de las esferas
ROBOT_OFFSET = 2.5   # metros atrás (en dirección opuesta al heading)
ROBOT_X   = SPHERE_X - math.cos(THETA_0) * ROBOT_OFFSET   # = 3.0
ROBOT_Y   = SPHERE_Y - math.sin(THETA_0) * ROBOT_OFFSET   # = -2.5

# Trayectoria compartida (esferas y robot0 siguen los mismos waypoints)
WAYPOINTS = [3.0, 4.0,   0.0, 7.0,   -4.0, 5.0,
             -4.0, 0.0,  -1.0, -3.0,   3.0,  0.0]

# Timing
T_START       = 3.0    # arranque de los primeros spawns
T_ROBOT_STEP  = 1.5    # gap entre robots
T_AIRE        = T_START + N_ROBOTS * T_ROBOT_STEP + 1.0   # 11.5 s
T_CONTROL     = T_AIRE + 1.0                               # 12.5 s
START_DELAY   = T_CONTROL - T_START + 2.0                 # 11.5 s — sphere espera esto

# Grabación de GIF (cámara cenital) — se detiene sola al llegar a la última
# meta de WAYPOINTS (robot0), no por tiempo fijo
RECORD_GIF    = True
CAMERA_HEIGHT = 16.0
GIF_FPS       = 10.0
STOP_DELAY    = 2.0    # s de margen tras alcanzar la última meta


# ------------------------------------------------------------------
# Cálculo de posiciones iniciales en V
# ------------------------------------------------------------------

def _v_positions(n, cx, cy, theta, angle_v, d):
    """Posiciones de los n robots en formación V centrada en (cx, cy)."""
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
# Launch setup (robots)
# ------------------------------------------------------------------

def launch_setup(context, *args, **kwargs):

    save_dir = LaunchConfiguration('save_dir').perform(context)

    articubot_pkg = get_package_share_directory('articubot_one')
    rsp_launch    = os.path.join(articubot_pkg, 'launch', 'rsp.launch.py')
    camera_sdf    = os.path.join(
        get_package_share_directory('simulador'), 'models', 'overhead_camera.sdf'
    )

    robot_pos = _v_positions(N_ROBOTS, ROBOT_X, ROBOT_Y, THETA_0, ANGLE_V, D)

    actions = []

    if RECORD_GIF:
        actions.append(TimerAction(period=T_START, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', camera_sdf,
                    '-entity', 'overhead_camera',
                    '-x', str(SPHERE_X), '-y', str(SPHERE_Y), '-z', str(CAMERA_HEIGHT),
                    '-P', '1.5708',
                ],
                output='screen'
            )
        ]))

        actions.append(TimerAction(period=T_AIRE, actions=[
            Node(
                package='simulador',
                executable='gif_recorder',
                name='gif_recorder',
                parameters=[{
                    'save_dir':    save_dir,
                    'gif_name':    'esferas.gif',
                    'fps':         GIF_FPS,
                    'stop_mode':   'goal',
                    'goal_topic':  '/robot0/final_goal_reached',
                    'stop_delay':  STOP_DELAY,
                    't_final':     300.0,
                    'auto_shutdown': True,
                }],
                output='screen'
            )
        ]))

    # --- Spawn de cada robot (staggered) ---
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

    # --- Nodos de control (todos arrancan juntos en T_CONTROL) ---
    actions.append(TimerAction(period=T_CONTROL, actions=[
        Node(
            package='control_nodes',
            executable='navigate_waypoints_pva',
            name='navigate_waypoints',
            namespace='robot0',
            parameters=[{
                'waypoints': WAYPOINTS,
                'vmax': 0.5,
                'wmax': 0.6,
            }],
            output='screen'
        )
    ]))

    for i in range(1, N_ROBOTS):
        ns = f'robot{i}'
        actions.append(TimerAction(period=T_CONTROL, actions=[
            Node(
                package='control_nodes',
                executable='consenso_node',
                name='consenso',
                namespace=ns,
                parameters=[{
                    'consensus_type': 'formacion',
                    'leader_id':      'robot0',
                    'n_robots':       N_ROBOTS,
                    'angle_v':        ANGLE_V,
                    'd':              D,
                    'beta':           2.0,
                    'k1':             1.0,
                    'k2':             1.5,
                    'vmax':           0.5,
                    'wmax':           0.6,
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

        DeclareLaunchArgument('save_dir',
            default_value='figuras/esferas',
            description='Directorio donde se guardan gifs/ y frames/'),

        # --- Gazebo con plugin de estado ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world_path}.items()
        ),

        # --- Estructura virtual (esferas) ---
        # Spawea en cuanto /spawn_entity está listo, luego espera START_DELAY antes de moverse
        TimerAction(period=T_START, actions=[
            Node(
                package='simulador',
                executable='sphere_demo',
                name='sphere_demo',
                parameters=[{
                    'n_robots':    N_ROBOTS,
                    'angle_v':     ANGLE_V,
                    'd':           D,
                    'spawn_x':     SPHERE_X,
                    'spawn_y':     SPHERE_Y,
                    'waypoints':   WAYPOINTS,
                    'v_max':       0.25,
                    'arrive_dist': 0.30,
                    'start_delay': START_DELAY,   # espera a que todo spawee
                }],
                output='screen',
            )
        ]),

        # --- Robots ---
        OpaqueFunction(function=launch_setup),

    ])
