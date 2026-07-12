import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, SetEntityState
from multi_robot_interfaces.msg import RobotState
from std_msgs.msg import Bool

from simulador.vs_trayectoria import TrayectoriaVS


# ------------------------------------------------------------------
# Helpers de visualización Gazebo
# ------------------------------------------------------------------

BLUE  = '0.0 0.4 1.0 1.0'
GREEN = '0.0 0.85 0.2 1.0'


def _make_sdf(name, color, radius=0.18, transparency=0.65):
    # Colisión de 1 mm: evita que Gazebo genere un bounding-box automático
    # a partir del visual (que sí sería detectado por el lidar).
    # Un cubo de 1 mm nunca es golpeado por ningún rayo en la práctica.
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <kinematic>true</kinematic>
      <visual name="visual">
        <transparency>{transparency}</transparency>
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def _compute_offsets(n_robots, theta_v, angle_v, d):
    n_followers = n_robots - 1
    n_left  = n_followers // 2
    n_right = n_followers - n_left

    def rot(theta, vx, vy):
        c, s = math.cos(theta), math.sin(theta)
        return c*vx - s*vy, s*vx + c*vy

    offs = {0: (0.0, 0.0)}
    for k in range(1, n_left + 1):
        offs[k] = rot(theta_v + angle_v, -k*d, 0.0)
    for k in range(1, n_right + 1):
        offs[n_left + k] = rot(theta_v - angle_v, -k*d, 0.0)
    return offs


# ------------------------------------------------------------------
# Nodo de estructura virtual
# ------------------------------------------------------------------

class VirtualStructureNode(Node):

    DT     = 0.1
    RADIUS = 0.18
    Z      = RADIUS + 0.02

    def __init__(self):
        super().__init__('vs_node')

        # --- Parámetros de la formación ---
        self.declare_parameter('n_robots', 5)
        self.declare_parameter('angle_v',  0.7854)
        self.declare_parameter('d',        1.0)

        # --- Parámetros de la trayectoria ---
        # waypoints[0] es la posición inicial del centroide virtual
        self.declare_parameter('waypoints',   [3.0, 0.0,  3.0, 4.0,
                                               0.0, 7.0, -4.0, 5.0,
                                               -4.0, 0.0, -1.0, -3.0,
                                               3.0, 0.0])
        self.declare_parameter('v_max',       0.25)
        self.declare_parameter('start_delay', 12.0)

        self.n           = self.get_parameter('n_robots').value
        self.av          = self.get_parameter('angle_v').value
        self.d_form      = self.get_parameter('d').value
        self.start_delay = self.get_parameter('start_delay').value

        wp_flat   = list(self.get_parameter('waypoints').value)
        waypoints = [(wp_flat[i], wp_flat[i+1]) for i in range(0, len(wp_flat), 2)]

        self.tray = TrayectoriaVS(
            waypoints = waypoints,
            v_max     = self.get_parameter('v_max').value,
        )

        self._spawned        = False
        self._ticks_waited   = 0
        self._done_published = False

        # --- Clientes Gazebo ---
        self.spawn_cli = self.create_client(SpawnEntity,    '/spawn_entity')
        self.move_cli  = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        # --- Publicador de la estructura virtual ---
        self.vs_pub   = self.create_publisher(RobotState, '/virtual_structure', 10)
        self.done_pub = self.create_publisher(Bool, 'final_goal_reached', 10)

        self.create_timer(self.DT, self.tick)
        self.get_logger().info(
            f'VirtualStructure | n={self.n}  d={self.d_form}  '
            f'start_delay={self.start_delay}s  wps={self.tray.n_waypoints}'
        )

    # --- Spawn de esferas en posiciones iniciales de la V ---
    def _spawn_all(self):
        offs = _compute_offsets(self.n, self.tray.theta, self.av, self.d_form)
        for i in range(self.n):
            name   = f'vs_{i}'
            color  = BLUE if i == 0 else GREEN
            ox, oy = offs.get(i, (0.0, 0.0))
            req = SpawnEntity.Request()
            req.name                       = name
            req.xml                        = _make_sdf(name, color, self.RADIUS)
            req.initial_pose.position.x    = self.tray.x + ox
            req.initial_pose.position.y    = self.tray.y + oy
            req.initial_pose.position.z    = self.Z
            req.initial_pose.orientation.w = 1.0
            req.reference_frame            = 'world'
            self.spawn_cli.call_async(req)
        self.get_logger().info(f'Esferas spawneadas. Esperando {self.start_delay}s...')

    # --- Mover esferas a posiciones actuales ---
    def _move_spheres(self):
        offs = _compute_offsets(self.n, self.tray.theta, self.av, self.d_form)
        for i, (ox, oy) in offs.items():
            req = SetEntityState.Request()
            req.state.name               = f'vs_{i}'
            req.state.pose.position.x    = self.tray.x + ox
            req.state.pose.position.y    = self.tray.y + oy
            req.state.pose.position.z    = self.Z
            req.state.pose.orientation.w = 1.0
            req.state.reference_frame    = 'world'
            self.move_cli.call_async(req)

    # --- Publicar centroide en /virtual_structure ---
    def _publish_state(self):
        msg = RobotState()
        msg.robot_id = 'virtual_structure'
        msg.x        = float(self.tray.x)
        msg.y        = float(self.tray.y)
        msg.theta    = float(self.tray.theta)
        self.vs_pub.publish(msg)

    # --- Loop principal ---
    def tick(self):
        # 1. Spawn en cuanto /spawn_entity esté disponible
        if not self._spawned:
            if self.spawn_cli.service_is_ready():
                self._spawn_all()
                self._spawned = True
            return

        # 2. Publicar posición inicial mientras se espera start_delay
        if self._ticks_waited * self.DT < self.start_delay:
            self._ticks_waited += 1
            self._publish_state()
            return

        # 3. Avanzar, publicar y mover esferas
        if not self.move_cli.service_is_ready():
            return

        self.tray.step(self.DT)
        self._publish_state()
        self._move_spheres()

        if self.tray.done and not self._done_published:
            self._done_published = True
            self.done_pub.publish(Bool(data=True))
            self.get_logger().info('Trayectoria de la estructura virtual completada')


def main(args=None):
    rclpy.init(args=args)
    node = VirtualStructureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
