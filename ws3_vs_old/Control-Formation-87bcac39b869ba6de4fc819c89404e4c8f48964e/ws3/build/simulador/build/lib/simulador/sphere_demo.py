import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, SetEntityState


# ------------------------------------------------------------------
# Colores y generador de SDF
# ------------------------------------------------------------------

BLUE  = '0.0 0.4 1.0 1.0'
GREEN = '0.0 0.85 0.2 1.0'

def _make_sdf(name, color, radius=0.18, transparency=0.65):
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


# ------------------------------------------------------------------
# Geometría de la formación en V
# ------------------------------------------------------------------

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
# Nodo
# ------------------------------------------------------------------

class SphereDemo(Node):

    DT     = 0.1
    RADIUS = 0.18
    Z      = RADIUS + 0.02

    def __init__(self):
        super().__init__('sphere_demo')

        # --- Parámetros de la formación ---
        self.declare_parameter('n_robots',    5)
        self.declare_parameter('angle_v',     0.7854)
        self.declare_parameter('d',           1.0)

        # --- Parámetros de la trayectoria ---
        self.declare_parameter('spawn_x',     3.0)
        self.declare_parameter('spawn_y',     0.0)
        self.declare_parameter('waypoints',   [3.0, 4.0, 0.0, 7.0,
                                               -4.0, 5.0, -4.0, 0.0,
                                               -1.0, -3.0, 3.0, 0.0])
        self.declare_parameter('v_max',       0.25)
        self.declare_parameter('arrive_dist', 0.30)

        # Tiempo de espera (s) después del spawn antes de empezar a moverse.
        # Debe ser suficiente para que todos los robots spaween.
        self.declare_parameter('start_delay', 12.0)

        self.n           = self.get_parameter('n_robots').value
        self.av          = self.get_parameter('angle_v').value
        self.d           = self.get_parameter('d').value
        self.v_max       = self.get_parameter('v_max').value
        self.arrive      = self.get_parameter('arrive_dist').value
        self.start_delay = self.get_parameter('start_delay').value

        wp_flat    = list(self.get_parameter('waypoints').value)
        self._wps  = [(wp_flat[i], wp_flat[i+1]) for i in range(0, len(wp_flat), 2)]
        self._wp_i = 0

        # --- Posición y heading inicial ---
        self.x_v = self.get_parameter('spawn_x').value
        self.y_v = self.get_parameter('spawn_y').value
        if self._wps:
            dx = self._wps[0][0] - self.x_v
            dy = self._wps[0][1] - self.y_v
            self.theta_v = math.atan2(dy, dx) if math.hypot(dx, dy) > 0.01 else 0.0
        else:
            self.theta_v = 0.0

        self._spawned      = False
        self._ticks_waited = 0          # ticks esperados después del spawn

        # --- Clientes de Gazebo ---
        self.spawn_cli = self.create_client(SpawnEntity,    '/spawn_entity')
        self.move_cli  = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        self.create_timer(self.DT, self.tick)
        self.get_logger().info(
            f'sphere_demo | n={self.n}  v_max={self.v_max}  '
            f'start_delay={self.start_delay}s  wps={len(self._wps)}'
        )

    # --- Spawn de todas las esferas ---
    def _spawn_all(self):
        offs = _compute_offsets(self.n, self.theta_v, self.av, self.d)
        for i in range(self.n):
            name     = f'vs_{i}'
            color    = BLUE if i == 0 else GREEN
            ox, oy   = offs.get(i, (0.0, 0.0))
            req      = SpawnEntity.Request()
            req.name                       = name
            req.xml                        = _make_sdf(name, color, self.RADIUS)
            req.initial_pose.position.x    = self.x_v + ox
            req.initial_pose.position.y    = self.y_v + oy
            req.initial_pose.position.z    = self.Z
            req.initial_pose.orientation.w = 1.0
            req.reference_frame            = 'world'
            self.spawn_cli.call_async(req)
        self.get_logger().info(
            f'Esferas spawneadas. Esperando {self.start_delay}s antes de moverse...'
        )

    # --- Avance hacia el waypoint actual ---
    def _advance(self):
        if self._wp_i >= len(self._wps):
            return

        tx, ty = self._wps[self._wp_i]
        dx     = tx - self.x_v
        dy     = ty - self.y_v
        dist   = math.hypot(dx, dy)

        if dist < self.arrive:
            self._wp_i += 1
            self.get_logger().info(f'Waypoint {self._wp_i}/{len(self._wps)}')
            return

        step          = min(self.v_max * self.DT, dist)
        self.x_v     += step * dx / dist
        self.y_v     += step * dy / dist
        self.theta_v  = math.atan2(dy, dx)

    # --- Loop principal ---
    def tick(self):

        # 1. Spawn en cuanto /spawn_entity esté disponible
        if not self._spawned:
            if self.spawn_cli.service_is_ready():
                self._spawn_all()
                self._spawned = True
            return

        # 2. Esperar start_delay antes de moverse
        if self._ticks_waited * self.DT < self.start_delay:
            self._ticks_waited += 1
            return

        # 3. Movimiento (requiere el plugin de estado)
        if not self.move_cli.service_is_ready():
            return

        self._advance()

        offs = _compute_offsets(self.n, self.theta_v, self.av, self.d)
        for i, (ox, oy) in offs.items():
            req = SetEntityState.Request()
            req.state.name               = f'vs_{i}'
            req.state.pose.position.x    = self.x_v + ox
            req.state.pose.position.y    = self.y_v + oy
            req.state.pose.position.z    = self.Z
            req.state.pose.orientation.w = 1.0
            req.state.reference_frame    = 'world'
            self.move_cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = SphereDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
