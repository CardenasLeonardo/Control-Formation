import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, SetEntityState
from multi_robot_interfaces.msg import RobotState
from std_msgs.msg import Bool

# ==================================================================
# >>> PUNTO DE INTERCAMBIO DE ALGORITMO <<<
#
# Este import es el ÚNICO lugar que hay que tocar para cambiar de
# estrategia. TrayectoriaVS resuelve la navegación (posición del
# centroide en el tiempo) y compute_formation_offsets resuelve la
# formación (offsets de cada seguidor respecto al centroide). El resto
# del nodo solo consume `self.tray.x/y/theta` y el dict que devuelve
# `self._offsets()` — no conoce el detalle de cómo se calculan.
#
# Para probar un algoritmo nuevo:
#   1. Escríbelo en control_nodes/algorithms/ (misma interfaz: una
#      clase con x, y, theta, step(dt), done — y/o una función
#      compute_formation_offsets(n_robots, theta_v, angle_v, d)).
#      Ver trayectoria_vs_2.py para un ejemplo de punto de partida.
#   2. Cambia el import de abajo al módulo nuevo.
# ==================================================================
from control_nodes.algorithms.trayectoria_vs import TrayectoriaVS, compute_formation_offsets

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


class TrayectoriaVSNode(Node):

    DT     = 0.1
    RADIUS = 0.18
    Z      = RADIUS + 0.02

    def __init__(self):
        super().__init__('trayectoria_vs_node')

        # --- Parámetros: SIN default real — todos deben venir del launch ---
        self.declare_parameter('n_robots',    0)
        self.declare_parameter('angle_v',     0.0)
        self.declare_parameter('d',           0.0)
        self.declare_parameter('waypoints',   [])
        self.declare_parameter('v_max',       0.0)
        self.declare_parameter('start_delay', 0.0)

        self.n           = self.get_parameter('n_robots').value
        self.av          = self.get_parameter('angle_v').value
        self.d_form      = self.get_parameter('d').value
        self.start_delay = self.get_parameter('start_delay').value
        v_max            = self.get_parameter('v_max').value

        wp_flat = list(self.get_parameter('waypoints').value)
        if self.n < 1 or self.d_form <= 0.0 or v_max <= 0.0 or len(wp_flat) < 4:
            raise ValueError(
                "Faltan parámetros requeridos (n_robots, d, v_max, waypoints) "
            )

        waypoints = [(wp_flat[i], wp_flat[i+1]) for i in range(0, len(wp_flat), 2)]
        
        self.tray = TrayectoriaVS(
            waypoints = waypoints,
            v_max     = v_max,
            
        )

        self._spawned        = False
        self._ticks_waited   = 0
        self._done_published = False

        # --- Clientes Gazebo ---
        self.spawn_cli = self.create_client(SpawnEntity,    '/spawn_entity')
        self.move_cli  = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        # --- Publicadores ---
        self.vs_pub   = self.create_publisher(RobotState, '/virtual_structure', 10)
        # Relay al plotter: se anuncia como un "robot" más para reutilizar
        # states_plotter_v2 en modo trajectory_only sin cambios.
        self.plot_pub = self.create_publisher(RobotState, '/robot_states_plot', 10)
        self.done_pub = self.create_publisher(Bool, 'final_goal_reached', 10)

        self.create_timer(self.DT, self.tick)
        self.get_logger().info(
            f'TrayectoriaVS | n={self.n}  d={self.d_form}  '
            f'start_delay={self.start_delay}s  wps={self.tray.n_waypoints}'
        )

    # --- Offsets actuales del set completo de referencias ---
    def _offsets(self):
        return compute_formation_offsets(self.n, self.tray.theta, self.av, self.d_form)

    # --- Spawn de la esfera del centroide + anclas de los seguidores ---
    def _spawn_all(self):
        offs = self._offsets()
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
        self.get_logger().info(f'Esferas de referencia spawneadas. Esperando {self.start_delay}s...')

    # --- Mover todas las esferas a las posiciones actuales ---
    def _move_spheres(self):
        offs = self._offsets()
        for i, (ox, oy) in offs.items():
            req = SetEntityState.Request()
            req.state.name               = f'vs_{i}'
            req.state.pose.position.x    = self.tray.x + ox
            req.state.pose.position.y    = self.tray.y + oy
            req.state.pose.position.z    = self.Z
            req.state.pose.orientation.w = 1.0
            req.state.reference_frame    = 'world'
            self.move_cli.call_async(req)

    # --- Publicar posición del centroide ---
    def _publish_state(self):
        msg = RobotState()
        msg.robot_id = 'virtual_structure'
        msg.x        = float(self.tray.x)
        msg.y        = float(self.tray.y)
        msg.theta    = float(self.tray.theta)
        self.vs_pub.publish(msg)

        plot_msg = RobotState()
        plot_msg.robot_id = 'vs'
        plot_msg.x        = float(self.tray.x)
        plot_msg.y        = float(self.tray.y)
        plot_msg.theta    = float(self.tray.theta)
        self.plot_pub.publish(plot_msg)

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

        # 3. Avanzar, publicar y mover las esferas
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
    node = TrayectoriaVSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
