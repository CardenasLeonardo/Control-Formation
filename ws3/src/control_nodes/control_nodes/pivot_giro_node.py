import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, SetEntityState
from multi_robot_interfaces.msg import RobotState
from std_msgs.msg import Bool

from control_nodes.algorithms.pivot_giro import PivotGiroSecuencia

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


class PivotGiroNode(Node):
    """
    Prueba aislada del algoritmo PivotGiro: la formación en V arranca en
    un set de posiciones iniciales dado (sin waypoints ni centroide móvil)
    y ejecuta una secuencia de fases: giro -180°, giro +180°, avance en
    línea recta hacia donde queda apuntando la formación tras los giros.
    """

    DT     = 0.1
    RADIUS = 0.18
    Z      = RADIUS + 0.02

    def __init__(self):
        super().__init__('pivot_giro_node')

        self.declare_parameter('n_robots',        0)
        self.declare_parameter('initial_positions', [0.0, 0.0])
        self.declare_parameter('omega',            0.0)
        self.declare_parameter('v_max',            0.0)
        self.declare_parameter('distance',         0.0)
        self.declare_parameter('theta_0',          0.0)
        self.declare_parameter('start_delay',      0.0)

        self.n           = self.get_parameter('n_robots').value
        self.start_delay = self.get_parameter('start_delay').value
        omega            = self.get_parameter('omega').value
        v_max            = self.get_parameter('v_max').value
        distance         = self.get_parameter('distance').value
        theta_0          = self.get_parameter('theta_0').value

        pos_flat = list(self.get_parameter('initial_positions').value)
        if (self.n < 3 or self.n % 2 == 0 or omega <= 0.0 or v_max <= 0.0
                or len(pos_flat) != 2 * self.n):
            raise ValueError(
                "Faltan o son inválidos los parámetros requeridos "
                "(n_robots impar >= 3, omega > 0, v_max > 0, "
                "initial_positions de longitud 2*n_robots)"
            )

        initial_positions = [(pos_flat[i], pos_flat[i + 1]) for i in range(0, len(pos_flat), 2)]

        fases = [
            ('giro',     math.radians(-180.0)),
            ('giro',     math.radians(180.0)),
            ('avanzar',  distance),
        ]
        self.giro = PivotGiroSecuencia(
            initial_positions = initial_positions,
            fases             = fases,
            omega             = omega,
            v_max             = v_max,
            theta_0           = theta_0,
        )

        self._spawned        = False
        self._ticks_waited   = 0
        self._done_published = False

        self.spawn_cli = self.create_client(SpawnEntity,    '/spawn_entity')
        self.move_cli  = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        self.vs_pub   = self.create_publisher(RobotState, '/virtual_structure', 10)
        self.plot_pub = self.create_publisher(RobotState, '/robot_states_plot', 10)
        self.done_pub = self.create_publisher(Bool, 'final_goal_reached', 10)

        self.create_timer(self.DT, self.tick)
        self.get_logger().info(
            f'PivotGiro | n={self.n}  fases=giro(-180)->giro(+180)->avanzar'
            f'({distance}m)  omega={omega}rad/s  v_max={v_max}m/s  '
            f'start_delay={self.start_delay}s'
        )

    def _spawn_all(self):
        positions = self.giro.positions()
        for i in range(self.n):
            name  = f'vs_{i}'
            color = BLUE if i == 0 else GREEN
            px, py = positions[i]
            req = SpawnEntity.Request()
            req.name                       = name
            req.xml                        = _make_sdf(name, color, self.RADIUS)
            req.initial_pose.position.x    = px
            req.initial_pose.position.y    = py
            req.initial_pose.position.z    = self.Z
            req.initial_pose.orientation.w = 1.0
            req.reference_frame            = 'world'
            self.spawn_cli.call_async(req)
        self.get_logger().info(f'Esferas de referencia spawneadas. Esperando {self.start_delay}s...')

    def _move_spheres(self):
        positions = self.giro.positions()
        for i, (px, py) in positions.items():
            req = SetEntityState.Request()
            req.state.name               = f'vs_{i}'
            req.state.pose.position.x    = px
            req.state.pose.position.y    = py
            req.state.pose.position.z    = self.Z
            req.state.pose.orientation.w = 1.0
            req.state.reference_frame    = 'world'
            self.move_cli.call_async(req)

    def _publish_state(self):
        msg = RobotState()
        msg.robot_id = 'virtual_structure'
        msg.x        = float(self.giro.x)
        msg.y        = float(self.giro.y)
        msg.theta    = float(self.giro.theta)
        self.vs_pub.publish(msg)

        plot_msg = RobotState()
        plot_msg.robot_id = 'vs'
        plot_msg.x        = float(self.giro.x)
        plot_msg.y        = float(self.giro.y)
        plot_msg.theta    = float(self.giro.theta)
        self.plot_pub.publish(plot_msg)

    def tick(self):
        if not self._spawned:
            if self.spawn_cli.service_is_ready():
                self._spawn_all()
                self._spawned = True
            return

        if self._ticks_waited * self.DT < self.start_delay:
            self._ticks_waited += 1
            self._publish_state()
            return

        if not self.move_cli.service_is_ready():
            return

        self.giro.step(self.DT)
        self._publish_state()
        self._move_spheres()

        if self.giro.done and not self._done_published:
            self._done_published = True
            self.done_pub.publish(Bool(data=True))
            self.get_logger().info('Giro de la estructura virtual completado')


def main(args=None):
    rclpy.init(args=args)
    node = PivotGiroNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
