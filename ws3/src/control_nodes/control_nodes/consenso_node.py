import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from multi_robot_interfaces.msg import RobotState, PVAConstraints

from control_nodes.algorithms.consensus_prom_err  import ConsensusPromErr
from control_nodes.algorithms.consensus_lider     import ConsensusLeader
from control_nodes.algorithms.consensus_formation import ConsensusFormation
from control_nodes.algorithms.control_law         import PolarControlLaw
from control_nodes.algorithms.pva                 import PVA


class ConsensoNode(Node):

    def __init__(self):
        super().__init__('consenso')

        self.robot_id = self.get_namespace().strip('/')

        # --- Parámetros comunes ---
        self.declare_parameter('consensus_type', 'prom_err')
        self.declare_parameter('k1',   0.8)
        self.declare_parameter('k2',   1.0)
        self.declare_parameter('vmax', 1.0)
        self.declare_parameter('wmax', 1.0)

        # --- Parámetros de formación ---
        self.declare_parameter('leader_id', 'robot0')
        self.declare_parameter('k_leader',  1.0)
        self.declare_parameter('n_robots',  5)
        self.declare_parameter('angle_v',   0.785398)
        self.declare_parameter('d',         1.5)
        self.declare_parameter('beta',      2.0)

        # --- Parámetros PVA (obstacle avoidance) ---
        self.declare_parameter('d_safe',      0.85)
        self.declare_parameter('d_influence', 3.0)
        self.declare_parameter('n_rays_used', 0)

        # --- Lectura de parámetros ---
        self.consensus_type = self.get_parameter('consensus_type').value
        k1   = self.get_parameter('k1').value
        k2   = self.get_parameter('k2').value
        vmax = self.get_parameter('vmax').value
        wmax = self.get_parameter('wmax').value

        # --- Selección de algoritmo de consenso ---
        if self.consensus_type == 'prom_err':
            self.consensus = ConsensusPromErr()

        elif self.consensus_type == 'lider':
            self.leader_id = self.get_parameter('leader_id').value
            self.k_leader  = self.get_parameter('k_leader').value
            self.consensus = ConsensusLeader()

        elif self.consensus_type == 'formacion':
            self.n_robots    = self.get_parameter('n_robots').value
            self.angle_v     = self.get_parameter('angle_v').value
            self.d           = self.get_parameter('d').value
            self.beta        = self.get_parameter('beta').value
            self.leader_id   = self.get_parameter('leader_id').value
            try:
                self.follower_index = int(self.robot_id.replace('robot', ''))
            except ValueError:
                self.follower_index = 0
            self.virtual_state = None  # (x_v, y_v, theta_v) — llega de /virtual_structure
            self.consensus = ConsensusFormation()

        else:
            raise ValueError(f"consensus_type desconocido: '{self.consensus_type}'")

        # --- Ley de control y obstacle avoidance ---
        self.control_law = PolarControlLaw(k1=k1, k2=k2, vmax=vmax, wmax=wmax)
        d_safe      = self.get_parameter('d_safe').value
        d_influence = self.get_parameter('d_influence').value
        n_rays_used = int(self.get_parameter('n_rays_used').value) or None
        self.pva    = PVA(d_safe=d_safe, d_influence=d_influence, xi=1.0,
                          rp=0.25, v_max=vmax, w_max=wmax, n_rays_used=n_rays_used)

        # --- Estado del robot ---
        self.x         = 0.0
        self.y         = 0.0
        self.theta     = 0.0
        self.neighbors = {}
        self.ranges    = []

        # --- Suscripciones ---
        self.create_subscription(Odometry,   'odom',         self.odom_callback,     10)
        self.create_subscription(RobotState, 'neighbors_rx', self.neighbor_callback, 10)
        self.create_subscription(LaserScan,  'scan',         self.scan_callback,     10)
        if self.consensus_type == 'formacion':
            self.create_subscription(RobotState, '/virtual_structure',
                                     self.virtual_callback, 10)

        # --- Publicadores ---
        self.state_pub = self.create_publisher(RobotState,     '/robot_states_tx',   10)
        self.cmd_pub   = self.create_publisher(Twist,          'cmd_vel',            10)
        self.pva_pub   = self.create_publisher(PVAConstraints, '/pva_constraints',   10)
        self.plot_pub  = self.create_publisher(RobotState,     '/robot_states_plot', 10)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"{self.robot_id} | consenso: {self.consensus_type}")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

        state = RobotState()
        state.robot_id = self.robot_id
        state.x        = self.x
        state.y        = self.y
        state.theta    = self.theta
        self.state_pub.publish(state)   # hacia AIRE
        self.plot_pub.publish(state)    # hacia plotter

    def neighbor_callback(self, msg):
        self.neighbors[msg.robot_id] = (msg.x, msg.y, msg.theta, self.get_clock().now())

    def scan_callback(self, msg):
        self.ranges = list(msg.ranges)

    def virtual_callback(self, msg):
        self.virtual_state = (msg.x, msg.y, msg.theta)

    # ------------------------------------------------------------------
    # Pipeline: consenso → ley de control → PVA → cmd_vel
    # ------------------------------------------------------------------

    _NEIGHBOR_TIMEOUT = Duration(seconds=0.5)

    def _active_neighbors(self):
        now = self.get_clock().now()
        return {
            rid: (x, y, th)
            for rid, (x, y, th, t) in self.neighbors.items()
            if (now - t) < self._NEIGHBOR_TIMEOUT
        }

    def control_loop(self):
        s         = (self.x, self.y, self.theta)
        neighbors = self._active_neighbors()

        if self.consensus_type == 'prom_err':
            result = self.consensus.step(s, neighbors)
        elif self.consensus_type == 'lider':
            result = self.consensus.step(s, neighbors,
                                         self.robot_id, self.leader_id, self.k_leader)
        elif self.consensus_type == 'formacion':
            # Sin estructura virtual publicada, el líder físico (vía AIRE) es
            # el ancla; se excluye de la suma relativa porque su atracción ya
            # entra ponderada por beta.
            virtual = self.virtual_state
            if virtual is None and self.leader_id in neighbors:
                virtual   = neighbors[self.leader_id]
                neighbors = {r: p for r, p in neighbors.items()
                             if r != self.leader_id}
            result = self.consensus.step(s, neighbors,
                                         virtual, self.follower_index,
                                         self.n_robots, self.angle_v, self.d, self.beta)

        if result is None:
            self.cmd_pub.publish(Twist())
            return

        a, alpha         = result
        v_ref, w_ref     = self.control_law.compute(a, alpha)
        v, w, constrs, *_ = self.pva.apply(v_ref, w_ref, self.ranges)
        self._pub_pva_debug(v_ref, w_ref, v, w, constrs)

        twist = Twist()
        twist.linear.x  = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)

    def _pub_pva_debug(self, v_ref, w_ref, v, w, constraints):
        msg = PVAConstraints()
        msg.robot_id = self.robot_id
        msg.a      = [float(c[0]) for c in constraints]
        msg.b      = [float(c[1]) for c in constraints]
        msg.c      = [float(c[2]) for c in constraints]
        msg.v_goal = float(v_ref)
        msg.w_goal = float(w_ref)
        msg.v_star = float(v)
        msg.w_star = float(w)
        self.pva_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConsensoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
