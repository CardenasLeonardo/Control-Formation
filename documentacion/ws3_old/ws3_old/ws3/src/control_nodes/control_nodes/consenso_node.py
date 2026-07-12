import rclpy
from rclpy.node import Node
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from multi_robot_interfaces.msg import RobotState, PVAConstraints

from control_nodes.algorithms.consensus_prom_err  import ConsensusPromErr
from control_nodes.algorithms.consensus_lider     import ConsensusLeader
from control_nodes.algorithms.consensus_formation import ConsensusFormation, compute_offsets
from control_nodes.algorithms.control_law         import PolarControlLaw
from control_nodes.algorithms.pva                 import PVA


class ConsensoNode(Node):

    def __init__(self):
        super().__init__('consenso')

        self.robot_id = self.get_namespace().strip('/')

        # Parámetros comunes
        self.declare_parameter('consensus_type',    'prom_err')
        self.declare_parameter('k1',                0.8)
        self.declare_parameter('k2',                1.0)
        self.declare_parameter('vmax',              1.0)
        self.declare_parameter('wmax',              1.0)
        self.declare_parameter('d_safe',            0.85)
        self.declare_parameter('d_influence',       3.0)
        # prom_err
        self.declare_parameter('n_robots_expected', 0)
        # lider / formacion
        self.declare_parameter('leader_id',         'robot0')
        self.declare_parameter('k_leader',          1.0)
        # formacion
        self.declare_parameter('n_robots',          5)
        self.declare_parameter('angle_v',           0.785398)
        self.declare_parameter('d',                 1.5)
        self.declare_parameter('beta',              2.0)

        self.consensus_type = self.get_parameter('consensus_type').value
        k1          = self.get_parameter('k1').value
        k2          = self.get_parameter('k2').value
        vmax        = self.get_parameter('vmax').value
        wmax        = self.get_parameter('wmax').value
        d_safe      = self.get_parameter('d_safe').value
        d_influence = self.get_parameter('d_influence').value

        if self.consensus_type == 'prom_err':
            self.n_robots_expected = self.get_parameter('n_robots_expected').value
            self.known_robots      = set()
            self.consensus         = ConsensusPromErr()

        elif self.consensus_type == 'lider':
            self.leader_id = self.get_parameter('leader_id').value
            self.consensus = ConsensusLeader(
                k_leader=self.get_parameter('k_leader').value
            )

        elif self.consensus_type == 'formacion':
            self.leader_id      = self.get_parameter('leader_id').value
            self.n_robots       = self.get_parameter('n_robots').value
            self.angle_v        = self.get_parameter('angle_v').value
            self.d              = self.get_parameter('d').value
            self.n_followers    = self.n_robots - 1
            self.theta_L_smooth = None
            try:
                self.follower_index = int(self.robot_id.replace('robot', ''))
            except ValueError:
                self.follower_index = 1
            self.consensus = ConsensusFormation(
                beta=self.get_parameter('beta').value
            )

        else:
            raise ValueError(f"consensus_type desconocido: '{self.consensus_type}'")

        self.control_law = PolarControlLaw(k1=k1, k2=k2)
        self.pva         = PVA(
            d_safe=d_safe, d_influence=d_influence, xi=1.0, rp=0.25,
            v_max=vmax, w_max=wmax
        )

        self.x           = 0.0
        self.y           = 0.0
        self.theta       = 0.0
        self.neighbors   = {}
        self.ranges      = []
        self.constraints = []

        self.create_subscription(Odometry,   'odom',         self.odom_callback,     10)
        self.create_subscription(LaserScan,  'scan',         self.scan_callback,     10)
        self.create_subscription(RobotState, 'neighbors_rx', self.neighbor_callback, 10)

        if self.consensus_type == 'prom_err':
            self.create_subscription(
                RobotState, '/robot_states_tx', self.global_state_callback, 10
            )

        self.state_pub = self.create_publisher(RobotState,     '/robot_states_tx',   10)
        self.cmd_pub   = self.create_publisher(Twist,          'cmd_vel',            10)
        self.pva_pub   = self.create_publisher(PVAConstraints, '/pva_constraints',   10)
        self.plot_pub  = self.create_publisher(RobotState,     '/robot_states_plot', 10)

        self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f"{self.robot_id} | consenso: {self.consensus_type}"
        )

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
        self.state_pub.publish(state)
        self.plot_pub.publish(state)

    def scan_callback(self, msg):
        self.ranges = list(msg.ranges)

    def neighbor_callback(self, msg):
        self.neighbors[msg.robot_id] = (msg.x, msg.y, msg.theta)

    def global_state_callback(self, msg):
        self.known_robots.add(msg.robot_id)

    # ------------------------------------------------------------------
    # Pipeline común: ley de control → PVA → publicar
    # ------------------------------------------------------------------

    def _apply_pipeline(self, a, alpha):
        v_ref, w_ref = self.control_law.compute(a, alpha)

        self.constraints = self.pva.build_constraints(self.ranges) if self.ranges else []
        v, w, _ = self.pva.compute(v_ref, w_ref, self.constraints, a, alpha)

        pva_msg = PVAConstraints()
        pva_msg.robot_id = self.robot_id
        pva_msg.a      = [float(c[0]) for c in self.constraints]
        pva_msg.b      = [float(c[1]) for c in self.constraints]
        pva_msg.c      = [float(c[2]) for c in self.constraints]
        pva_msg.v_goal = float(v_ref)
        pva_msg.w_goal = float(w_ref)
        pva_msg.v_star = float(v)
        pva_msg.w_star = float(w)
        self.pva_pub.publish(pva_msg)

        twist = Twist()
        twist.linear.x  = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    # ------------------------------------------------------------------
    # Control loops por modo
    # ------------------------------------------------------------------

    def _loop_prom_err(self):
        if self.n_robots_expected > 0 and len(self.known_robots) < self.n_robots_expected:
            return

        neighbors = [(x, y) for x, y, _ in self.neighbors.values()]
        a, alpha  = self.consensus.compute((self.x, self.y, self.theta), neighbors)
        self._apply_pipeline(a, alpha)

    def _loop_lider(self):
        if self.robot_id == self.leader_id:
            self._stop()
            return

        neighbors    = [(x, y) for rid, (x, y, _) in self.neighbors.items() if rid != self.leader_id]
        leader_state = None
        if self.leader_id in self.neighbors:
            xL, yL, _ = self.neighbors[self.leader_id]
            leader_state = (xL, yL)

        if not neighbors and leader_state is None:
            self._stop()
            return

        a, alpha = self.consensus.compute((self.x, self.y, self.theta), neighbors, leader_state)
        self._apply_pipeline(a, alpha)

    def _loop_formacion(self):
        leader_state = None
        theta_L      = 0.0

        if self.leader_id in self.neighbors:
            xL, yL, theta_L_raw = self.neighbors[self.leader_id]
            leader_state = (xL, yL)

            if self.theta_L_smooth is None:
                self.theta_L_smooth = theta_L_raw
            else:
                diff = theta_L_raw - self.theta_L_smooth
                if diff >  math.pi: diff -= 2 * math.pi
                if diff < -math.pi: diff += 2 * math.pi
                self.theta_L_smooth += 0.05 * diff

            theta_L = self.theta_L_smooth

        if leader_state is None and not self.neighbors:
            self._stop()
            return

        offsets_all = compute_offsets(self.n_followers, theta_L, self.angle_v, self.d)
        my_offset   = offsets_all.get(self.follower_index, (0.0, 0.0))

        neighbors        = []
        neighbor_offsets = []
        for rid, (xj, yj, _) in self.neighbors.items():
            if rid == self.leader_id:
                continue
            try:
                j = int(rid.replace('robot', ''))
            except ValueError:
                continue
            neighbors.append((xj, yj))
            neighbor_offsets.append(offsets_all.get(j, (0.0, 0.0)))

        a, alpha = self.consensus.compute(
            (self.x, self.y, self.theta),
            neighbors, leader_state, my_offset, neighbor_offsets
        )
        self._apply_pipeline(a, alpha)

    # ------------------------------------------------------------------

    def control_loop(self):
        if self.consensus_type == 'prom_err':
            self._loop_prom_err()
        elif self.consensus_type == 'lider':
            self._loop_lider()
        elif self.consensus_type == 'formacion':
            self._loop_formacion()


def main(args=None):
    rclpy.init(args=args)
    node = ConsensoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
