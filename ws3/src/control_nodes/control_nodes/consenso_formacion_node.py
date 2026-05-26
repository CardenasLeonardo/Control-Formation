import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from multi_robot_interfaces.msg import RobotState

from control_nodes.algorithms.consensus_formation import ConsensusFormation, compute_offsets
from control_nodes.algorithms.control_law import PolarControlLaw
from control_nodes.algorithms.pva import PVA

import math


class ConsensoFormacion(Node):

    def __init__(self):

        super().__init__('consenso_formacion')

        self.robot_id = self.get_namespace().strip('/')

        self.declare_parameter('leader_id', 'robot0')
        self.declare_parameter('n_robots',  5)
        self.declare_parameter('angle_v',   0.785398)
        self.declare_parameter('d',         1.5)
        self.declare_parameter('beta',      2.0)
        self.declare_parameter('k1',        1.0)
        self.declare_parameter('k2',        1.5)
        self.declare_parameter('vmax',      0.8)
        self.declare_parameter('wmax',      0.8)

        self.leader_id   = self.get_parameter('leader_id').value
        self.n_robots    = self.get_parameter('n_robots').value
        self.angle_v     = self.get_parameter('angle_v').value
        self.d           = self.get_parameter('d').value
        beta             = self.get_parameter('beta').value
        k1               = self.get_parameter('k1').value
        k2               = self.get_parameter('k2').value
        self.vmax        = self.get_parameter('vmax').value
        self.wmax        = self.get_parameter('wmax').value

        self.n_followers = self.n_robots - 1

        try:
            self.follower_index = int(self.robot_id.replace('robot', ''))
        except ValueError:
            self.follower_index = 1

        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        self.neighbors      = {}    # { robot_id: (x, y, theta) }
        self.theta_L_smooth = None  # low-pass del heading del líder

        # Pipeline: consenso → ley de control → PVA (saturación)
        self.consensus    = ConsensusFormation(beta=beta)
        self.control_law  = PolarControlLaw(k1=k1, k2=k2)
        self.pva          = PVA(v_max=self.vmax, w_max=self.wmax)

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        self.neighbor_sub = self.create_subscription(
            RobotState, 'neighbors_rx', self.neighbor_callback, 10
        )

        self.state_pub = self.create_publisher(RobotState, '/robot_states_tx', 10)
        self.plot_pub  = self.create_publisher(RobotState, '/robot_states_plot', 10)
        self.cmd_pub   = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f"{self.robot_id} formación iniciada | líder: {self.leader_id} | "
            f"idx: {self.follower_index} | angle_v: {math.degrees(self.angle_v):.1f}° | "
            f"d: {self.d} m | beta: {beta} | k1: {k1} | k2: {k2}"
        )

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

    def neighbor_callback(self, msg):
        self.neighbors[msg.robot_id] = (msg.x, msg.y, msg.theta)

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):

        # Estado del líder con filtro de paso bajo sobre theta_L
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
            self.publish_zero()
            return

        # Offsets de todos los seguidores según orientación del líder
        offsets_all = compute_offsets(
            self.n_followers, theta_L, self.angle_v, self.d
        )
        my_offset = offsets_all.get(self.follower_index, (0.0, 0.0))

        # Vecinos con sus offsets (excluye líder)
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

        # 1. Consenso → error polar (a, alpha)
        a, alpha = self.consensus.compute(
            (self.x, self.y, self.theta),
            neighbors,
            leader_state,
            my_offset,
            neighbor_offsets
        )

        # 2. Ley de control → velocidades de referencia (sin saturar)
        v_ref, w_ref = self.control_law.compute(a, alpha)

        # 3. PVA → velocidades finales capadas (sin obstáculos: constraints=[])
        v, w, _ = self.pva.compute(v_ref, w_ref, [], a, alpha)

        twist = Twist()
        twist.linear.x  = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ConsensoFormacion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
