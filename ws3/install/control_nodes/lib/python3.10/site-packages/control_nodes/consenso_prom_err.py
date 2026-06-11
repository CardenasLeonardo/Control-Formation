import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from multi_robot_interfaces.msg import RobotState
from multi_robot_interfaces.msg import PVAConstraints

from control_nodes.algorithms.consensus_prom_err import ConsensusPromErr
from control_nodes.algorithms.pva import PVA

import math


class ConsensoPromErr(Node):

    def __init__(self):

        super().__init__('consenso_prom_err')

        self.robot_id = self.get_namespace().strip('/')

        # -----------------------------
        # Esperar a todos los robots
        # -----------------------------

        self.declare_parameter('n_robots_expected', 0)
        self.n_robots_expected = self.get_parameter('n_robots_expected').value
        self.known_robots = set()

        # -----------------------------
        # Estado propio
        # -----------------------------

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # -----------------------------
        # Estados vecinos — ya filtrados por AIRE
        # -----------------------------

        self.neighbors = {}      # { neighbor_id: (x, y, theta) }

        # -----------------------------
        # LiDAR
        # -----------------------------

        self.ranges = []
        self.constraints = []

        # -----------------------------
        # Algoritmo consenso
        # -----------------------------

        self.algorithm = ConsensusPromErr()

        # -----------------------------
        # PVA
        # -----------------------------

        self.pva = PVA(
            d_safe=0.85,
            d_influence=3.0,
            xi=1.0,
            rp=0.25,
            v_max=1.0,
            w_max=1.0
        )

        # -----------------------------
        # Subscribers
        # -----------------------------

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Vecinos filtrados — tópico propio dentro del namespace
        self.neighbor_sub = self.create_subscription(
            RobotState,
            'neighbors_rx',
            self.neighbor_callback,
            10
        )

        # Conteo global de robots para saber cuándo arrancar
        self.global_state_sub = self.create_subscription(
            RobotState,
            '/robot_states_tx',
            self.global_state_callback,
            10
        )

        # -----------------------------
        # Publishers
        # -----------------------------

        self.state_pub = self.create_publisher(
            RobotState,
            '/robot_states_tx',
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.pva_pub = self.create_publisher(
            PVAConstraints,
            '/pva_constraints',
            10
        )

        self.plot_pub = self.create_publisher(
            RobotState,
            '/robot_states_plot',
            10
        )

        # -----------------------------
        # Timer
        # -----------------------------

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f"{self.robot_id} consenso iniciado — "
            f"esperando {self.n_robots_expected} robots antes de arrancar"
        )

    # -----------------------------------------------------

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

        # Publicar estado al AIRE
        state = RobotState()
        state.robot_id = self.robot_id
        state.x = self.x
        state.y = self.y
        state.theta = self.theta

        self.state_pub.publish(state)

        # Publicar estado para el plotter
        self.plot_pub.publish(state)

    # -----------------------------------------------------

    def scan_callback(self, msg):

        self.ranges = list(msg.ranges)

    # -----------------------------------------------------

    def neighbor_callback(self, msg):

        self.neighbors[msg.robot_id] = (msg.x, msg.y, msg.theta)

    # -----------------------------------------------------

    def global_state_callback(self, msg):

        self.known_robots.add(msg.robot_id)

    # -----------------------------------------------------

    def _all_ready(self):
        if self.n_robots_expected <= 0:
            return True
        return len(self.known_robots) >= self.n_robots_expected

    # -----------------------------------------------------

    def control_loop(self):

        if not self._all_ready():
            return

        neighbors = [(x, y) for x, y, _ in self.neighbors.values()]

        # -----------------------------
        # CONSENSO
        # -----------------------------

        v_goal, w_goal = self.algorithm.compute(
            (self.x, self.y, self.theta),
            neighbors
        )

        # -----------------------------
        # PVA
        # -----------------------------

        if self.ranges:

            self.constraints = self.pva.build_constraints(self.ranges)

            v_safe, w_safe = self.pva.solve_qp(
                v_goal,
                w_goal,
                self.constraints
            )

        else:

            v_safe, w_safe = v_goal, w_goal
            self.constraints = []

        # -----------------------------
        # PUBLICAR PVA
        # -----------------------------

        pva_msg = PVAConstraints()

        pva_msg.robot_id = self.robot_id

        pva_msg.a = [float(c[0]) for c in self.constraints]
        pva_msg.b = [float(c[1]) for c in self.constraints]
        pva_msg.c = [float(c[2]) for c in self.constraints]

        pva_msg.v_goal = float(v_goal)
        pva_msg.w_goal = float(w_goal)

        pva_msg.v_star = float(v_safe)
        pva_msg.w_star = float(w_safe)

        self.pva_pub.publish(pva_msg)

        # -----------------------------
        # PUBLICAR VELOCIDAD
        # -----------------------------

        twist = Twist()
        twist.linear.x = float(v_safe)
        twist.angular.z = float(w_safe)

        self.cmd_pub.publish(twist)


def main(args=None):

    rclpy.init(args=args)

    node = ConsensoPromErr()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()