import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from multi_robot_interfaces.msg import RobotState
from multi_robot_interfaces.msg import PVAConstraints

from control_nodes.algorithms.consensus_lider import ConsensusLeader
from control_nodes.algorithms.pva import PVA

import math


class ConsensoLider(Node):

    def __init__(self):

        super().__init__('consenso_lider')

        self.robot_id = self.get_namespace().strip('/')

        # -----------------------------
        # Parámetros
        # -----------------------------

        self.declare_parameter('leader_id', 'robot0')
        self.declare_parameter('vmax', 1.0)
        self.declare_parameter('wmax', 1.0)
        self.declare_parameter('k1', 0.8)
        self.declare_parameter('k2', 1.0)

        self.leader_id = str(self.get_parameter('leader_id').value)
        vmax = float(self.get_parameter('vmax').value)
        wmax = float(self.get_parameter('wmax').value)
        k1   = float(self.get_parameter('k1').value)
        k2   = float(self.get_parameter('k2').value)

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
        # Algoritmo consenso con líder
        # -----------------------------

        self.algorithm = ConsensusLeader(k1=k1, k2=k2, vmax=vmax, wmax=wmax)

        # -----------------------------
        # PVA
        # -----------------------------

        self.pva = PVA(
            d_safe=0.85,
            d_influence=3.0,
            xi=1.0,
            rp=0.25,
            v_max=vmax,
            w_max=wmax
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

        self.neighbor_sub = self.create_subscription(
            RobotState,
            'neighbors_rx',
            self.neighbor_callback,
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
            f"{self.robot_id} consenso líder iniciado. Líder: {self.leader_id}"
        )

    # -----------------------------------------------------

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

        state = RobotState()
        state.robot_id = self.robot_id
        state.x = self.x
        state.y = self.y
        state.theta = self.theta

        self.state_pub.publish(state)
        self.plot_pub.publish(state)

    # -----------------------------------------------------

    def scan_callback(self, msg):

        self.ranges = list(msg.ranges)

    # -----------------------------------------------------

    def neighbor_callback(self, msg):

        self.neighbors[msg.robot_id] = (msg.x, msg.y, msg.theta)

    # -----------------------------------------------------

    def publish_zero(self):

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    # -----------------------------------------------------

    def control_loop(self):

        # El líder no actúa
        if self.robot_id == self.leader_id:
            self.publish_zero()
            return

        # Vecinos sin el líder
        neighbors = [
            (x, y)
            for rid, (x, y, _) in self.neighbors.items()
            if rid != self.leader_id
        ]

        # Estado del líder si está disponible
        leader_state = None
        if self.leader_id in self.neighbors:
            xL, yL, _ = self.neighbors[self.leader_id]
            leader_state = (xL, yL)

        # Sin vecinos ni líder — no actuar
        if not neighbors and leader_state is None:
            self.publish_zero()
            return

        # -----------------------------
        # CONSENSO CON LÍDER
        # -----------------------------

        v_goal, w_goal = self.algorithm.compute(
            (self.x, self.y, self.theta),
            neighbors,
            leader_state
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

    node = ConsensoLider()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()