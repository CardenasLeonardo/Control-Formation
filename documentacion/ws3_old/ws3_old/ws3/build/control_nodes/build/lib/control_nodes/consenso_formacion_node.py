import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from multi_robot_interfaces.msg import RobotState
from multi_robot_interfaces.msg import PVAConstraints

from control_nodes.algorithms.consensus_formation import ConsensusFormation, compute_offsets
from control_nodes.algorithms.pva import PVA

import math


class ConsensoFormacion(Node):

    def __init__(self):

        super().__init__('consenso_formacion')

        self.robot_id = self.get_namespace().strip('/')

        # -----------------------------
        # Parámetros
        # -----------------------------

        self.declare_parameter('leader_id',   'robot0')
        self.declare_parameter('n_robots',    4)
        self.declare_parameter('angle_v',     0.785398)   # 45 deg en radianes
        self.declare_parameter('d',           0.5)

        self.leader_id = str(self.get_parameter('leader_id').value)
        self.n_robots  = int(self.get_parameter('n_robots').value)
        self.angle_v   = float(self.get_parameter('angle_v').value)
        self.d         = float(self.get_parameter('d').value)

        self.n_followers = self.n_robots - 1

        # Índice del seguidor dentro de la formación (1..N-1)
        # Se asigna según el número del robot: robot1→1, robot2→2, etc.
        try:
            robot_num = int(self.robot_id.replace('robot', ''))
        except ValueError:
            robot_num = 1

        self.follower_index = robot_num  # 1-based

        # -----------------------------
        # Estado propio
        # -----------------------------

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # -----------------------------
        # Estados vecinos — filtrados por AIRE
        # -----------------------------

        self.neighbors = {}     # { neighbor_id: (x, y, theta) }

        # -----------------------------
        # LiDAR
        # -----------------------------

        self.ranges = []
        self.constraints = []

        # -----------------------------
        # Algoritmo
        # -----------------------------

        self.algorithm = ConsensusFormation()

        # -----------------------------
        # PVA
        # -----------------------------

        self.pva = PVA(
            d_safe=0.65,
            d_influence=1.5,
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
            f"{self.robot_id} consenso formación iniciado. "
            f"Líder: {self.leader_id} | "
            f"Índice en formación: {self.follower_index} | "
            f"angle_v: {math.degrees(self.angle_v):.1f}° | "
            f"d: {self.d} m"
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

        # Esperar hasta conocer al líder
        if self.leader_id not in self.neighbors:
            self.publish_zero()
            return

        # -----------------------------
        # Estado del líder
        # -----------------------------

        xL, yL, theta_L = self.neighbors[self.leader_id]

        # -----------------------------
        # Calcular offsets con orientación actual del líder
        # -----------------------------

        offsets = compute_offsets(
            self.n_followers,
            theta_L,
            self.angle_v,
            self.d
        )

        offset = offsets.get(self.follower_index, (0.0, 0.0))

        # -----------------------------
        # Vecinos (sin el líder)
        # -----------------------------

        neighbors = [
            (x, y)
            for rid, (x, y, _) in self.neighbors.items()
            if rid != self.leader_id
        ]

        # -----------------------------
        # CONSENSO CON FORMACIÓN
        # -----------------------------

        v_goal, w_goal = self.algorithm.compute(
            (self.x, self.y, self.theta),
            neighbors,
            (xL, yL),
            offset
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

    node = ConsensoFormacion()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()