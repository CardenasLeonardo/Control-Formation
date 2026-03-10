import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from multi_robot_interfaces.msg import RobotState

from control_nodes.algorithms.control_law import NavControlador
from control_nodes.algorithms.pva import PVA
from control_nodes.algorithms.pva_plotter import PVAPlotter

import math


class NavigateIndividual(Node):

    def __init__(self):
        super().__init__('navigate_individual')

        self.robot_id = self.get_namespace().strip('/')

        # --------------------------------------------------
        # OBJETIVO
        # --------------------------------------------------

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        self.tolerance = 0.05

        # --------------------------------------------------
        # CONTROLADOR
        # --------------------------------------------------

        self.controller = NavControlador()

        # --------------------------------------------------
        # PVA
        # --------------------------------------------------

        self.pva = PVA(
            d_safe=0.85,
            d_influence=3.0,
            xi=1.0,
            rp=0.25,
            v_max=1.0,
            w_max=1.0
        )

        # --------------------------------------------------
        # ESTADO ROBOT
        # --------------------------------------------------

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v = 0.0
        self.w = 0.0

        self.ranges = []

        # constraints actuales
        self.constraints = []

        # --------------------------------------------------
        # GRAFICADOR
        # --------------------------------------------------

        self.plotter = PVAPlotter(vmax=1.0, wmax=1.0)
        self.plot_counter = 0

        # --------------------------------------------------
        # SUBSCRIBERS
        # --------------------------------------------------

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

        # --------------------------------------------------
        # PUBLISHERS
        # --------------------------------------------------

        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.state_pub = self.create_publisher(
            RobotState,
            '/robot_states_tx',
            10
        )

        # --------------------------------------------------
        # TIMER
        # --------------------------------------------------

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"{self.robot_id} navegando hacia ({self.goal_x}, {self.goal_y})"
        )

    # --------------------------------------------------
    # ODOM
    # --------------------------------------------------

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)

        self.theta = math.atan2(siny_cosp, cosy_cosp)

        state = RobotState()
        state.robot_id = self.robot_id
        state.x = self.x
        state.y = self.y
        state.theta = self.theta

        self.state_pub.publish(state)

        self.controller.x = self.x
        self.controller.y = self.y
        self.controller.theta = self.theta

    # --------------------------------------------------
    # LIDAR
    # --------------------------------------------------

    def scan_callback(self, msg):

        self.ranges = list(msg.ranges)

    # --------------------------------------------------
    # CONTROL LOOP
    # --------------------------------------------------

    def control_loop(self):

        distance_to_goal = math.sqrt(
            (self.goal_x - self.x)**2 +
            (self.goal_y - self.y)**2
        )

        if distance_to_goal < self.tolerance:

            stop = Twist()
            self.cmd_pub.publish(stop)

            return

        # --------------------------------------------------
        # CONTROL NOMINAL
        # --------------------------------------------------

        v_goal, w_goal = self.controller.ley_control(
            self.goal_x,
            self.goal_y
        )

        # --------------------------------------------------
        # DEBUG: usar solo 10 rayos
        # --------------------------------------------------

        if self.ranges:

            # construir todas las constraints
            all_constraints = self.pva.build_constraints(self.ranges)

            # índices cada 36°
            debug_indices = [i*36 for i in range(10)]

            # seleccionar solo esas constraints
            self.constraints = [
                all_constraints[i]
                for i in debug_indices
                if i < len(all_constraints)
            ]

            print("\n--- DEBUG CONSTRAINTS ---")

            for i,(A,B,C) in enumerate(self.constraints):

                angle = math.degrees(math.atan2(B/0.25, A))

                print(
                    f"{i}: angle≈{angle:.1f}°  "
                    f"A={A:.3f}  B={B:.3f}  C={C:.3f}"
                )

            v_safe, w_safe = self.pva.solve_qp(
                v_goal,
                w_goal,
                self.constraints
            )

        else:

            v_safe, w_safe = v_goal, w_goal
            self.constraints = []

        self.v = v_safe
        self.w = w_safe

        # --------------------------------------------------
        # GRAFICAR
        # --------------------------------------------------

        self.plot_counter += 1

        if self.plot_counter % 5 == 0:

            self.plotter.plot_constraints(
                self.constraints,
                goal=(v_goal, w_goal),
                safe=(v_safe, w_safe)
            )

        # --------------------------------------------------
        # PUBLICAR
        # --------------------------------------------------

        twist = Twist()
        twist.linear.x = float(v_safe)
        twist.angular.z = float(w_safe)

        self.cmd_pub.publish(twist)


def main(args=None):

    rclpy.init(args=args)

    node = NavigateIndividual()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()