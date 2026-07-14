import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from multi_robot_interfaces.msg import RobotState
from multi_robot_interfaces.msg import PVAConstraints

from control_nodes.algorithms.control_law import NavControlador
from control_nodes.algorithms.pva import PVA

import math


class NavigateWaypoints(Node):

    def __init__(self):

        super().__init__('navigate_waypoints')

        self.robot_id = self.get_namespace().strip('/')

        # --------------------------------------------------
        # WAYPOINTS — lista de (x, y) a recorrer en ciclo
        # --------------------------------------------------

        self.declare_parameter('waypoints', [0.0, 0.0, 5.0, 0.0])

        raw = list(self.get_parameter('waypoints').value)

        # Convertir lista plana [x0, y0, x1, y1, ...] a lista de tuplas
        self.waypoints = [
            (raw[i], raw[i + 1])
            for i in range(0, len(raw) - 1, 2)
        ]

        if not self.waypoints:
            self.waypoints = [(0.0, 0.0)]

        self.wp_index = 0
        self.tolerance = 0.15

        self.goal_x, self.goal_y = self.waypoints[self.wp_index]

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

        self.ranges = []
        self.constraints = []

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

        self.plot_pub = self.create_publisher(
            RobotState,
            '/robot_states_plot',
            10
        )

        self.pva_pub = self.create_publisher(
            PVAConstraints,
            '/pva_constraints',
            10
        )

        # --------------------------------------------------
        # TIMER
        # --------------------------------------------------

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"{self.robot_id} navegación por waypoints iniciada. "
            f"Total waypoints: {len(self.waypoints)}. "
            f"Primer objetivo: {self.waypoints[self.wp_index]}"
        )

    # --------------------------------------------------
    # ODOM
    # --------------------------------------------------

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

        self.controller.x = self.x
        self.controller.y = self.y
        self.controller.theta = self.theta

    # --------------------------------------------------
    # LIDAR
    # --------------------------------------------------

    def scan_callback(self, msg):

        self.ranges = list(msg.ranges)

    # --------------------------------------------------
    # AVANZAR AL SIGUIENTE WAYPOINT
    # --------------------------------------------------

    def next_waypoint(self):

        self.wp_index = (self.wp_index + 1) % len(self.waypoints)
        self.goal_x, self.goal_y = self.waypoints[self.wp_index]

        self.get_logger().info(
            f"{self.robot_id} waypoint alcanzado. "
            f"Siguiente: {self.waypoints[self.wp_index]}"
        )

    # --------------------------------------------------
    # CONTROL LOOP
    # --------------------------------------------------

    def control_loop(self):

        distance_to_goal = math.sqrt(
            (self.goal_x - self.x) ** 2 +
            (self.goal_y - self.y) ** 2
        )

        # Avanzar waypoint si llegamos
        if distance_to_goal < self.tolerance:
            self.next_waypoint()

        # --------------------------------------------------
        # CONTROL NOMINAL
        # --------------------------------------------------

        v_goal, w_goal = self.controller.ley_control(
            self.goal_x,
            self.goal_y
        )

        # --------------------------------------------------
        # PVA
        # --------------------------------------------------

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

        # --------------------------------------------------
        # PUBLICAR PVA
        # --------------------------------------------------

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

        # --------------------------------------------------
        # PUBLICAR VELOCIDADES
        # --------------------------------------------------

        twist = Twist()
        twist.linear.x = float(v_safe)
        twist.angular.z = float(w_safe)

        self.cmd_pub.publish(twist)


def main(args=None):

    rclpy.init(args=args)

    node = NavigateWaypoints()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()