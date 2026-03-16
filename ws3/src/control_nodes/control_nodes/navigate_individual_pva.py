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
            f"{self.robot_id} navegando hacia ({self.goal_x}, {self.goal_y})"
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
            (self.goal_x - self.x) ** 2 +
            (self.goal_y - self.y) ** 2
        )

        # --------------------------------------------------
        # CONTROL NOMINAL
        # --------------------------------------------------

        if distance_to_goal < self.tolerance:
            v_goal = 0.0
            w_goal = 0.0
        else:
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

        self.v = v_safe
        self.w = w_safe

        # --------------------------------------------------
        # PUBLICAR PVA
        # --------------------------------------------------

        msg = PVAConstraints()

        msg.robot_id = self.robot_id

        msg.a = [c[0] for c in self.constraints]
        msg.b = [c[1] for c in self.constraints]
        msg.c = [c[2] for c in self.constraints]

        msg.v_goal = float(v_goal)
        msg.w_goal = float(w_goal)

        msg.v_star = float(v_safe)
        msg.w_star = float(w_safe)

        self.pva_pub.publish(msg)

        # --------------------------------------------------
        # PUBLICAR VELOCIDADES
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


if __name__ == '__main__':
    main()