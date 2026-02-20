import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import numpy as np


class HMIVIsual(Node):
    def __init__(self):
        super().__init__('hmi_visual')

        # Ventana GUI
        self.root = tk.Tk()
        self.root.title("HMI Visual - Multi Robot")

        # Selector de robot
        tk.Label(self.root, text="Robot ID:").pack()
        self.robot_selector = ttk.Combobox(
            self.root, values=[f"robot{i}" for i in range(5)]
        )
        self.robot_selector.current(0)
        self.robot_selector.pack()

        tk.Button(self.root, text="Cargar robot", command=self.load_robot).pack()

        # Figuras
        self.fig = Figure(figsize=(6, 3), dpi=100)

        # Radar
        self.ax_radar = self.fig.add_subplot(121, projection='polar')
        self.ax_radar.set_title("LIDAR Radar")
        self.ax_radar.set_ylim(0, 5.0)

        # Espacio de velocidades
        self.ax_vel = self.fig.add_subplot(122)
        self.ax_vel.set_title("v–w Space")
        self.ax_vel.set_xlim(-1.0, 1.0)
        self.ax_vel.set_ylim(-1.0, 1.0)
        self.vel_point, = self.ax_vel.plot(0, 0, 'ro')

        # Integrar en Tk
        self.canvas = FigureCanvasTkAgg(self.fig, self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack()

        # Variables del robot activo
        self.current_ns = "robot0"
        self.scan_ranges = []
        self.vel_v = 0.0
        self.vel_w = 0.0
        self.last_v = 0.0
        self.last_w = 0.0
        self.last_update_time = self.get_clock().now()



        # Timer de actualización gráfica
        self.root.after(50, self.update_graphics)

        # ROS timers
        self.create_subscription(LaserScan,
                                 "/robot0/scan",
                                 self.scan_callback,
                                 10)
        self.create_subscription(Odometry,
                                 "/robot0/odom",
                                 self.odom_callback,
                                 10)


    def load_robot(self):
        """Cambia el robot al que se está suscrito."""
        new_ns = self.robot_selector.get()
        self.current_ns = new_ns

        self.get_logger().info(f"Cambiando visualización al robot: {new_ns}")

        # Remover subs previos
        self.scan_ranges = []
        self.vel_v = 0
        self.vel_w = 0

        # Suscribir a los nuevos tópicos
        self.create_subscription(
            LaserScan, f"/{new_ns}/scan", self.scan_callback, 10
        )
        self.create_subscription(
            Odometry, f"/{new_ns}/odom", self.odom_callback, 10
        )

    # -------------------- Callbacks ROS -----------------------

    def scan_callback(self, msg: LaserScan):
        self.scan_ranges = np.array(msg.ranges)

    def odom_callback(self, msg):
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        # Detectar valores válidos vs ruido
        now = self.get_clock().now()

        # Si (v,w)==(0,0) pero pasó muy poco tiempo → mantener los últimos válidos
        if abs(v) < 0.0001 and abs(w) < 0.0001:
            dt = (now - self.last_update_time).nanoseconds / 1e6  # ms
            if dt < 150:
                # Mantener último valor (NO mover el punto al origen)
                self.vel_v = self.last_v
                self.vel_w = self.last_w
                return

        # Si aquí llegamos, el valor es válido
        self.vel_v = v
        self.vel_w = w

        # Guardar último valor y tiempo
        self.last_v = v
        self.last_w = w
        self.last_update_time = now


    # -------------------- Actualización visual ----------------

    def update_graphics(self):
        # ----- RADAR -----
        self.ax_radar.clear()
        self.ax_radar.set_title("LIDAR Radar")

        if len(self.scan_ranges) > 0:
            angles = np.linspace(0, 2*np.pi, len(self.scan_ranges))
            self.ax_radar.plot(angles, self.scan_ranges, linewidth=1)

        # ----- VELOCITY SPACE -----
        # SOLO mover el punto, NO limpiar
        self.vel_point.set_xdata(self.vel_v)
        self.vel_point.set_ydata(self.vel_w)

        # Redibujar
        self.canvas.draw()

        self.root.after(50, self.update_graphics)


def main(args=None):
    rclpy.init(args=args)
    node = HMIVIsual()

    # Loop híbrido para Tk + ROS
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        node.root.update_idletasks()
        node.root.update()


if __name__ == "__main__":
    main()
