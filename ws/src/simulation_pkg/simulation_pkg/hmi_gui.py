import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import messagebox
import threading
import time
import os
import sys

from .spawner import (
    launch_gazebo,
    launch_robot_rsp,
    spawn_robot,
    launch_hmi_visual,
    kill_all_ros,
    kill_gazebo
)


class HMIGUI(Node):
    def __init__(self):
        super().__init__("hmi_gui")

        self.sim_running = False

        gui_thread = threading.Thread(target=self.gui_loop, daemon=True)                            # Iniciar GUI en hilo separado
        gui_thread.start()


    # ---------------------------------------------------------
    # VENTANA PRINCIPAL
    # ---------------------------------------------------------
    def gui_loop(self):
        self.root = tk.Tk()
        self.root.title("HMI - Simulación Multi-Robot")

        # Cuando el usuario hace clic en la X
        self.root.protocol("WM_DELETE_WINDOW", self.quit_app)

        # Variables Tk
        self.num_robots = tk.IntVar(value=1)
        self.mode = tk.StringVar(value="auto")
        self.positions = []

        tk.Label(self.root, text="Número de robots:").pack()
        tk.Entry(self.root, textvariable=self.num_robots).pack()

        tk.Radiobutton(self.root, text="Posiciones automáticas",
                       variable=self.mode, value="auto").pack()

        tk.Radiobutton(self.root, text="Posiciones manuales",
                       variable=self.mode, value="manual").pack()

        tk.Button(self.root, text="Configurar posiciones manuales",
                  command=self.manual_positions_window).pack(pady=5)

        tk.Button(self.root, text="Iniciar simulación",
                  command=self.start_simulation).pack(pady=10)

        tk.Button(self.root, text="Salir",
                  command=self.quit_app).pack(pady=10)

        self.root.mainloop()


    # ---------------------------------------------------------
    # VENTANA PARA POSICIONES MANUALES
    # ---------------------------------------------------------
    def manual_positions_window(self):
        win = tk.Toplevel(self.root)
        win.title("Posiciones manuales")

        robots = self.num_robots.get()
        entries = []

        for i in range(robots):
            tk.Label(win, text=f"robot{i} (x y):").pack()
            e = tk.Entry(win)
            e.pack()
            entries.append(e)

        def save_positions():
            try:
                self.positions = []
                for e in entries:
                    x, y = map(float, e.get().split())
                    self.positions.append((x, y))
                win.destroy()
                messagebox.showinfo("HMI", "Posiciones guardadas.")
            except:
                messagebox.showerror("Error", "Formato inválido (use: x  y)")

        tk.Button(win, text="Guardar", command=save_positions).pack()


    # ---------------------------------------------------------
    # INICIAR SIMULACIÓN
    # ---------------------------------------------------------
    def start_simulation(self):
        if self.sim_running:
            messagebox.showwarning("HMI", "La simulación ya está corriendo.")
            return

        self.sim_running = True

        n = self.num_robots.get()

        if n <= 0:
            messagebox.showerror("Error", "Número inválido.")
            self.sim_running = False
            return

        if self.mode.get() == "auto":
            self.positions = [(i * 1.5, 0.0) for i in range(n)]
        else:
            if len(self.positions) != n:
                messagebox.showerror("Error", "Posiciones manuales incompletas.")
                self.sim_running = False
                return

        messagebox.showinfo("HMI", "Iniciando simulación...")

        threading.Thread(target=self.simulation_thread, daemon=True).start()


    # ---------------------------------------------------------
    # HILO DE SIMULACIÓN
    # ---------------------------------------------------------
    def simulation_thread(self):

        kill_all_ros()
        kill_gazebo()

        launch_gazebo()
        time.sleep(5)

        for i, (x, y) in enumerate(self.positions):
            ns = f"robot{i}"
            launch_robot_rsp(ns)
            spawn_robot(ns, x, y)

        launch_hmi_visual()

        print("=== Simulación iniciada ===")
        self.sim_running = False


    # ---------------------------------------------------------
    # CERRAR TODO (GUI + NODO + PROCESO)
    # ---------------------------------------------------------
    def quit_app(self):
        print("[HMI] Cerrando aplicación...")

        try:
            self.root.quit()
            self.root.destroy()
        except:
            pass

        kill_all_ros()
        kill_gazebo()

        rclpy.shutdown()

        #La forma correcta de finalizar Tkinter + ROS2 + threads
        os._exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = HMIGUI()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
