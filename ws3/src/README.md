> Instrucciones de compilación/instalación: ver el [README](../../README.md) en
> la raíz del repo. Esto de aquí son comandos sueltos útiles para debug manual.

# Terminal 1: Lanzar una simulación (paquete real: `simulador`; el archivo
# `spawn_multi_robot.launch.py` de abajo es solo ilustrativo — usa uno real
# de ws3/src/simulador/launch/, p. ej. demo_pva.launch.py o consenso_prom.launch.py)
ros2 launch simulador spawn_multi_robot.launch.py n_robots:=3

# Terminal teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot0/cmd_vel
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot1/cmd_vel
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot2/cmd_vel

# Terminal pose
ros2 topic echo /robot0/odom --field pose.pose
ros2 topic echo /robot1/odom --field pose.pose
ros2 topic echo /robot2/odom --field pose.pose

# Terminal laser
ros2 topic echo /robot0/scan
ros2 topic echo /robot0/scan --flow-style


ros2 topic echo /robot1/scan
ros2 topic echo /robot1/scan --flow-style

ros2 topic echo /robot2/scan
ros2 topic echo /robot2/scan --flow-style
