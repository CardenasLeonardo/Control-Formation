# Control-Formation

Simulación multi-robot en ROS2 + Gazebo: estructura virtual, consenso, ley de
control no holonómica y evasión de obstáculos mediante el Polígono de
Velocidades Admisibles (PVA).

Todo el código vive en el workspace de colcon `ws3/`.

## 1. Requisitos del sistema

- **Ubuntu 22.04** (Jammy)
- **ROS2 Humble** (`ros-humble-desktop` recomendado, incluye Gazebo classic vía `gazebo_ros`)
- **colcon** y **rosdep**

Si el agente arranca en una imagen sin ROS2 instalado:

```bash
# ROS2 Humble (ver guía oficial si la imagen no tiene el repo de ROS agregado)
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep

# Inicializar rosdep (solo la primera vez en la máquina)
sudo rosdep init
rosdep update
```

## 2. Dependencias del proyecto

### 2.1 Paquetes ROS2 (vía rosdep)

Cada paquete declara sus dependencias en su `package.xml`. En vez de instalarlas
a mano, usa `rosdep` desde la raíz del workspace:

```bash
cd ws3
rosdep install --from-paths src --ignore-src -r -y
```

Esto cubre, entre otros: `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`,
`std_msgs`, `launch`, `launch_ros`, `gazebo_ros`, `rosidl_default_generators`
(para el paquete de mensajes `multi_robot_interfaces`).

### 2.2 Librerías de Python (pip)

Los nodos de `simulador` y `control_nodes` usan además:

```bash
pip3 install numpy scipy matplotlib imageio
```

`opencv-python` y `cv_bridge` normalmente ya vienen con `ros-humble-desktop`;
si falta alguno:

```bash
sudo apt install -y ros-humble-cv-bridge python3-opencv
```

## 3. Compilar el workspace

```bash
cd ws3
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` es opcional pero recomendado: los cambios en archivos
Python (nodos, launch files) se reflejan sin recompilar. Solo hace falta
recompilar (`colcon build --packages-select <paquete>`) cuando cambian
mensajes (`multi_robot_interfaces`) o archivos no-Python.

Para recompilar un solo paquete tras editar código:

```bash
colcon build --packages-select simulador     # o control_nodes, etc.
source install/setup.bash
```

## 4. Correr una simulación

Con el workspace compilado y `source install/setup.bash` hecho en cada
terminal nueva:

```bash
ros2 launch simulador <archivo>.launch.py
```

Los launch files viven en `ws3/src/simulador/launch/` (por ejemplo
`demo_pva.launch.py`, `consenso_prom.launch.py`, `consenso_formacion.launch.py`,
`vs_formacion.launch.py`, `vs_formacion_noble.launch.py`).

## 5. Estructura del workspace

```
ws3/src/
├── articubot_one/            # Descripción del robot (URDF/xacro) y su robot_state_publisher
├── multi_robot_interfaces/   # Mensajes custom (p. ej. PVAConstraints, RobotState)
├── control_nodes/            # Nodos de control: consenso, ley de control, PVA, deadlock
│   └── control_nodes/algorithms/   # Algoritmos puros (sin ROS): pva.py, deadlock.py, trayectoria_vs.py, ...
└── simulador/                 # Nodos de simulación/visualización + launch files
    ├── simulador/             # vs_node, states_plotter_v2, gif_recorder, camera_zoom, ...
    ├── launch/
    ├── worlds/
    └── models/
```

## 6. Notas

- `ws3/build/`, `ws3/install/`, `ws3/log/` son generados por `colcon build` y
  están en `.gitignore` — nunca se versionan, se regeneran localmente.
- `documentacion/` (presentaciones, tesis, figuras) está fuera del repo de
  git (también en `.gitignore`); vive solo en disco local.
