import subprocess
import time
import os


# ----------------------------
# MATAR PROCESOS VIEJOS
# ----------------------------

def kill_all_ros():
    print("[SPAWNER] Matando nodos ROS antiguos...")
    os.system("pkill -f 'robot_state_publisher'")               # Cierra cualquier robot_state_publisher
    os.system("pkill -f 'spawn_entity.py'")                     # Cierra cualquier spawn_entity.py
    os.system("pkill -f 'ros2 run simulation_pkg hmi_visual'")  # Cierra HMI visual
    os.system("pkill -f 'ros2 run articubot_one'")              # Cierra cualquier nodo de articubot_one


def kill_gazebo():
    print("[SPAWNER] Matando Gazebo previo...")                 # Cierra cualquier Gazebo viejo
    os.system("pkill -9 gzserver")
    os.system("pkill -9 gzclient")
    os.system("pkill -9 ign")
    os.system("pkill -9 gz-sim")
    time.sleep(1)


# ----------------------------
# LANZAR PROCESOS NUEVOS
# ----------------------------

def launch_gazebo():                                            # Lanzar Gazebo
    print("[SPAWNER] Lanzando Gazebo...")
    return subprocess.Popen([
        "ros2", "launch", "gazebo_ros", "gazebo.launch.py"
    ])


def launch_robot_rsp(namespace):                                # Lanzar robot_state_publisher para un robot
    print(f"[SPAWNER] RSP → {namespace}")
    return subprocess.Popen([
        "ros2", "launch", "articubot_one", "rsp.launch.py",
        f"namespace:={namespace}",
        "use_sim_time:=true"
    ])


def spawn_robot(namespace, x, y):                               # Spawnear un robot en Gazebo
    print(f"[SPAWNER] Spawn → {namespace} en ({x}, {y})")
    return subprocess.Popen([
        "ros2", "run", "gazebo_ros", "spawn_entity.py",
        "-topic", f"/{namespace}/robot_description",
        "-entity", namespace,
        "-x", str(x),
        "-y", str(y),
        "-z", "0.1"
    ])


def launch_hmi_visual():                                        # Lanzar la HMI visual
    print("[SPAWNER] Lanzando HMI visual...")
    return subprocess.Popen([
        "ros2", "run", "simulation_pkg", "hmi_visual"
    ])
