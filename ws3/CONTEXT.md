# 🗂️ CONTEXT — Control de Formación de Robots Móviles Mediante Consenso

> **Bitácora de contexto general del proyecto**  
> Proyecto: `Control-Formation/ws3`  
> Ubicación: `/home/calafaker/Control-Formation/ws3`  
> Fecha de creación: 2026-05-25

---

## 📋 Tabla de Contenido

1. Resumen del Proyecto
2. Árbol del Proyecto
3. Algoritmos Principales
4. Arquitectura ROS 2
5. Nodos
6. Tópicos y Mensajes
7. Archivos de Lanzamiento
8. Sistema de Simulación
9. Sistema AIRE
10. PVA Algorithm
11. Gestión de Experimentos
12. Artículo IEEE
13. Comandos Útiles

---


## 📌 Resumen del Proyecto

Sistema de **control de formación** para múltiples robots móviles (**differential-drive**) usando algoritmos de **consenso** con evasión de obstáculos mediante **PVA (Projected Velocity Algorithm)**. Todo implementado sobre **ROS 2 Humble** con simulación en **Gazebo**.

**Objetivo:** Lograr que un grupo de robots mantenga una formación deseada (en **V**, líder-seguidor, o por promedio de error) mientras navegan evitando obstáculos.

### Stack Tecnológico

| Componente | Tecnología |
|---|---|
| Framework robótico | ROS 2 Humble |
| Simulador | Gazebo |
| Lenguaje | Python 3 |
| Optimización | SciPy (SLSQP) |
| Cálculos | NumPy |
| Descripción robots | URDF/Xacro |
| Paper | LaTeX (IEEE style) |

### Paquetes ROS 2

| Paquete | Ruta | Propósito |
|---|---|---|
| `control_nodes` | `src/control_nodes` | Algoritmos de control y nodos ROS |
| `multi_robot_interfaces` | `src/multi_robot_interfaces` | Mensajes personalizados |
| `simulador` | `src/simulador` | Lanzadores, plotters, nodo AIRE |
| `articubot_one` | `src/articubot_one` | Descripción URDF del robot |

## 🌳 Árbol del Proyecto

```
/home/calafaker/Control-Formation/
├── ws3/                                    # ROS 2 workspace
│   ├── CONTEXT.md                          # <<< ESTE ARCHIVO
│   ├── Makefile                            # Build automation
│   ├── run_experiment.sh                   # Experiment runner
│   ├── docs/
│   │   └── paper_ieee/                    # IEEE paper (LaTeX)
│   │       ├── main.tex
│   │       └── references.bib
│   ├── experiments/                        # Experiment results
│   └── src/
│       ├── articubot_one/
│       │   ├── urdf/robot.urdf.xacro       # Robot description
│       │   └── launch/multi_robot.launch.py # Multi-robot spawner
│       ├── control_nodes/
│       │   ├── control_nodes/
│       │   │   ├── algorithms/
│       │   │   │   ├── consensus_formation.py  # V-formation consensus
│       │   │   │   ├── consensus_lider.py      # Leader-follower consensus
│       │   │   │   ├── consensus_prom_err.py   # Average error consensus
│       │   │   │   ├── control_law.py          # Polar control law
│       │   │   │   └── pva.py                  # Projected Velocity Alg.
│       │   │   ├── consenso_formacion_node.py
│       │   │   ├── consenso_lider_node.py
│       │   │   ├── consenso_prom_err.py
│       │   │   └── navigate_waypoints_pva.py
│       ├── multi_robot_interfaces/
│       │   └── msg/
│       │       ├── PVAConstraints.msg
│       │       └── RobotState.msg
│       └── simulador/
│           ├── simulador/
│           │   ├── AIRE.py                 # Neighbor routing
│           │   └── states_plotter_v2.py
│           └── launch/
│               ├── consenso_lider.launch.py
│               ├── consenso_formacion.launch.py
│               └── consenso_prom.launch.py
```

## 🧠 Algoritmos Principales

### 1. `consensus_formation.py` — Consenso con Formación en V

**Archivo:** `src/control_nodes/control_nodes/algorithms/consensus_formation.py`

Implementa la ley de control para mantener una **formación en V**. Usa **consenso relativo**: el error se calcula sobre posiciones desplazadas por el offset de formación.

**Función `compute_offsets(n_followers, theta_leader, angle_v, d)`:**
- Divide seguidores en brazo izquierdo y derecho
- Cada brazo se rota según `theta_leader +/- angle_v`
- `d` es la separación entre robots en el brazo

**Clase `ConsensusFormation` / `compute()`:**
- Parámetros: `beta` (ganancia líder), `k1` (vel lineal), `k2` (vel angular), `vmax`, `wmax`
- Ley: `ẋ_i = -Σ_j [(x_i - r_i) - (x_j - r_j)] - β (x_i - x_0 - r_i)`
- Luego aplica **ley de control polar**: `v = k1·a·cos(α)`, `w = k2·α + k1·sin(α)·cos(α)`

### 2. `consensus_lider.py` — Consenso Líder-Seguidor

**Clase `ConsensusLeader`:** Parámetros `k1=0.8`, `k2=1.0`, `k_leader=10.0`

- Calcula promedio de errores con vecinos
- Añade término del líder con peso `k_leader`
- Sin offsets de formación (los robots siguen al líder directamente)

### 3. `consensus_prom_err.py` — Consenso por Promedio de Error

**Clase `ConsensusPromErr`:** Parámetros `k1=0.8`, `k2=1.0`

- Sin líder: todos convergen al **centroide** de sus vecinos
- Útil para consenso sin líder / agrupamiento

### 4. `control_law.py` — Navegación a Punto

**Clase `NavControlador`:** Parámetros `k1=0.5`, `k2=1.0`

- Calcula `v, w` para navegar hacia `(xr, yr)` usando ley polar
- Usado por: `navigate_individual.py`, `navigate_individual_pva.py`, `navigate_waypoints_pva.py`

### 5. PVA — Projected Velocity Algorithm

**Archivo:** `src/control_nodes/control_nodes/algorithms/pva.py`

Ver sección completa sobre PVA más abajo.

## 🏗️ Arquitectura ROS 2

### Diagrama de Comunicación

```
                    ┌──────────────────────┐
                    │      Nodo AIRE        │
                    │  (Enrutador vecinos)   │
                    └──────┬───────────────┘
                           │ /{robot_id}/neighbors_rx
                           │
        ┌──────────────────┼──────────────────┐
        ▼                  ▼                  ▼
┌───────────────┐  ┌───────────────┐  ┌───────────────┐
│   Robot 0     │  │   Robot 1     │  │   Robot 2     │
│  (LIDER)      │  │ (SEGUIDOR)    │  │ (SEGUIDOR)    │
└───────┬───────┘  └───────┬───────┘  └───────┬───────┘
        │                  │                  │
        └──────────────────┼──────────────────┘
                           ▼
               ┌───────────────────────┐
               │   /robot_states_plot   │
               │   (Plotters)           │
               └───────────────────────┘
```

### Flujo de Datos (Ciclo de Control 10 Hz)

1. Gazebo simula y publica `/odom` y `/scan`
2. Cada nodo recibe su odometría (`odom_callback`)
3. Publica su estado en `/robot_states_tx`
4. AIRE recibe estados, calcula vecinos dentro del radio R, publica en `/{id}/neighbors_rx`
5. Cada nodo ejecuta `control_loop`:
   a. Obtiene estados de vecinos
   b. Calcula `(v_goal, w_goal)` por consenso
   c. Aplica PVA para evasión de obstáculos
   d. Publica `Twist` en `cmd_vel`
6. Gazebo aplica `cmd_vel` y vuelve al paso 1

---

## 🤖 Nodos ROS 2

### 1. `consenso_lider_node.py` — Líder-Seguidor + PVA
- **Clase:** `ConsensoLider`
- **Algoritmo:** `ConsensusLeader` + `PVA`
- **Sub:** `odom`, `scan`, `neighbors_rx`
- **Pub:** `cmd_vel`, `robot_states_tx`, `pva_constraints`, `robot_states_plot`
- **Params:** `leader_id`, `vmax`, `wmax`, `k1`, `k2`, `k_leader`
- **Lógica:** Líder publica 0; seguidores calculan consenso + PVA

### 2. `consenso_formacion_node.py` — Formación en V
- **Clase:** `ConsensoFormacion`
- **Algoritmo:** `ConsensusFormation` (con `compute_offsets`)
- **Sub:** `odom`, `neighbors_rx`
- **Pub:** `cmd_vel`, `robot_states_tx`, `robot_states_plot`
- **Params:** `leader_id`, `n_robots`, `angle_v`, `d`, `beta`, `k1`, `k2`

### 3. `consenso_prom_err.py` — Promedio de Error + PVA
- **Clase:** `ConsensoPromErr`
- **Algoritmo:** `ConsensusPromErr` + `PVA`
- **Sub:** `odom`, `scan`, `neighbors_rx`, `/robot_states_tx`
- **Pub:** `cmd_vel`, `robot_states_tx`, `pva_constraints`, `robot_states_plot`
- **Especial:** Espera a `n_robots_expected` antes de empezar

### 4. `navigate_individual.py` — Navegación Simple
- **Clase:** `NavigateIndividual`
- **Algoritmo:** `NavControlador` (control_law.py)
- **Params:** `goal_x`, `goal_y` | Tolerancia: 0.05m

### 5. `navigate_individual_pva.py` — Navegación + PVA (QP only)
- **Algoritmo:** `NavControlador` + `PVA.solve_qp()`
- **Params:** `goal_x`, `goal_y`, `d_safe=0.85`, `d_influence=3.0`
- **Nota:** Usa `solve_qp()` sin máquina de estados

### 6. `navigate_waypoints_pva.py` — Waypoints + PVA + Boundary Following
- **Algoritmo:** `NavControlador` + `PVA.compute()` (máquina de estados completa)
- **Params:** `waypoints` (lista), `vmax`, `wmax` | Tolerancia: 0.15m
- **Lógica:** Ciclo de waypoints; usa `reaching_goal` → `boundary_following`

### 7. `AIRE.py` — Enrutador de Vecinos
- **Clase:** `Aire`
- **Params:** `neighbor_radius` (R=3.0m)
- **Sub:** `/robot_states_tx` | **Pub:** `/{robot_id}/neighbors_rx` (dinámico)
- **Función:** Topología de proximidad (disk graph) — solo vecinos dentro de R

---

## 📡 Tópicos y Mensajes

### Tópicos Principales

| Tópico | Tipo | Propósito |
|---|---|---|
| `/{ns}/odom` | `Odometry` | Odometría del robot |
| `/{ns}/scan` | `LaserScan` | Lecturas del LIDAR (360°) |
| `/{ns}/cmd_vel` | `Twist` | Velocidades de salida (v, w) |
| `/robot_states_tx` | `RobotState` | Broadcast de estados (todos publican aquí) |
| `/{ns}/neighbors_rx` | `RobotState` | Vecinos filtrados por AIRE |
| `/robot_states_plot` | `RobotState` | Datos para plotters |
| `/{ns}/pva_constraints` | `PVAConstraints` | Restricciones PVA |

### Mensajes Personalizados

**RobotState.msg:**
```
string robot_id
float64 x
float64 y
float64 theta
```

**PVAConstraints.msg:**
```
string robot_id
float64[] a    # Coeficientes A (cos de ángulo de rayo)
float64[] b    # Coeficientes B (rp * sin(ángulo))
float64[] c    # Términos C (restricción)
float64 v_goal # Velocidad lineal deseada
float64 w_goal # Velocidad angular deseada
float64 v_star # Velocidad lineal segura (post-PVA)
float64 w_star # Velocidad angular segura (post-PVA)
```

---

## 🚀 Archivos de Lanzamiento (Launch)

| Launch File | Algoritmo | Obstáculos |
|---|---|---|
| `consenso_lider.launch.py` | Líder-Seguidor + Waypoints | No |
| `consenso_formacion.launch.py` | Formación en V | Sí (cilindro rojo) |
| `consenso_prom.launch.py` | Promedio de Error | No |
| `consenso_prom_3x3.launch.py` | Promedio Error (3x3=9 robots) | No |
| `corredor_pva.launch.py` | PVA en corredor | Paredes |
| `navegacion_individual.launch.py` | Navegación + PVA | Sí |
| `navegar_individual.launch.py` | Navegación simple | No |

### Parámetros Comunes en Launch

- `n_robots`: Número de robots (default: 3)
- `neighbor_radius`: Radio de comunicación (default: 3.0m)
- `leader_id`: ID del líder (default: "robot0")
- `waypoints`: Lista de waypoints [x1,y1,x2,y2,...]
- `t_final`: Duración simulación (default: 30s)
- `angle_v`: Ángulo apertura V (default: π/4)
- `d`: Separación entre robots (default: 0.75m)

---

## 🎮 Sistema de Simulación (Gazebo)

- Mundo vacío, robots spawn en línea separados 1.5m en X
- `multi_robot.launch.py` + `rsp.launch.py` manejan el spawning
- Cada robot tiene namespace único (`robot0`, `robot1`, ...)
- Robots: chasis diferencial + 2 ruedas + LIDAR 360° (1° resolución)
- Modelo cinemático: `ẋ=v·cos(θ), ẏ=v·sin(θ), θ̇=w`

---

## 🌐 Sistema AIRE (Enrutamiento de Vecinos)

Nodo central que implementa **topología de proximidad** (disk graph):

1. Escucha `/robot_states_tx`
2. Mantiene diccionario `{robot_id: (x, y, theta)}`
3. Para cada robot, calcula vecinos dentro del radio `R`
4. Publica en `/{robot_id}/neighbors_rx` el estado de cada vecino

No es comunicación broadcast — solo robots dentro de R metros intercambian información.

---

## 🛡️ PVA — Projected Velocity Algorithm (Detallado)

**Archivo:** `src/control_nodes/control_nodes/algorithms/pva.py`

PVA es un **optimizador en tiempo real** que toma velocidades de referencia `(v_goal, w_goal)` y las proyecta sobre un conjunto de restricciones lineales del LIDAR para obtener velocidades seguras `(v_safe, w_safe)`.

### Parámetros

| Parámetro | Default | Descripción |
|---|---|---|
| `d_safe` | 0.3m | Distancia mínima de seguridad |
| `d_influence` | 1.0m | Distancia de influencia del obstáculo |
| `xi` | 1.0 | Ganancia de restricción |
| `rp` | 0.25 | Radio del robot |
| `v_max, w_max` | 0.5 | Límites de velocidad |
| `deadlock_tol` | 0.05 | Tolerancia de deadlock |

### Construcción de Restricciones

Para cada rayo LIDAR con distancia `d` dentro del radio de influencia:

```
ratio = (d - d_safe) / (d_influence - d_safe)
C = xi * ratio
A = cos(theta_i)
B = rp * sin(theta_i)
# Restricción: C - A*v - B*w >= 0
```

### QP (Quadratic Programming)

Minimiza: `J(v,w) = 0.5*[(v - v_goal)^2 + (w - w_goal)^2]`
Sujeto a: restricciones lineales + límites de velocidad
Método: `scipy.optimize.minimize(method='SLSQP')`

### Máquina de Estados (usada por navigate_waypoints_pva.py)

```
reaching_goal ──→(deadlock?)──→ boundary_following
     ↑                                  │
     └──────────(V < V_block?)──────────┘
```

- **Reaching goal:** Aplica QP para seguir el objetivo
- **Boundary following:** Sigue contorno del obstáculo
  - Identifica restricción más restrictiva (C más grande)
  - Dirección: `ccw` (B>0) o `cw` (B<0)
- **Retorno:** Cuando V(z) < V_block (obstáculo rodeado)
  - Función de Lyapunov: `V(z) = 0.5*a^2 + 0.5*alpha^2`

---

## 📊 Gestión de Experimentos

### Makefile

| Comando | Descripción |
|---|---|
| `make build` | Compila `control_nodes` y `simulador` |
| `make exp` | Ejecuta experimento (default settings) |
| `make exp NOMBRE=test` | Ejecuta experimento "test" |
| `make paper` | Compila paper LaTeX |
| `make last` | Copia figuras del último experimento al paper |
| `make use-exp DIR=...` | Copia figuras de experimento específico al paper |

### Script `run_experiment.sh`

```
./run_experiment.sh <nombre> [launch_file] [args...]
```

Flujo:
1. Crea `experiments/<timestamp>_<nombre>/figures/`
2. Genera `config.yaml` (metadatos)
3. Crea `notes.md` (documentación)
4. Exporta `EXP_SAVE_DIR` para plotters
5. Ejecuta `ros2 launch`

---

## 📐 Diagramas SDL (Specification and Description Language)

**Ubicación:** `docs/sdl/`

Diagramas formales según el estándar ITU-T Z.100 generados con **Graphviz DOT**.

| Diagrama | Archivo DOT | PDF | Descripción |
|---|---|---|---|
| System SDL | `system_sdl.dot` | `system_sdl.pdf` | Bloques del sistema (Gazebo, Robots, AIRE, Plotter) y canales de señal (tópicos ROS 2) |
| Robot Control Process | `process_robot.dot` | `process_robot.pdf` | Proceso del nodo robot: callbacks (odom, scan, neighbors) + control loop 10 Hz con consenso y PVA |
| AIRE Process | `process_aire.dot` | `process_aire.pdf` | Proceso del enrutador de vecinos: recepción de estados, filtrado por distancia R, publicación a cada robot |
| PVA State Machine | `process_pva.dot` | `process_pva.pdf` | Máquina de estados del PVA: reaching_goal (QP) y boundary_following con función de Lyapunov |

### Compilar diagramas
```bash
cd /home/calafaker/Control-Formation/ws3
for f in docs/sdl/*.dot; do
  dot -Tpdf "$f" -o "${f%.dot}.pdf"
done
```

---

## 📄 Artículo IEEE

**Título:** "Control de Formación de Robots Móviles Mediante Consenso"
**Ubicación:** `docs/paper_ieee/main.tex`

Control de formación para múltiples robots móviles usando consenso. Analiza líder-seguidor y promedio de error, implementados en ROS 2 + Gazebo. Los seguidores convergen a la trayectoria del líder con restricciones de velocidad por PVA.

**Keywords:** Formation control, consensus, mobile robots, PVA, ROS 2

---

## 💻 Comandos Útiles

### Build
```bash
cd /home/calafaker/Control-Formation/ws3
source /opt/ros/humble/setup.bash
make build
```

### Experimentos
```bash
make exp NOMBRE=test LAUNCH=consenso_lider.launch.py ARGS="n_robots:=5"
make exp NOMBRE=formacion LAUNCH=consenso_formacion.launch.py ARGS="n_robots:=4"
```

### Monitoreo en Vivo
```bash
ros2 topic echo /robot0/odom --field pose.pose
ros2 topic echo /robot0/scan --flow-style
ros2 topic echo /robot0/pva_constraints
ros2 topic echo /robot_states_tx
```

### Teleoperación
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot0/cmd_vel
```

---

## 📝 Notas Importantes

- **ROS 2 Humble** es la distribución utilizada
- El workspace está en `/home/calafaker/Control-Formation/ws3`
- Hay un backup en `Control-Formation-backup/`
- Versión corrupta en `Control-Formation-corrupt/` (no usar)
- Algoritmos en `src/control_nodes/control_nodes/algorithms/`
- Launch files de experimentos en `src/simulador/launch/`
- PVA con boundary following = versión más avanzada
- Nodos de consenso básicos usan solo `solve_qp()` (sin boundary following)

---

> *Bitácora generada por Codebuff (Buffy) — 2026-05-25*  
> *Mantenimiento: Actualizar este archivo cuando se añadan/modifiquen algoritmos, tópicos o nodos.*


### Historial del Proyecto

- Proyecto desarrollado con asistencia de **Claude AI**
- Bitácora de contexto generada por **Codebuff (Buffy)** — 2026-05-25
- Configuración de IA: `.claude/` en `ws3/.claude/` (contiene configuraciones para agentes)
- El proyecto está bajo **control de versiones Git** (repositorio en `/home/calafaker/Control-Formation/`)
- **Nota para futuros agentes:** Este archivo CONTEXT.md contiene el contexto completo del proyecto.
  Al iniciar una nueva sesión, leer este archivo primero para obtener contexto completo.
