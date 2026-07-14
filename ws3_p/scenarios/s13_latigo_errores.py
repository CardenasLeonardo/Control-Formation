#!/usr/bin/env python3
"""
S13 — Efecto látigo: figura con errores en X y Y.

Corre la misma simulación que S01 (formación V con consenso relativo,
trayectoria con dos arcos de 90° CCW para generar efecto látigo) y
genera UNA figura combinada con tres paneles al estilo de 
states_plotter_v2._save_triple:

  1. Trayectorias (con triángulos de heading)
  2. Error en X (posición real − ancla ideal) para cada seguidor
  3. Error en Y (posición real − ancla ideal) para cada seguidor

Reemplaza la figura anterior en:
  documentacion/figuras/personales/formacion_latigo.pdf
"""
import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
import matplotlib.pyplot as plt

from algorithms.control_law         import PolarControlLaw
from algorithms.waypoints_nav       import WaypointsNav
from algorithms.consensus_formation import ConsensusFormation
from sim.engine                     import Simulation, LeaderAgent, FollowerAgent

# ── Configuración global de matplotlib (idéntica a states_plotter_v2) ──────
plt.rcParams.update({
    "font.family":      "serif",
    "font.serif":       ["Times New Roman"],
    "mathtext.fontset": "stix",
    "font.size":        11,
})

COLORS = plt.cm.tab10.colors


# ── _draw_triangles (copia exacta de states_plotter_v2) ────────────────────
def _draw_triangles(ax, x, y, theta, color, d_step=0.4, size=0.25):
    """Triángulos sólidos cada d_step metros a lo largo de la trayectoria."""
    if len(x) < 2:
        return
    cumul = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    total = cumul[-1]
    if total < d_step:
        return
    for d_target in np.arange(d_step / 2, total, d_step):
        idx = int(np.clip(np.searchsorted(cumul, d_target, side='right') - 1,
                          0, len(x) - 1))
        xi, yi, ti = x[idx], y[idx], theta[idx]
        pts = np.array([
            [ size,        0.0        ],
            [-size * 0.5,  size * 0.45],
            [-size * 0.5, -size * 0.45],
        ])
        c_t, s_t = np.cos(ti), np.sin(ti)
        R = np.array([[c_t, -s_t], [s_t, c_t]])
        rotated = (R @ pts.T).T + np.array([xi, yi])
        ax.fill(rotated[:, 0], rotated[:, 1], color=color, zorder=3, alpha=0.85)


# ── Parámetros ───────────────────────────────────────────────────────────────
N_ROBOTS   = 5
ANGLE_V    = math.pi / 4   # 45°
D          = 1.5
BETA       = 2.0
ANCHOR_SPD = 0.5
NEIGHBOR_R = 10.0
T_FINAL    = 60.0

n_followers = N_ROBOTS - 1
n_left  = n_followers // 2
n_right = n_followers - n_left

# ── Trayectoria con giros ────────────────────────────────────────────────────

def _smooth_arc(lx, ly, lth, turn_dir, n_inner, n=10):
    R  = n_inner * D
    cx = lx + R * math.cos(lth + turn_dir * math.pi / 2)
    cy = ly + R * math.sin(lth + turn_dir * math.pi / 2)
    t0 = math.atan2(ly - cy, lx - cx)
    pts = []
    for i in range(n):
        t = t0 + turn_dir * math.pi / 2 * i / (n - 1)
        pts += [round(cx + R * math.cos(t), 4), round(cy + R * math.sin(t), 4)]
    return pts

WAYPOINTS = (
    [5.0, 0.0]
    + _smooth_arc(5.0, 0.0,          0.0,         +1, n_left)
    + [8.0, 5.0]
    + _smooth_arc(8.0, 5.0, math.pi / 2,         +1, n_left)
    + [0.0, 8.0]
)

# ── Posiciones iniciales ────────────────────────────────────────────────────

def _init_positions(n, angle_v, d):
    n_followers = n - 1
    n_left  = n_followers // 2
    n_right = n_followers - n_left

    def rot(theta, vx, vy):
        c, s = math.cos(theta), math.sin(theta)
        return c * vx - s * vy, s * vx + c * vy

    pos = [(0.0, 0.0, 0.0)]
    for k in range(1, n_left + 1):
        rx, ry = rot(angle_v, -k * d, 0.0)
        pos.append((rx, ry, 0.0))
    for k in range(1, n_right + 1):
        rx, ry = rot(-angle_v, -k * d, 0.0)
        pos.append((rx, ry, 0.0))
    return pos

# ── Construir agentes ───────────────────────────────────────────────────────

init_pos = _init_positions(N_ROBOTS, ANGLE_V, D)

agents = [LeaderAgent(WaypointsNav(WAYPOINTS, vmax=0.5, wmax=0.6))]
for i in range(1, N_ROBOTS):
    agents.append(FollowerAgent(
        ConsensusFormation(anchor_speed=ANCHOR_SPD),
        PolarControlLaw(k1=1.0, k2=1.5, vmax=0.8, wmax=0.8),
        leader_id     = 'robot0',
        follower_index= i,
        n_followers   = N_ROBOTS - 1,
        angle_v       = ANGLE_V,
        d             = D,
        beta          = BETA,
    ))

# ── Simulación ───────────────────────────────────────────────────────────────

print("=" * 55)
print("S13 — Efecto látigo: figura con errores en X y Y")
print("=" * 55)

sim     = Simulation(init_pos, agents, neighbor_radius=NEIGHBOR_R)
history = sim.run(T_FINAL)

# ── Extraer datos ───────────────────────────────────────────────────────────

times   = np.array(history['time'])
n_steps = len(times)

# Extraer x, y, theta para cada robot
robot_data = {}
for i in range(N_ROBOTS):
    xs = [s[0] for s in history['states'][i]]
    ys = [s[1] for s in history['states'][i]]
    th = [s[2] for s in history['states'][i]]
    robot_data[i] = {
        'x': np.array(xs),
        'y': np.array(ys),
        'theta': np.array(th),
    }

# ── Calcular errores en X y Y ───────────────────────────────────────────────
# error_i(t) = posición_real_i(t) − (posición_líder(t) + offset_i(t))

def ideal_offset(follower_idx, theta_L):
    R_left  = np.array([[math.cos(theta_L + ANGLE_V), -math.sin(theta_L + ANGLE_V)],
                        [math.sin(theta_L + ANGLE_V),  math.cos(theta_L + ANGLE_V)]])
    R_right = np.array([[math.cos(theta_L - ANGLE_V), -math.sin(theta_L - ANGLE_V)],
                        [math.sin(theta_L - ANGLE_V),  math.cos(theta_L - ANGLE_V)]])
    if follower_idx <= n_left:
        k = follower_idx
        off = R_left @ np.array([-k * D, 0.0])
    else:
        k = follower_idx - n_left
        off = R_right @ np.array([-k * D, 0.0])
    return off

L = robot_data[0]
error_x = {}
error_y = {}

for i in range(1, N_ROBOTS):
    ex = np.zeros(n_steps)
    ey = np.zeros(n_steps)
    for k in range(n_steps):
        off = ideal_offset(i, float(L['theta'][k]))
        ex[k] = robot_data[i]['x'][k] - (L['x'][k] + off[0])
        ey[k] = robot_data[i]['y'][k] - (L['y'][k] + off[1])
    error_x[i] = ex
    error_y[i] = ey

# ── Figura combinada (3 paneles, estilo states_plotter_v2) ─────────────────

FIG_W, FIG_H = 15, 5
LABELS = ['Líder'] + [f'Seguidor {i}' for i in range(1, N_ROBOTS)]

fig = Figure(figsize=(FIG_W, FIG_H))
FigureCanvasAgg(fig)
axes = fig.subplots(1, 3)

# ── Panel 1: Trayectorias ──────────────────────────────────────────────────
ax = axes[0]
for i in range(N_ROBOTS):
    c = COLORS[i % len(COLORS)]
    x = robot_data[i]['x']
    y = robot_data[i]['y']
    t = robot_data[i]['theta']

    ax.plot(x, y, color=c, label=LABELS[i], linewidth=1.8, zorder=2)
    ax.scatter(x[0], y[0], s=90, facecolors='white',
               edgecolors=c, linewidths=2, zorder=4)
    ax.scatter(x[-1], y[-1], color=c, marker='*',
               s=220, zorder=5, edgecolors='black', linewidths=0.5)
    _draw_triangles(ax, x, y, t, c)

ax.set_title("Trayectoria de los robots", fontsize=12)
ax.set_xlabel("x (m)", fontsize=11)
ax.set_ylabel("y (m)", fontsize=11)
ax.grid(True, alpha=0.3)
ax.set_aspect('equal')
ax.legend(fontsize=9, loc='best')

# ── Panel 2: Error en X ────────────────────────────────────────────────────
ax = axes[1]
for i in range(1, N_ROBOTS):
    c = COLORS[i % len(COLORS)]
    ax.plot(times, error_x[i], color=c, label=LABELS[i], linewidth=1.8)
ax.axhline(0.0, color='black', linewidth=0.5, alpha=0.3, zorder=1)
ax.set_title("Error en X", fontsize=12)
ax.set_xlabel("Tiempo (s)", fontsize=11)
ax.set_ylabel("Error en x (m)", fontsize=11)
ax.grid(True, alpha=0.3)
ax.legend(fontsize=9)

# ── Panel 3: Error en Y ────────────────────────────────────────────────────
ax = axes[2]
for i in range(1, N_ROBOTS):
    c = COLORS[i % len(COLORS)]
    ax.plot(times, error_y[i], color=c, label=LABELS[i], linewidth=1.8)
ax.axhline(0.0, color='black', linewidth=0.5, alpha=0.3, zorder=1)
ax.set_title("Error en Y", fontsize=12)
ax.set_xlabel("Tiempo (s)", fontsize=11)
ax.set_ylabel("Error en y (m)", fontsize=11)
ax.grid(True, alpha=0.3)
ax.legend(fontsize=9)

fig.tight_layout()

# ── Guardar ─────────────────────────────────────────────────────────────────

SAVE_PATH = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'documentacion', 'figuras', 'personales', 'formacion_latigo.pdf'
))
os.makedirs(os.path.dirname(SAVE_PATH), exist_ok=True)
fig.savefig(SAVE_PATH, dpi=150, bbox_inches="tight")
print(f"Figura guardada en: {SAVE_PATH}")
plt.close(fig)
print("Listo.")
