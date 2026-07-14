#!/usr/bin/env python3
"""
S01 — Formación en V con consenso relativo inter-agente.

Algoritmo: ConsensusFormation
    eᵢ = Σⱼ[(xⱼ−rⱼ)−(xᵢ−rᵢ)]  +  β·[(xL+rᵢ)−xᵢ]

Trayectoria limpia con dos arcos de 90° CCW consecutivos, sin retrocesos:
    (0,0) → (7,0) → arco → (9,7) → arco → (0,9)
"""
import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import matplotlib.pyplot as plt

from algorithms.control_law         import PolarControlLaw
from algorithms.waypoints_nav       import WaypointsNav
from algorithms.consensus_formation import ConsensusFormation
from sim.engine                     import Simulation, LeaderAgent, FollowerAgent
from viz.plotter                    import plot_trajectories, plot_errors, plot_snapshots

# ── Parámetros ───────────────────────────────────────────────────────────────
N_ROBOTS   = 7
ANGLE_V    = math.pi / 4   # 45°
D          = 1.0
BETA       = 2.0
ANCHOR_SPD = 0.5
NEIGHBOR_R = 10.0
T_FINAL    = 70.0

# ── Trayectoria: giros con arcos suaves (sin retrocesos) ───────────────
#
# Ruta limpia que NO se cruza a sí misma:
#   (0,0) →E→ (7,0) → arco CCW 90° → (9,7) → arco CCW 90° → (0,9)

n_followers = N_ROBOTS - 1
n_left  = n_followers // 2
n_right = n_followers - n_left


def _smooth_arc(lx, ly, lth, turn_dir, n_inner, n=10):
    """Arco de 90° con continuidad de heading."""
    R  = n_inner * D
    cx = lx + R * math.cos(lth + turn_dir * math.pi / 2)
    cy = ly + R * math.sin(lth + turn_dir * math.pi / 2)
    t0 = math.atan2(ly - cy, lx - cx)
    pts = []
    for i in range(n):
        t = t0 + turn_dir * math.pi / 2 * i / (n - 1)
        pts += [round(cx + R * math.cos(t), 4), round(cy + R * math.sin(t), 4)]
    return pts

# ── Planificación de la ruta ────────────────────────────────────────────────

WAYPOINTS = (
    _smooth_arc(7.0, 0.0,          0.0,         +1, n_left)
    + [9.0, 7.0]
    + _smooth_arc(9.0, 7.0, math.pi / 2,         +1, n_left)
    + [0.0, 9.0]
)

# ── Posiciones iniciales en formación perfecta ────────────────────────────────

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

# ── Construir agentes ─────────────────────────────────────────────────────────

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

# ── Simulación ────────────────────────────────────────────────────────────────

print("=" * 55)
print("S01 — Formación V: Consenso relativo inter-agente")
print("=" * 55)

sim     = Simulation(init_pos, agents, neighbor_radius=NEIGHBOR_R)
history = sim.run(T_FINAL)

# ── Visualización ─────────────────────────────────────────────────────────────

plot_trajectories(history, N_ROBOTS, WAYPOINTS,
                  title="S01 — Formación V: Trayectorias (consenso relativo)")

plot_errors(history, N_ROBOTS, ANGLE_V, D,
            title="S01 — Formación V: Error ‖eᵢ‖ = ‖xᵢ − (x₀ + rᵢ(θ₀))‖")

plot_snapshots(history, N_ROBOTS,
               times_s=[0, 14, 28, 42, 57, 70],
               title="S01 — Snapshots de formación")

plt.show()
