#!/usr/bin/env python3
"""
S02 — Formación por estructura virtual pura (sin consenso inter-agente).
Trayectoria CIRCULAR — 2 vueltas completas.

Algoritmo: FormacionVS
    eᵢ = β · (x₀ + rᵢ(θ₀) − xᵢ)

No hay término Σⱼ. Cada agente actúa independientemente siguiendo solo su ancla.
"""
import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import matplotlib.pyplot as plt

from algorithms.control_law   import PolarControlLaw
from algorithms.waypoints_nav import WaypointsNav
from algorithms.formacion_vs  import FormacionVS
from sim.engine               import Simulation, LeaderAgent, FollowerAgent
from viz.plotter              import plot_trajectories, plot_errors, plot_snapshots

# ── Parámetros ───────────────────────────────────────────────────────────────
N_ROBOTS   = 5
ANGLE_V    = math.pi / 4
D          = 1.0
BETA       = 2.0
ANCHOR_SPD = 0.5
NEIGHBOR_R = 10.0
T_FINAL    = 160.0

# ── Trayectoria: círculo ────────────────────────────────────────────────────
# El líder empieza en (0,0) mirando hacia el este (derecha).
# El círculo está centrado en (0, RADIUS) = (0, 4) con radio RADIUS.
# El líder arranca desde ABAJO (ángulo -π/2), que es justo (0,0).
# Gira en sentido ANTIHORARIO: abajo → derecha → arriba → izquierda → abajo.
# La primera dirección desde (0,0) es ligeramente hacia la derecha (natural
# con el heading inicial = 0).

RADIUS       = 4.0
N_WP_CIRCLE  = 48       # waypoints por vuelta
N_CIRCLES    = 2        # número de vueltas completas


def _circle_waypoints(cx, cy, radius, n_wp, n_circles, start_angle=0):
    """
    Genera waypoints para n_circles vueltas en círculo CCW.
    start_angle determina el ángulo inicial (rad).
    Con start_angle=-π/2 arranca desde abajo.
    """
    pts = []
    total = n_wp * n_circles + 1  # +1 para cerrar el último arco
    for i in range(total):
        angle = start_angle + 2 * math.pi * i / n_wp
        pts += [round(cx + radius * math.cos(angle), 4),
                round(cy + radius * math.sin(angle), 4)]
    return pts


# Círculo centrado en (0, RADIUS), arranca desde -π/2 que es (0,0)
CX, CY = 0.0, RADIUS
circle_wps = _circle_waypoints(CX, CY, RADIUS, N_WP_CIRCLE, N_CIRCLES,
                                start_angle=-math.pi / 2)

# El primer waypoint es (0,0) = posición inicial del líder → arranque inmediato
WAYPOINTS = circle_wps


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
        FormacionVS(anchor_speed=ANCHOR_SPD),
        PolarControlLaw(k1=1.0, k2=1.5, vmax=0.8, wmax=0.8),
        agent0_id  = 'robot0',
        agent_index= i,
        n_agents   = N_ROBOTS,
        angle_v    = ANGLE_V,
        d          = D,
        beta       = BETA,
    ))

# ── Simulación ────────────────────────────────────────────────────────────────

print("=" * 55)
print("S02 — Estructura Virtual: Trayectoria CIRCULAR")
print(f"      Centro=({CX},{CY})  Radio={RADIUS}  Vueltas={N_CIRCLES}")
print("=" * 55)

sim     = Simulation(init_pos, agents, neighbor_radius=NEIGHBOR_R)
history = sim.run(T_FINAL)

# ── Visualización ─────────────────────────────────────────────────────────────

# Aumentamos frecuencia de ghosts para ver la formación en detalle
plot_trajectories(history, N_ROBOTS, WAYPOINTS,
                  title="S02 — Estructura Virtual: Trayectoria CIRCULAR",
                  ghost_every_s=3.0)

plot_errors(history, N_ROBOTS, ANGLE_V, D,
            title="S02 — Estructura Virtual: Error ‖eᵢ‖ — Trayectoria circular")

# Snapshots cada ~18 s para seguir el movimiento circular
snap_times = list(range(0, int(T_FINAL) + 1, 18))
plot_snapshots(history, N_ROBOTS,
               times_s=snap_times,
               title="S02 — Snapshots de formación (trayectoria circular)")

plt.show()
