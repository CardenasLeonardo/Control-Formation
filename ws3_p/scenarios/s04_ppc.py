#!/usr/bin/env python3
"""
S03 — Estructura virtual + Control de Rendimiento Prescrito (PPC).

Algoritmo: FormacionVSPPC
    ρ(t)  = (ρ₀ − ρ∞)·exp(−l·t) + ρ∞           (envolvente)
    ξᵢ    = ‖eᵢ‖ / ρ(t)
    aᵢ_T  = arctanh(ξᵢ)                          (transformación)

Garantiza ‖eᵢ‖ < ρ(t) para todo t ≥ 0.
Cuando el error se acerca a la cota, la ganancia crece → control más agresivo.
"""
import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import matplotlib.pyplot as plt

from algorithms.control_law       import PolarControlLaw
from algorithms.waypoints_nav     import WaypointsNav
from algorithms.formacion_vs_ppc  import FormacionVSPPC
from sim.engine                   import Simulation, LeaderAgent, FollowerAgent
from viz.plotter                  import plot_trajectories, plot_errors, plot_snapshots

# ── Parámetros ───────────────────────────────────────────────────────────────
N_ROBOTS   = 5
ANGLE_V    = math.pi / 4
D          = 1.0
BETA       = 2.0
ANCHOR_SPD = 0.5
RHO_0      = 3.0
RHO_INF    = 0.10
L_RATE     = 0.05
NEIGHBOR_R = 10.0
T_FINAL    = 60.0

# ── Trayectoria: arco alrededor del pivote ──────────────────────────────────
#
# En un giro a la izquierda (CCW), el pivote es el robot extremo del brazo
# izquierdo (robot con índice n_left). En vez de que el líder gire alrededor
# de un centro arbitrario, los waypoints se generan como puntos sobre un arco
# de radio R = n_left·D centrado EXACTAMENTE en la posición del pivote.
#
# Así, el pivote permanece (casi) estacionario durante el giro, evitando que
# el robot interior reciba referencias que generen latigazos o reversas.
#
# La transformación aplicada a cada waypoint es:
#     P        = pivote (posición del robot extremo interior)
#     v₀       = líder − P   (vector del pivote al líder, magnitud R)
#     v(Δθ)    = rotación(Δθ) · v₀
#     líder(Δθ)= P + v(Δθ)

n_left = (N_ROBOTS - 1) // 2

TURN_ANGLE  = math.pi / 2    # 90° de giro (luego probamos más)
TURN_DIR    = +1             # +1 = CCW (izquierda), -1 = CW (derecha)
N_WP_PIVOT  = 100            # waypoints en el arco


def _pivot_arc_waypoints(lx, ly, lth, n_left, d, angle_v, turn_angle, n_wp, turn_dir=+1):
    """
    Genera waypoints para el líder trazando un arco alrededor del pivote
    (el robot extremo del brazo interior del giro).

    Parámetros:
        lx, ly, lth : posición y heading iniciales del líder
        n_left      : número de robots en el brazo izquierdo
        d           : separación entre robots en la V
        angle_v     : ángulo de apertura de la V (rad)
        turn_angle  : ángulo total del giro (rad), ej. π/2 = 90°
        n_wp        : número de waypoints en el arco
        turn_dir    : +1 CCW (izquierda), -1 CW (derecha)
    """
    # Offset del pivote respecto al líder (en frame mundo)
    pivot_ox = -n_left * d * math.cos(lth + angle_v)
    pivot_oy = -n_left * d * math.sin(lth + angle_v)

    Px = lx + pivot_ox   # pivote: posición mundo
    Py = ly + pivot_oy

    # Vector del pivote al líder
    vx = lx - Px   # = n_left·d·cos(lth + angle_v)
    vy = ly - Py   # = n_left·d·sin(lth + angle_v)

    pts = []
    for i in range(n_wp):
        delta = turn_dir * turn_angle * i / (n_wp - 1)
        c, s = math.cos(delta), math.sin(delta)
        rx = vx * c - vy * s
        ry = vx * s + vy * c
        pts += [round(Px + rx, 4), round(Py + ry, 4)]
    return pts


WAYPOINTS = _pivot_arc_waypoints(
    0.0, 0.0, 0.0,
    n_left, D, ANGLE_V,
    TURN_ANGLE, N_WP_PIVOT, TURN_DIR
)

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

# Guardar referencia al primer algoritmo PPC para graficar ρ(t)
ppc_ref = None
agents  = [LeaderAgent(WaypointsNav(WAYPOINTS, vmax=0.5, wmax=0.6))]
for i in range(1, N_ROBOTS):
    alg = FormacionVSPPC(
        anchor_speed=ANCHOR_SPD,
        rho_0=RHO_0, rho_inf=RHO_INF, l_rate=L_RATE,
    )
    if ppc_ref is None:
        ppc_ref = alg
    agents.append(FollowerAgent(
        alg,
        PolarControlLaw(k1=1.0, k2=1.5, vmax=0.8, wmax=0.8),
        agent0_id  = 'robot0',
        agent_index= i,
        n_agents   = N_ROBOTS,
        angle_v    = ANGLE_V,
        d          = D,
        beta       = BETA,
    ))

# ── Simulación ────────────────────────────────────────────────────────────────

R_PIVOT      = n_left * D
ARC_LENGTH   = TURN_ANGLE * R_PIVOT
T_ESTIMATED  = ARC_LENGTH / 0.4  # ~tiempo a velocidad crucero

print("=" * 55)
print("S03 — VS + PPC: Giro pivote 90°")
print(f"      Pivote=robot{n_left}  R_pivote={R_PIVOT:.2f}m  Arco={ARC_LENGTH:.2f}m")
print(f"      ρ₀={RHO_0}  ρ∞={RHO_INF}  l={L_RATE}")
print(f"      T_FINAL={T_FINAL}s  (estimado ~{T_ESTIMATED:.0f}s)")
print("=" * 55)

sim     = Simulation(init_pos, agents, neighbor_radius=NEIGHBOR_R)
history = sim.run(T_FINAL)

# ── Visualización ─────────────────────────────────────────────────────────────

OUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'figuras', 's03_pivot_90')
os.makedirs(OUT_DIR, exist_ok=True)

plot_trajectories(history, N_ROBOTS, WAYPOINTS,
                  title="S03 — VS + PPC: Giro pivote 90°",
                  ghost_every_s=2.0,
                  save_path=os.path.join(OUT_DIR, 'trayectorias.png'))

plot_errors(history, N_ROBOTS, ANGLE_V, D,
            title="S03 — VS + PPC: Error ‖eᵢ‖ vs envolvente ρ(t)",
            rho_algo=ppc_ref,
            save_path=os.path.join(OUT_DIR, 'errores.png'))

snap_times = list(range(0, int(T_FINAL) + 1, 6))
plot_snapshots(history, N_ROBOTS,
               times_s=snap_times,
               title="S03 — Snapshots: Giro pivote 90° (VS+PPC)",
               save_path=os.path.join(OUT_DIR, 'snapshots.png'))

print(f"\nGráficas guardadas en: {OUT_DIR}/")
plt.close('all')
