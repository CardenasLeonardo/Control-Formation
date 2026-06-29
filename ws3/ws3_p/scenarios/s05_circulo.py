#!/usr/bin/env python3
"""
S04 — Círculo completo con pivote en el extremo del brazo INTERIOR.

Para un giro CCW (izquierda), el brazo interior es el DERECHO (robots 3 y 4).
El robot 4 (brazo derecho k=2, extremo NW) es el pivote: quasi-estático
mientras el líder traza un círculo de radio R = n_right·D = 2 m alrededor de él.

Todos los seguidores usan FormacionVSPPC con los mismos parámetros.

Radios de cada robot alrededor del pivote (robot 4 en (-√2, +√2)):
  robot 0 (líder)    2.000 m   v = 0.500 m/s
  robot 1            1.000 m   v = 0.250 m/s
  robot 2 (exterior) 2.828 m   v = 0.707 m/s  (< vmax 0.8)  ← más amplio
  robot 3            2.236 m   v = 0.559 m/s
  robot 4 (PIVOTE)   0.000 m   v ≈ 0 m/s  ← quasi-estático

Heading mismatch inicial: 45° (tangente CCW apunta NE, líder parte heading E).
"""

import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib.pyplot as plt

from algorithms.control_law       import PolarControlLaw
from algorithms.circulo_nav       import CirculoNav
from algorithms.formacion_vs_ppc  import FormacionVSPPC
from sim.engine                   import Simulation, LeaderAgent, FollowerAgent
from viz.plotter                  import (plot_trajectories, plot_errors,
                                          plot_snapshots, compute_formation_errors)

# ── Parámetros de la formación ────────────────────────────────────────────────

N_ROBOTS   = 5
ANGLE_V    = math.pi / 4
D          = 1.0
BETA       = 2.0
ANCHOR_SPD = 0.5

RHO_0    = 3.0
RHO_INF  = 0.15
L_RATE   = 0.04

NEIGHBOR_R = 10.0

N_ROBOTS_F = N_ROBOTS - 1
N_LEFT     = N_ROBOTS_F // 2          # 2
N_RIGHT    = N_ROBOTS_F - N_LEFT      # 2

# Para CCW (+1, giro izquierda): pivote en brazo INTERIOR = brazo DERECHO
# Para CW  (-1, giro derecha):   pivote en brazo INTERIOR = brazo IZQUIERDO
TURN_DIR   = +1    # CCW → pivote en robot 4 (brazo derecho k=2, NW)
N_CIRCLES  = 2

V_LIDER  = 0.5
W_LIDER  = 0.6
VMAX_F   = 0.8
WMAX_F   = 0.8

# Índice del pivote y del exterior según la dirección de giro
if TURN_DIR > 0:   # CCW: interior = brazo derecho → robot n_left + n_right = 4
    IDX_PIVOT    = N_LEFT + N_RIGHT   # robot 4
    IDX_EXTERIOR = N_LEFT             # robot 2
else:              # CW: interior = brazo izquierdo → robot n_left = 2
    IDX_PIVOT    = N_LEFT             # robot 2
    IDX_EXTERIOR = N_LEFT + N_RIGHT   # robot 4


# ── Geometría del círculo ─────────────────────────────────────────────────────

R_PIVOT    = (N_RIGHT if TURN_DIR > 0 else N_LEFT) * D   # 2.0 m
ARC_LENGTH = 2 * math.pi * N_CIRCLES * R_PIVOT
T_FINAL    = ARC_LENGTH / (V_LIDER * 0.80) + 5.0
WP_CHORD   = 0.30

# Posición del pivote en el mundo (posición ideal inicial)
PIVOT_X = -R_PIVOT * math.cos(0.0 - ANGLE_V) if TURN_DIR > 0 else \
           -R_PIVOT * math.cos(0.0 + ANGLE_V)
PIVOT_Y = -R_PIVOT * math.sin(0.0 - ANGLE_V) if TURN_DIR > 0 else \
           -R_PIVOT * math.sin(0.0 + ANGLE_V)


# ── Posiciones iniciales ───────────────────────────────────────────────────────

def _init_positions():
    pos = [(0.0, 0.0, 0.0)]
    for k in range(1, N_LEFT + 1):
        th = ANGLE_V
        pos.append((-k * D * math.cos(th), -k * D * math.sin(th), 0.0))
    for k in range(1, N_RIGHT + 1):
        th = -ANGLE_V
        pos.append((-k * D * math.cos(th), -k * D * math.sin(th), 0.0))
    return pos


# ── Construir agentes ─────────────────────────────────────────────────────────

init_pos = _init_positions()

ppc_ref = None
agents  = [LeaderAgent(CirculoNav(
    pivot_id  = f'robot{IDX_PIVOT}',
    R         = R_PIVOT,
    n_circles = N_CIRCLES,
    turn_dir  = TURN_DIR,
    chord     = WP_CHORD,
    vmax      = V_LIDER,
    wmax      = W_LIDER,
))]
for i in range(1, N_ROBOTS):
    alg = FormacionVSPPC(
        anchor_speed = ANCHOR_SPD,
        rho_0    = RHO_0,
        rho_inf  = RHO_INF,
        l_rate   = L_RATE,
    )
    if ppc_ref is None:
        ppc_ref = alg
    agents.append(FollowerAgent(
        alg,
        PolarControlLaw(k1=1.0, k2=1.5, vmax=VMAX_F, wmax=WMAX_F),
        agent0_id   = 'robot0',
        agent_index = i,
        n_agents    = N_ROBOTS,
        angle_v     = ANGLE_V,
        d           = D,
        beta        = BETA,
    ))

# ── Simulación ────────────────────────────────────────────────────────────────

dir_str = "CCW (izq)" if TURN_DIR > 0 else "CW (der)"
print("=" * 60)
print(f"S04 — Círculo pivote {dir_str}  |  pivote=robot{IDX_PIVOT}  (brazo interior)")
print(f"  Pivote: ({PIVOT_X:.3f}, {PIVOT_Y:.3f})   R = {R_PIVOT:.2f} m")
print(f"  Vueltas: {N_CIRCLES}   Arc: {ARC_LENGTH:.1f} m   chord: {WP_CHORD} m")
print(f"  T_FINAL = {T_FINAL:.0f} s   ρ₀={RHO_0}  ρ∞={RHO_INF}  l={L_RATE}")
print("=" * 60)

sim     = Simulation(init_pos, agents, neighbor_radius=NEIGHBOR_R)
history = sim.run(T_FINAL)

# ── Métricas ──────────────────────────────────────────────────────────────────

errors_all = compute_formation_errors(history, N_ROBOTS, ANGLE_V, D)
times      = np.array(history['time'])
i_ss       = len(times) // 2

print("\nError  [pico_total / pico_SS / std_SS / media_SS]:")
print(f"{'Robot':>7}  {'pico_tot':>9}  {'pico_SS':>8}  {'std_SS':>7}  {'media_SS':>9}")
for i in range(1, N_ROBOTS):
    e    = np.array(errors_all[i])
    nota = (" ← PIVOTE" if i == IDX_PIVOT else
            " ← EXTERIOR" if i == IDX_EXTERIOR else "")
    print(f"  robot{i}  {np.max(e):9.3f}  {np.max(e[i_ss:]):8.3f}"
          f"  {np.std(e[i_ss:]):7.3f}  {np.mean(e[i_ss:]):9.3f}{nota}")

# Desplazamiento del pivote respecto a su posición inicial
px0, py0 = init_pos[IDX_PIVOT][0], init_pos[IDX_PIVOT][1]
desp_piv  = [math.hypot(history['states'][IDX_PIVOT][k][0] - px0,
                         history['states'][IDX_PIVOT][k][1] - py0)
             for k in range(len(times))]
print(f"\nDesplazamiento robot{IDX_PIVOT} (pivote):  "
      f"máximo={max(desp_piv):.3f} m   final={desp_piv[-1]:.3f} m")

# ── Visualización ─────────────────────────────────────────────────────────────

OUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'figuras', 's05_circulo')
os.makedirs(OUT_DIR, exist_ok=True)

# Fig 1: Trayectorias
fig1 = plot_trajectories(
    history, N_ROBOTS,
    title=f"S04 — Círculo pivote {dir_str}  (robot{IDX_PIVOT} = pivot, {N_CIRCLES} vueltas)",
    ghost_every_s=8.0,
    save_path=os.path.join(OUT_DIR, 'trayectorias.png')
)
ax1 = fig1.axes[0]

# Marcador del pivote
ax1.plot(PIVOT_X, PIVOT_Y, 'k+', markersize=16, markeredgewidth=2.5, zorder=6,
         label=f'Pivote robot{IDX_PIVOT} (pos. inicial)')

# Círculos de referencia: comportamiento ideal si el pivote fuera estático y la formación perfecta.
# Para cada robot, el radio ideal = distancia inicial al pivote.
cmap_ref = plt.get_cmap('tab10')
ref_colors = [cmap_ref(i % 10) for i in range(N_ROBOTS)]
ref_colors[0] = (0.85, 0.1, 0.1, 1.0)

for j, (xj, yj, _) in enumerate(init_pos):
    if j == IDX_PIVOT:
        continue
    rj = math.hypot(xj - PIVOT_X, yj - PIVOT_Y)
    lw  = 2.0 if j == 0 else 1.3
    tag = ' ← crít.' if j == IDX_PIVOT - 1 else ''
    ax1.add_patch(plt.Circle(
        (PIVOT_X, PIVOT_Y), rj,
        fill=False, color=ref_colors[j],
        linestyle='--', linewidth=lw, alpha=0.50, zorder=1,
        label=f'Ref robot{j}  r={rj:.3f} m{tag}'
    ))

ax1.legend(loc='best', fontsize=8)
fig1.savefig(os.path.join(OUT_DIR, 'trayectorias.png'), dpi=150)

# Fig 2: Errores vs tiempo + envolvente PPC
fig2 = plot_errors(
    history, N_ROBOTS, ANGLE_V, D,
    title=f"S04 — VS+PPC: Error ‖eᵢ‖  (pivote=robot{IDX_PIVOT})",
    rho_algo=ppc_ref,
    save_path=os.path.join(OUT_DIR, 'errores.png')
)

# Fig 3: Desplazamiento del pivote
fig3, ax3 = plt.subplots(figsize=(11, 4))
ax3.plot(times, desp_piv, color='tab:green', lw=2,
         label=f'Desplazamiento robot{IDX_PIVOT} (pivote)')
ax3.axhline(max(desp_piv), color='tab:red', ls='--', lw=1,
            label=f'Máx = {max(desp_piv):.3f} m')
ax3.set_xlabel('Tiempo (s)')
ax3.set_ylabel('Desplazamiento (m)')
ax3.set_title(f'S04 — Cuánto se mueve el pivote (robot{IDX_PIVOT}) durante el círculo completo')
ax3.legend(fontsize=9)
ax3.grid(True, alpha=0.3)
ax3.set_ylim(bottom=0)
fig3.tight_layout()
fig3.savefig(os.path.join(OUT_DIR, 'desplazamiento_pivote.png'), dpi=150)

# Fig 4: Snapshots
snap_times = list(range(0, int(T_FINAL), max(1, int(T_FINAL // 8))))
plot_snapshots(
    history, N_ROBOTS,
    times_s=snap_times,
    title=f"S04 — Snapshots: círculo pivote {dir_str} ({N_CIRCLES} vueltas)",
    save_path=os.path.join(OUT_DIR, 'snapshots.png')
)

print(f"\nFiguras guardadas en: {OUT_DIR}/")
plt.show()
