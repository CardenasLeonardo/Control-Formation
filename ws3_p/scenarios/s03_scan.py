#!/usr/bin/env python3
"""
S02b — Barrido de radio mínimo para trayectoria circular sin culetazos.

Para cada radio R en RADIOS:
  - Simula 1.5 vueltas completas (la primera vuelta como transitorio).
  - En la segunda media vuelta (estado ~estacionario) mide:
      · error_pico[i]   = max ‖eᵢ‖
      · error_std[i]    = std ‖eᵢ‖  (alta → oscilaciones → culetazo)
      · v_sat_frac      = fracción de pasos en que robot exterior saturó vmax

Criterio de culetazo: error_pico del robot EXTERIOR (robot 2) supera UMBRAL_PICO,
o su desviación estándar en estado estacionario supera UMBRAL_STD.

Robots:
  - robot 2 → brazo IZQUIERDO k=2 → EXTERIOR del giro CCW → más propenso
  - robot 4 → brazo DERECHO  k=2 → INTERIOR del giro CCW
"""

import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

from algorithms.control_law   import PolarControlLaw
from algorithms.waypoints_nav import WaypointsNav
from algorithms.formacion_vs  import FormacionVS
from sim.engine               import Simulation, LeaderAgent, FollowerAgent
from viz.plotter              import compute_formation_errors

# ── Configuración fija ────────────────────────────────────────────────────────

N_ROBOTS   = 5
ANGLE_V    = math.pi / 4
D          = 1.0
BETA       = 2.0
ANCHOR_SPD = 0.5
NEIGHBOR_R = 10.0

V_LIDER    = 0.5
W_LIDER    = 0.6
VMAX_F     = 0.8
WMAX_F     = 0.8

N_WP       = 72          # waypoints por vuelta (cada 5°)
N_VUELTAS  = 1.5         # total: 1 vuelta de transitorio + 0.5 de análisis
SS_FRACCION = 2 / 3      # fracción final de la sim. = "estado estacionario"

UMBRAL_PICO = 0.8        # m — por encima → culetazo detectado
UMBRAL_STD  = 0.20       # m — por encima → oscilaciones detectadas

# Robot exterior (izquierdo k=2) durante giro CCW
IDX_EXTERIOR = 2
IDX_INTERIOR = 4

# Radios a barrer: densos cerca de la zona crítica predicha (~2.75 m)
RADIOS = [1.0, 1.5, 2.0, 2.25, 2.5, 2.75, 3.0, 3.25, 3.5, 4.0, 5.0]

# Radios destacados para graficar error vs tiempo (4 paneles)
RADIOS_DETALLE = [4.0, 3.0, 2.5, 1.5]


# ── Helpers ───────────────────────────────────────────────────────────────────

def _circle_waypoints(radius, n_wp, n_vueltas):
    """Círculo CCW centrado en (0, radius), arrancando desde (0,0) heading este."""
    cx, cy = 0.0, radius
    total  = int(n_wp * n_vueltas) + 1
    pts    = []
    for i in range(total):
        angle = -math.pi / 2 + 2 * math.pi * i / n_wp
        pts  += [round(cx + radius * math.cos(angle), 4),
                 round(cy + radius * math.sin(angle), 4)]
    return pts


def _init_positions():
    n_followers = N_ROBOTS - 1
    n_left  = n_followers // 2
    n_right = n_followers - n_left
    pos = [(0.0, 0.0, 0.0)]
    for k in range(1, n_left  + 1):
        th = ANGLE_V
        pos.append((-k * D * math.cos(th), -k * D * math.sin(th), 0.0))
    for k in range(1, n_right + 1):
        th = -ANGLE_V
        pos.append((-k * D * math.cos(th), -k * D * math.sin(th), 0.0))
    return pos


def _run(radius):
    """Ejecuta la simulación para el radio dado y retorna history."""
    wps      = _circle_waypoints(radius, N_WP, N_VUELTAS)
    t_final  = N_VUELTAS * 2 * math.pi * radius / (V_LIDER * 0.85)   # +15% margen

    init_pos = _init_positions()
    agents   = [LeaderAgent(WaypointsNav(wps, vmax=V_LIDER, wmax=W_LIDER))]
    for i in range(1, N_ROBOTS):
        agents.append(FollowerAgent(
            FormacionVS(anchor_speed=ANCHOR_SPD),
            PolarControlLaw(k1=1.0, k2=1.5, vmax=VMAX_F, wmax=WMAX_F),
            agent0_id   = 'robot0',
            agent_index = i,
            n_agents    = N_ROBOTS,
            angle_v     = ANGLE_V,
            d           = D,
            beta        = BETA,
        ))

    sim = Simulation(init_pos, agents, neighbor_radius=NEIGHBOR_R)
    return sim.run(t_final, verbose=False), t_final


def _ss_slice(errors_dict, history, ss_frac):
    """Retorna un slice del diccionario de errores correspondiente al estado estacionario."""
    n_total = len(history['time'])
    i_ss    = int(n_total * (1 - ss_frac))
    return {k: np.array(v[i_ss:]) for k, v in errors_dict.items()}


# ── Barrido principal ─────────────────────────────────────────────────────────

print("=" * 58)
print("S02b — Barrido de radio mínimo (formación VS, giro CCW)")
print(f"       Radios: {RADIOS}")
print("=" * 58)

resultados   = []   # lista de dicts con métricas por radio
historias    = {}   # {radio: history}   para los radios de detalle

for R in RADIOS:
    print(f"  R = {R:.2f} m ...", end=' ', flush=True)
    history, t_final = _run(R)
    errors_all = compute_formation_errors(history, N_ROBOTS, ANGLE_V, D)
    errors_ss  = _ss_slice(errors_all, history, SS_FRACCION)

    fila = {'R': R, 't_final': t_final}
    for i in range(1, N_ROBOTS):
        fila[f'pico_{i}']  = float(np.max(errors_ss[i]))
        fila[f'std_{i}']   = float(np.std(errors_ss[i]))
        fila[f'media_{i}'] = float(np.mean(errors_ss[i]))

    ext  = fila[f'pico_{IDX_EXTERIOR}']
    std  = fila[f'std_{IDX_EXTERIOR}']
    flag = ('** CULETAZO **' if ext > UMBRAL_PICO or std > UMBRAL_STD
            else 'OK')
    print(f"ext_pico={ext:.3f}  std={std:.3f}  {flag}")
    fila['flag'] = flag

    resultados.append(fila)
    if R in RADIOS_DETALLE:
        historias[R] = (history, errors_all, t_final)

# ── Tabla resumen ─────────────────────────────────────────────────────────────

print()
print(f"{'R':>6}  {'r2_pico':>8}  {'r2_std':>7}  {'r4_pico':>8}  {'r4_std':>7}  Estado")
print("-" * 58)
for f in resultados:
    R   = f['R']
    ep2 = f[f'pico_{IDX_EXTERIOR}']
    s2  = f[f'std_{IDX_EXTERIOR}']
    ep4 = f[f'pico_{IDX_INTERIOR}']
    s4  = f[f'std_{IDX_INTERIOR}']
    print(f"{R:6.2f}  {ep2:8.3f}  {s2:7.3f}  {ep4:8.3f}  {s4:7.3f}  {f['flag']}")

# ── Figura 1: métricas vs radio ───────────────────────────────────────────────

Rs     = [f['R'] for f in resultados]
pico_e = [f[f'pico_{IDX_EXTERIOR}'] for f in resultados]
std_e  = [f[f'std_{IDX_EXTERIOR}']  for f in resultados]
pico_i = [f[f'pico_{IDX_INTERIOR}'] for f in resultados]
std_i  = [f[f'std_{IDX_INTERIOR}']  for f in resultados]

fig1, axes = plt.subplots(1, 2, figsize=(13, 5))
fig1.suptitle("S02b — Barrido de radio: robot EXTERIOR (r2) vs INTERIOR (r4)", fontsize=12)

ax = axes[0]
ax.plot(Rs, pico_e, 'o-', color='tab:red',    lw=2, label='r2 pico (exterior)')
ax.plot(Rs, pico_i, 's-', color='tab:blue',   lw=2, label='r4 pico (interior)')
ax.axhline(UMBRAL_PICO, color='tab:red', ls='--', lw=1.2, alpha=0.6,
           label=f'umbral pico = {UMBRAL_PICO} m')
ax.set_xlabel('Radio del círculo R (m)')
ax.set_ylabel('Error pico ‖eᵢ‖ (m)  —  estado estacionario')
ax.set_title('Error pico en estado estacionario')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)

ax = axes[1]
ax.plot(Rs, std_e, 'o-', color='tab:orange',  lw=2, label='r2 std (exterior)')
ax.plot(Rs, std_i, 's-', color='tab:green',   lw=2, label='r4 std (interior)')
ax.axhline(UMBRAL_STD,  color='tab:orange', ls='--', lw=1.2, alpha=0.6,
           label=f'umbral std = {UMBRAL_STD} m')
ax.set_xlabel('Radio del círculo R (m)')
ax.set_ylabel('Desv. estándar ‖eᵢ‖ (m)  —  estado estacionario')
ax.set_title('Oscilaciones en estado estacionario (indicador de culetazo)')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)

fig1.tight_layout()

# ── Figura 2: error vs tiempo para radios de detalle ─────────────────────────

fig2, axs = plt.subplots(2, 2, figsize=(14, 8))
fig2.suptitle("S02b — Error ‖eᵢ‖ vs tiempo para radios representativos", fontsize=12)
axs = axs.flatten()

colors_r = {1: 'tab:orange', 2: 'tab:red', 3: 'tab:cyan', 4: 'tab:blue'}

for ax, R in zip(axs, RADIOS_DETALLE):
    if R not in historias:
        continue
    hist, errs, tf = historias[R]
    times = hist['time']
    for i in range(1, N_ROBOTS):
        lbl = f'r{i}' + (' ← ext' if i == IDX_EXTERIOR else
                         ' ← int' if i == IDX_INTERIOR else '')
        lw  = 2.2 if i in (IDX_EXTERIOR, IDX_INTERIOR) else 1.0
        ax.plot(times, errs[i], color=colors_r[i], lw=lw, label=lbl, alpha=0.85)
    ax.axhline(UMBRAL_PICO, color='k', ls='--', lw=1, alpha=0.4)
    # Franja de estado estacionario
    t_ss = times[int(len(times) * (1 - SS_FRACCION))]
    ax.axvspan(t_ss, times[-1], alpha=0.07, color='gray', label='zona SS')
    ax.set_title(f'R = {R} m  (T = {tf:.0f} s)')
    ax.set_xlabel('Tiempo (s)')
    ax.set_ylabel('‖eᵢ‖ (m)')
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

fig2.tight_layout()

# ── Figura 3: trayectorias para radios de detalle ────────────────────────────

colors_rob = plt.get_cmap('tab10')
rob_colors = [(0.85, 0.1, 0.1, 1)] + [colors_rob(i) for i in range(1, N_ROBOTS)]

fig3, axs3 = plt.subplots(1, len(RADIOS_DETALLE),
                           figsize=(5 * len(RADIOS_DETALLE), 5))
fig3.suptitle("S02b — Trayectorias por radio (1.5 vueltas)", fontsize=12)

for ax, R in zip(axs3, RADIOS_DETALLE):
    if R not in historias:
        continue
    hist, _, _ = historias[R]
    for i in range(N_ROBOTS):
        xs = [s[0] for s in hist['states'][i]]
        ys = [s[1] for s in hist['states'][i]]
        lw = 2.2 if i == 0 else 1.4
        ax.plot(xs, ys, color=rob_colors[i], lw=lw,
                label=('Líder' if i == 0 else f'r{i}'), alpha=0.75)
    circle_ref = plt.Circle((0, R), R, fill=False, color='k',
                             ls=':', lw=0.8, alpha=0.4)
    ax.add_patch(circle_ref)
    ax.set_title(f'R = {R} m')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=7, loc='upper right')

fig3.tight_layout()

# ── Guardar ───────────────────────────────────────────────────────────────────

OUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'figuras', 's03_scan')
os.makedirs(OUT_DIR, exist_ok=True)

fig1.savefig(os.path.join(OUT_DIR, 'metricas_vs_radio.png'), dpi=150)
fig2.savefig(os.path.join(OUT_DIR, 'error_tiempo_detalle.png'), dpi=150)
fig3.savefig(os.path.join(OUT_DIR, 'trayectorias_detalle.png'), dpi=150)
print(f"\nFiguras guardadas en: {OUT_DIR}/")

plt.show()
