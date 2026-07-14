#!/usr/bin/env python3
"""
S16 — Formación en V en Gazebo, trayectoria en C (3 metas), error de consenso.

Post-procesa los datos crudos de consenso_formacion.launch.py con waypoints
(5,0) → (5,5) → (0,5) (datos.npz de states_plotter_v2) y genera la figura de
tres paneles con la estética de states_plotter_v2:

  1. Trayectoria de los robots (triángulos de heading, círculo inicio,
     estrella fin)
  2. Error de consenso en x
  3. Error de consenso en y

El error graficado es el del protocolo, el mismo vector (ex, ey) que
ConsensusFormation calcula antes de pasarlo a coordenadas polares:

  e_i = sum_j [(x_j - r_j) - (x_i - r_i)] + beta * [(x_L + r_i) - x_i]

con j sobre los demás seguidores (el líder va solo en el término beta) y
offsets r rotados con el heading real (odom) del líder.

Salida: documentacion/figuras/formacion_c/formacion_c_consenso.pdf
"""
import math
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family":      "serif",
    "font.serif":       ["Times New Roman"],
    "mathtext.fontset": "stix",
    "font.size":        11,
})

COLORS = plt.cm.tab10.colors

BASE = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'documentacion', 'figuras', 'formacion_c'))
DATA_PATH = os.path.join(BASE, 'datos.npz')
SAVE_PATH = os.path.join(BASE, 'formacion_c_consenso.pdf')

# Parámetros de la corrida (consenso_formacion.launch.py, defaults)
LEADER   = 'robot0'
N_ROBOTS = 5
ANGLE_V  = 0.7854
D        = 1.0
BETA     = 2.0

n_followers = N_ROBOTS - 1
n_left = n_followers // 2


def offsets_v(theta_v):
    """Offsets {índice: (rx, ry)} de la V, rotados con el heading del líder
    (misma geometría que consensus_formation._compute_offsets)."""
    out = {0: (0.0, 0.0)}
    for k in range(1, n_left + 1):
        th = theta_v + ANGLE_V
        out[k] = (-k * D * math.cos(th), -k * D * math.sin(th))
    for k in range(1, n_followers - n_left + 1):
        th = theta_v - ANGLE_V
        out[n_left + k] = (-k * D * math.cos(th), -k * D * math.sin(th))
    return out


# ── _draw_triangles (copia exacta de states_plotter_v2) ────────────────────
def _draw_triangles(ax, x, y, theta, color, d_step=0.4, size=0.25):
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


# ── Cargar datos crudos ──────────────────────────────────────────────────────
data = np.load(DATA_PATH, allow_pickle=True)
labels = sorted({k.rsplit('_', 1)[0] for k in data.files if k != 'labels'},
                key=lambda r: int(r.replace('robot', '')))
n = len(labels)

print("=" * 55)
print("S16 — Formación V en Gazebo, trayectoria en C, error de consenso")
print("=" * 55)
print(f"robots ({n}): {labels}")

series = {}
for r in labels:
    idx = np.argsort(data[f'{r}_t'])
    series[r] = {k: data[f'{r}_{k}'][idx] for k in ('t', 'x', 'y', 'theta')}

t0 = max(s['t'].min() for s in series.values())
t1 = min(s['t'].max() for s in series.values())
t = np.linspace(t0, t1, 1200)

X, Y, TH = {}, {}, {}
for r in labels:
    s = series[r]
    X[r] = np.interp(t, s['t'], s['x'])
    Y[r] = np.interp(t, s['t'], s['y'])
    TH[r] = np.interp(t, s['t'], np.unwrap(s['theta']))

followers = [r for r in labels if r != LEADER]

# ── Error de consenso (vector del protocolo, por componentes) ────────────────
err_x = {r: np.zeros(t.size) for r in followers}
err_y = {r: np.zeros(t.size) for r in followers}

for k in range(t.size):
    offs = offsets_v(TH[LEADER][k])
    for r in followers:
        i = int(r.replace('robot', ''))
        rix, riy = offs[i]
        ex = ey = 0.0
        for q in followers:
            if q == r:
                continue
            j = int(q.replace('robot', ''))
            rjx, rjy = offs[j]
            ex += (X[q][k] - rjx) - (X[r][k] - rix)
            ey += (Y[q][k] - rjy) - (Y[r][k] - riy)
        ex += BETA * ((X[LEADER][k] + rix) - X[r][k])
        ey += BETA * ((Y[LEADER][k] + riy) - Y[r][k])
        err_x[r][k] = ex
        err_y[r][k] = ey

# ── Figura de tres paneles (estética de states_plotter_v2) ──────────────────
fig = Figure(figsize=(15, 5))
FigureCanvasAgg(fig)
axes = fig.subplots(1, 3)

ax = axes[0]
for i, r in enumerate(labels):
    c = COLORS[int(r.replace('robot', '')) % len(COLORS)]
    s = series[r]
    ax.plot(s['x'], s['y'], color=c, label=r, linewidth=1.8, zorder=2)
    ax.scatter(s['x'][0], s['y'][0], s=90, facecolors='white',
               edgecolors=c, linewidths=2, zorder=4)
    ax.scatter(s['x'][-1], s['y'][-1], color=c, marker='*',
               s=220, zorder=5, edgecolors='black', linewidths=0.5)
    _draw_triangles(ax, s['x'], s['y'], s['theta'], c)
ax.set_title("Trayectoria de los robots")
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.grid(True)
ax.set_aspect('equal')
ax.legend(fontsize=8)

for ax, err, comp in ((axes[1], err_x, "x"), (axes[2], err_y, "y")):
    for r in followers:
        c = COLORS[int(r.replace('robot', '')) % len(COLORS)]
        ax.plot(t, err[r], color=c, label=r, linewidth=1.8)
    ax.axhline(0.0, color='black', linewidth=0.6, alpha=0.3, zorder=1)
    ax.set_title(f"Error de consenso en {comp}")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel(f"Error de consenso en {comp} (m)")
    ax.grid(True)
    ax.legend(fontsize=8)

fig.tight_layout()
fig.savefig(SAVE_PATH, bbox_inches="tight")
print(f"Figura guardada en: {SAVE_PATH}")
print("Listo.")
