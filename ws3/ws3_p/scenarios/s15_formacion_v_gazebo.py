#!/usr/bin/env python3
"""
S15 — Formación en V con líder físico en Gazebo (consenso_formacion.launch.py
de ws3_old) con evolución del error de formación.

Reusa los datos EXACTOS de la corrida de 5 robots de
ws3_old/ws3/figures_formacion/posiciones_finales.pdf, extraídos del PDF
vectorial a formacion_v_gazebo_data.npz. Genera la misma figura de tres
paneles con la estética original (triángulos de heading a lo largo de la
trayectoria, círculo hueco al inicio, estrella al final), pero con la
evolución del error de formación en vez de la posición x/y:

  1. Trayectoria de los robots (idéntica a la original)
  2. Error en x (posición real − ancla ideal) para cada seguidor
  3. Error en y (posición real − ancla ideal) para cada seguidor

El ancla ideal de cada seguidor es líder + offset rotado con el heading del
líder (implícito de su trayectoria). Los offsets se miden de los tramos
rectos estables de la propia corrida: se promedian el primero y el último,
cancelando el sesgo de arrastre del consenso, que cambia de signo con la
dirección de marcha.

Salida: documentacion/figuras/personales/formacion_v_gazebo.pdf
"""
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
    'documentacion', 'figuras', 'personales'))
DATA_PATH = os.path.join(BASE, 'formacion_v_gazebo_data.npz')
SAVE_PATH = os.path.join(BASE, 'formacion_v_gazebo.pdf')

LEADER  = 'robot0'
V_STOP  = 0.08   # m/s: bajo esto el heading del líder se congela
W_MIN_S = 8.0    # s: duración mínima del tramo recto para medir offsets
DTH_MAX = np.deg2rad(1.5)   # rad/s: umbral de heading constante


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


def implied_heading(x, y):
    """Heading implícito del movimiento a lo largo de una polilínea."""
    dx, dy = np.gradient(x), np.gradient(y)
    return np.arctan2(dy, dx)


# ── Cargar datos extraídos ───────────────────────────────────────────────────
data = np.load(DATA_PATH, allow_pickle=True)
labels = [str(l) for l in data['labels']]
n = len(labels)
traj = [data[f'traj_{j}'] for j in range(n)]
evx  = [data[f'evx_{j}'] for j in range(n)]
evy  = [data[f'evy_{j}'] for j in range(n)]
cidx = [int(data[f'traj_color_{j}']) for j in range(n)]

print("=" * 55)
print("S15 — Formación V en Gazebo (5 robots) con errores")
print("=" * 55)
print(f"robots ({n}): {labels}")

# ── Series x(t), y(t) en una malla temporal común ────────────────────────────
def sorted_curve(c):
    idx = np.argsort(c[:, 0])
    return c[idx, 0], c[idx, 1]

t0 = max(min(c[:, 0].min() for c in evx), min(c[:, 0].min() for c in evy))
t1 = min(max(c[:, 0].max() for c in evx), max(c[:, 0].max() for c in evy))
t = np.linspace(t0, t1, 900)

X = np.zeros((n, t.size))
Y = np.zeros((n, t.size))
t_start = np.zeros(n)            # spawn escalonado: inicio real de cada robot
t_end = np.zeros(n)              # fin real de los datos de cada robot
for j in range(n):
    tx, vx = sorted_curve(evx[j])
    ty, vy = sorted_curve(evy[j])
    X[j] = np.interp(t, tx, vx)
    Y[j] = np.interp(t, ty, vy)
    t_start[j] = max(tx.min(), ty.min())
    t_end[j] = min(tx.max(), ty.max())

jL = labels.index(LEADER)

# ── Heading del líder: implícito del movimiento, congelado si está quieto ──
def smooth(v, w=25):
    k = np.ones(w) / w
    return np.convolve(np.pad(v, w // 2, mode='edge'), k, mode='valid')[:v.size]

xl, yl = smooth(X[jL]), smooth(Y[jL])
dt = t[1] - t[0]
vx, vy = np.gradient(xl, dt), np.gradient(yl, dt)
speed = np.hypot(vx, vy)
theta_L = np.unwrap(np.arctan2(vy, vx))
for k in range(1, t.size):                       # congelar bajo V_STOP
    if speed[k] < V_STOP:
        theta_L[k] = theta_L[k - 1]
k0 = np.argmax(speed >= V_STOP)                  # arranque: primer heading válido
theta_L[:k0] = theta_L[k0]

# ── Offsets medidos en los tramos rectos estables de la corrida ─────────────
dth = np.abs(np.gradient(smooth(theta_L), dt))
straight = (dth < DTH_MAX) & (speed > V_STOP) & (t > t_start.max())
w_len = int(W_MIN_S / dt)

segments = []
k = 0
while k < t.size:
    if straight[k]:
        k_end = k
        while k_end < t.size and straight[k_end]:
            k_end += 1
        if k_end - k >= w_len:
            segments.append(slice(k_end - w_len, k_end))  # cola del tramo
        k = k_end
    else:
        k += 1
assert segments, "no se encontró tramo recto estable"
windows = [segments[0], segments[-1]] if len(segments) > 1 else segments
for w in windows:
    print(f"ventana de offsets: t ∈ [{t[w][0]:.1f}, {t[w][-1]:.1f}] s "
          f"(heading {np.degrees(np.median(theta_L[w])):+.1f}°)")

def offsets_in(win):
    out = {}
    dx = X[:, win] - X[jL][win]
    dy = Y[:, win] - Y[jL][win]
    th = theta_L[win]
    for j in range(n):
        if j == jL:
            continue
        rx = np.median(np.cos(-th) * dx[j] - np.sin(-th) * dy[j])
        ry = np.median(np.sin(-th) * dx[j] + np.cos(-th) * dy[j])
        out[j] = (rx, ry)
    return out

per_win = [offsets_in(w) for w in windows]
r_body = {j: (float(np.mean([o[j][0] for o in per_win])),
              float(np.mean([o[j][1] for o in per_win])))
          for j in per_win[0]}
for j, (rx, ry) in r_body.items():
    print(f"  {labels[j]}: offset en marco líder ({rx:+.2f}, {ry:+.2f}) m")

# ── Error de formación: real − (líder + offset rotado) ──────────────────────
err_x = {}
err_y = {}
c_t, s_t = np.cos(theta_L), np.sin(theta_L)
for j, (rx, ry) in r_body.items():
    ax_ideal = X[jL] + c_t * rx - s_t * ry
    ay_ideal = Y[jL] + s_t * rx + c_t * ry
    ex = X[j] - ax_ideal
    ey = Y[j] - ay_ideal
    invalid = (t < t_start[j]) | (t > t_end[j])   # fuera de los datos reales
    ex[invalid] = np.nan
    ey[invalid] = np.nan
    err_x[j] = ex
    err_y[j] = ey

# ── Figura de tres paneles (estética de states_plotter_v2) ──────────────────
fig = Figure(figsize=(15, 5))
FigureCanvasAgg(fig)
axes = fig.subplots(1, 3)

ax = axes[0]
for j in range(n):
    c = COLORS[cidx[j] % len(COLORS)]
    x, y = traj[j][:, 0], traj[j][:, 1]
    ax.plot(x, y, color=c, label=labels[j], linewidth=1.8, zorder=2)
    ax.scatter(x[0], y[0], s=90, facecolors='white',
               edgecolors=c, linewidths=2, zorder=4)
    ax.scatter(x[-1], y[-1], color=c, marker='*',
               s=220, zorder=5, edgecolors='black', linewidths=0.5)
    _draw_triangles(ax, x, y, implied_heading(x, y), c)
ax.set_title("Trayectoria de los robots")
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.grid(True)
ax.set_aspect('equal')
ax.legend(fontsize=8)

for ax, err, comp in ((axes[1], err_x, "x"), (axes[2], err_y, "y")):
    for j in sorted(err):
        c = COLORS[cidx[j] % len(COLORS)]
        ax.plot(t, err[j], color=c, label=labels[j], linewidth=1.8)
    ax.axhline(0.0, color='black', linewidth=0.6, alpha=0.3, zorder=1)
    ax.set_title(f"Error en {comp}")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel(f"Error en {comp} (m)")
    ax.grid(True)
    ax.legend(fontsize=8)

fig.tight_layout()
fig.savefig(SAVE_PATH, bbox_inches="tight")
print(f"Figura guardada en: {SAVE_PATH}")
print("Listo.")
