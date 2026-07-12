#!/usr/bin/env python3
"""
S14 — Efecto látigo en Gazebo (corrida original de ws3_old) con errores.

Reusa los datos EXACTOS de la simulación de 11 robots del resultados.pdf
antiguo (ws3_old.zip, figures_plotter_v2), extraídos del PDF vectorial a
formacion_latigo_gazebo_data.npz. Genera la misma figura de tres paneles,
pero con la evolución del error de formación en vez de la posición x/y:

  1. Trayectoria de los robots (idéntica a la original)
  2. Error en x (posición real − ancla ideal) para cada seguidor
  3. Error en y (posición real − ancla ideal) para cada seguidor

El ancla ideal de cada seguidor es líder + offset rotado con el heading del
líder. Los offsets no se recalculan con parámetros: se miden del estado
estacionario final de la propia corrida (en el marco del líder), y el heading
del líder se estima de su trayectoria (heading implícito del movimiento).

Salida: documentacion/figuras/personales/formacion_latigo_gazebo.pdf
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
DATA_PATH = os.path.join(BASE, 'formacion_latigo_gazebo_data.npz')
SAVE_PATH = os.path.join(BASE, 'formacion_latigo_gazebo.pdf')

LEADER = 'robot0'
V_STOP  = 0.08   # m/s: bajo esto el heading del líder se congela
W_MIN_S = 15.0   # s: duración mínima del tramo recto para medir offsets
DTH_MAX = np.deg2rad(1.0)   # rad/s: umbral de heading constante

# ── Cargar datos extraídos ───────────────────────────────────────────────────
data = np.load(DATA_PATH, allow_pickle=True)
labels = [str(l) for l in data['labels']]
n = len(labels)
traj   = [data[f'traj_{j}'] for j in range(n)]
evx    = [data[f'evx_{j}'] for j in range(n)]
evy    = [data[f'evy_{j}'] for j in range(n)]
cidx   = [int(data[f'traj_color_{j}']) for j in range(n)]

print("=" * 55)
print("S14 — Látigo Gazebo (11 robots) con errores de formación")
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
# La corrida termina a mitad de un giro, así que la cola no sirve. Se toma la
# parte final del primer y del último tramo recto y se promedian: el sesgo de
# arrastre del consenso (proporcional a la velocidad) cambia de signo con la
# dirección de marcha y se cancela entre ambos tramos.
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

# ── Figura de tres paneles (estética de states_plotter_v2 / resultados.pdf) ─
fig = Figure(figsize=(15, 5))
FigureCanvasAgg(fig)
axes = fig.subplots(1, 3)

ax = axes[0]
for j in range(n):
    c = COLORS[cidx[j] % len(COLORS)]
    ax.plot(traj[j][:, 0], traj[j][:, 1], color=c, label=labels[j],
            linewidth=1.8, zorder=2)
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
