#!/usr/bin/env python3
"""
S05 — Visualización estática de la formación inicial.

Muestra la formación de 5 robots (1 líder + 4 seguidores) en la posición
de inicio: líder en el origen mirando al OESTE (heading = π).
"""

import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── Parámetros ────────────────────────────────────────────────────────────────

N_ROBOTS = 5
ANGLE_V  = math.pi / 4
D        = 1.0
HEADING  = 0.0              # mirando al este (derecha)

# ── Posiciones ────────────────────────────────────────────────────────────────

n_followers = N_ROBOTS - 1
n_left      = n_followers // 2
n_right     = n_followers - n_left

states = [(0.0, 0.0, HEADING)]   # líder

for k in range(1, n_left + 1):
    th = HEADING + ANGLE_V
    states.append((-k * D * math.cos(th), -k * D * math.sin(th), HEADING))
for k in range(1, n_right + 1):
    th = HEADING - ANGLE_V
    states.append((-k * D * math.cos(th), -k * D * math.sin(th), HEADING))

# ── Colores ───────────────────────────────────────────────────────────────────

cmap   = plt.get_cmap('tab10')
colors = [cmap(i % 10) for i in range(N_ROBOTS)]
colors[0] = (0.85, 0.1, 0.1, 1.0)   # líder en rojo

labels = ['Líder (robot0)'] + [f'robot{i}' for i in range(1, N_ROBOTS)]

# ── Aristas de la V ───────────────────────────────────────────────────────────

edges = []
for k in range(1, n_left + 1):
    edges.append((k - 1 if k > 1 else 0, k))
for k in range(1, n_right + 1):
    j = n_left + k
    edges.append((n_left + k - 1 if k > 1 else 0, j))

# ── Círculo pivote ───────────────────────────────────────────────────────────
# Diámetro = distancia entre líder y extremo del brazo derecho (robot n_left+n_right)

idx_extremo = n_left + n_right          # robot4
x_l, y_l    = states[0][0], states[0][1]
x_e, y_e    = states[idx_extremo][0], states[idx_extremo][1]
cx = x_e          # centro en el extremo (pivote)
cy = y_e
r  = math.hypot(x_e - x_l, y_e - y_l)   # radio = distancia líder–extremo

# ── Figura ────────────────────────────────────────────────────────────────────

fig, ax = plt.subplots(figsize=(8, 7))

# Ejes X e Y
ax.axhline(0, color='k', linewidth=0.8, alpha=0.6, zorder=1)
ax.axvline(0, color='k', linewidth=0.8, alpha=0.6, zorder=1)

# Círculos de todos los robots centrados en el extremo (pivote)
for j, (xj, yj, _) in enumerate(states):
    if j == idx_extremo:
        continue
    rj = math.hypot(xj - cx, yj - cy)
    lw = 2.0 if j == 0 else 1.4
    ax.add_patch(plt.Circle((cx, cy), rj, fill=False,
                             color=colors[j], linestyle='--',
                             linewidth=lw, alpha=0.45, zorder=1))

# Centro (pivote)
ax.plot(cx, cy, '+', color='k', markersize=10, markeredgewidth=1.8,
        zorder=3, label=f'Pivote robot{idx_extremo} (centro)')

# Aristas de la formación
for (a, b) in edges:
    ax.plot([states[a][0], states[b][0]],
            [states[a][1], states[b][1]],
            color='k', lw=1.8, alpha=0.6, zorder=2)

# El triángulo apunta en la dirección del heading.
# marker=(3, 0, rot): triángulo regular rotado rot grados CCW.
# Por defecto apunta al norte (+90°), así que rot = heading_deg - 90.
for i, (x, y, th) in enumerate(states):
    rot = math.degrees(th) - 90
    ms  = 22 if i == 0 else 17
    ax.plot(x, y,
            marker=(3, 0, rot),
            color=colors[i],
            markersize=ms,
            markeredgecolor='k',
            markeredgewidth=0.8,
            zorder=4,
            label=labels[i])
    # Etiqueta del robot
    lbl_off = (8, 6) if y >= 0 else (8, -14)
    if i == 0:
        lbl_off = (8, 8)
    ax.annotate(labels[i],
                xy=(x, y),
                xytext=lbl_off,
                textcoords='offset points',
                fontsize=9, color=colors[i],
                fontweight='bold')

ax.set_xlabel('x (m)', fontsize=12)
ax.set_ylabel('y (m)', fontsize=12)
ax.set_title('Formación inicial — 5 robots, heading este (0 rad)', fontsize=12)
ax.legend(loc='lower right', fontsize=9)
ax.set_aspect('equal')
ax.grid(True, alpha=0.25, linestyle='--')
ax.tick_params(labelsize=10)

margin = 0.5
r_max = max(math.hypot(states[j][0] - cx, states[j][1] - cy)
            for j in range(N_ROBOTS) if j != idx_extremo)
ax.set_xlim(cx - r_max - margin, cx + r_max + margin)
ax.set_ylim(cy - r_max - margin, cy + r_max + margin)

fig.tight_layout()

OUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'figuras', 's06_inicial')
os.makedirs(OUT_DIR, exist_ok=True)
fig.savefig(os.path.join(OUT_DIR, 'formacion.png'), dpi=150)
print(f"Figura guardada en: {OUT_DIR}/formacion.png")

plt.show()
