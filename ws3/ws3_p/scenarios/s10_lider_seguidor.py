#!/usr/bin/env python3
"""
S10 — Consenso con seguimiento de líder, líder recorre 4 metas secuenciales.

robot0 (líder) avanza hacia 4 metas en secuencia (control proporcional simple).
Los seguidores (robot1-3) hacen consenso entre sí y son atraídos hacia la
posición ACTUAL del líder (no hacia la meta directamente).
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np

from algorithms.holonomic import waypoint_leader_step
from viz.holonomic_plotter import plot_holonomic_demo

# ── Parámetros ───────────────────────────────────────────────────────────────
LABELS = ['líder (robot0)', 'robot1', 'robot2', 'robot3']
X0 = np.array([
    [-1.0,  0.5],
    [-1.8,  0.2],
    [-1.3, -0.4],
    [-0.4, -0.2],
])
# Ruta en U (sin cruces): sube-derecha, baja, izquierda por abajo.
WAYPOINTS = [(0.0, 0.0), (4.0, 0.0), (4.0, -4.0), (0.0, -4.0)]

BETA    = 0.15    # atracción de cada seguidor hacia el líder
K_FOLL  = 0.10    # consenso entre seguidores
V_LEADER = 0.09
DT = 1.0
N_ITERS = 220

SAVE_PATH = os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'documentacion', 'docs', 'seminario_iv', 'Pics',
    'lider_seguidor_multimeta_mid3 (1).png'
)

# ── Grafo (para el panel de conexiones): líder -> cada seguidor,
#    y seguidores conectados entre sí en cadena ───────────────────────────────
N = len(LABELS)
A = np.zeros((N, N))
for i in (1, 2, 3):
    A[i, 0] = BETA
for (i, j) in [(1, 2), (2, 3)]:
    A[i, j] = K_FOLL
    A[j, i] = K_FOLL

# ── Simulación ────────────────────────────────────────────────────────────────
print("=" * 55)
print("S10 — Líder-seguidor: 4 metas secuenciales")
print("=" * 55)

history = np.zeros((N_ITERS + 1, N, 2))
history[0] = X0
X = X0.copy()
wp_index = 0
visited_at = []
last_wp_index = -1

for k in range(N_ITERS):
    X[0], wp_index, reached_last = waypoint_leader_step(
        X[0], wp_index, WAYPOINTS, V_LEADER, tolerance=0.08, dt=DT)
    if wp_index != last_wp_index:
        visited_at.append((last_wp_index, k))
        last_wp_index = wp_index

    dX = np.zeros_like(X)
    for i in (1, 2, 3):
        for j in (1, 2, 3):
            if i != j:
                dX[i] += K_FOLL * (X[j] - X[i])
        dX[i] += BETA * (X[0] - X[i])
    for i in (1, 2, 3):
        X[i] = X[i] + DT * dX[i]

    history[k + 1] = X

print(f"Última meta alcanzada (índice, en iteración): wp_index final = {wp_index}")
print(f"Posición final del líder: {history[-1, 0]}")
print(f"Posiciones finales de los seguidores:\n{history[-1, 1:]}")

# ── Error: cada seguidor respecto a la posición ACTUAL del líder ───────────
leader_pos = history[:, 0, :]
error_x = [None] + [history[:, i, 0] - leader_pos[:, 0] for i in (1, 2, 3)]
error_y = [None] + [history[:, i, 1] - leader_pos[:, 1] for i in (1, 2, 3)]

trajectories = [history[:, i, :] for i in range(N)]

# ── Marcadores de metas: visitadas (gris) vs meta actual/final (negra) ─────
wx = [w[0] for w in WAYPOINTS]
wy = [w[1] for w in WAYPOINTS]
extra_markers = [
    {'x': wx[:-1], 'y': wy[:-1], 'color': 'gray',  'label': 'meta visitada', 's': 90},
    {'x': [wx[-1]], 'y': [wy[-1]], 'color': 'black', 'label': 'meta', 's': 110},
]

# ── Visualización ─────────────────────────────────────────────────────────────
save_path = os.path.abspath(SAVE_PATH)
print(f"Guardando en: {save_path}")
assert os.path.isdir(os.path.dirname(save_path)), "Directorio destino no existe"

plot_holonomic_demo(save_path, LABELS, trajectories, A, error_x, error_y,
                     extra_markers=extra_markers)
print("Listo.")
