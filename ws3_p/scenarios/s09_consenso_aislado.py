#!/usr/bin/env python3
"""
S09 — Consenso promedio, agente aislado (robot1 sin conexiones propias).

Mismo grafo en estrella que S08, pero se anula la fila de robot1 en A: deja
de "escuchar" a los demás (xi_dot=0, nunca se mueve), aunque robot0 sigue
"escuchándolo" a él. robot1 actúa como ancla fija: el resto del grupo
converge hacia SU posición inicial en vez del promedio real de los 4.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np

from algorithms.holonomic import simulate_consensus
from viz.holonomic_plotter import plot_holonomic_demo

# ── Parámetros ───────────────────────────────────────────────────────────────
LABELS = ['robot0', 'robot1', 'robot2', 'robot3']
X0 = np.array([
    [-4.0, -3.5],
    [ 4.0, -2.5],
    [ 3.5,  4.0],
    [-3.5,  3.0],
])
DT = 0.02
N_ITERS = 1400

SAVE_PATH = os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'documentacion', 'docs', 'seminario_iv', 'Pics',
    'consenso_promedio_not (1).png'
)

# ── Estrella centrada en robot0, pero robot1 no escucha a nadie ────────────
N = len(LABELS)
A = np.zeros((N, N))
for j in (1, 2, 3):
    A[0, j] = 1.0
    A[j, 0] = 1.0
A[1, :] = 0.0   # robot1 deja de escuchar -> nunca se mueve

# ── Simulación ────────────────────────────────────────────────────────────────
print("=" * 55)
print("S09 — Consenso promedio: robot1 aislado (ancla fija)")
print("=" * 55)

history = simulate_consensus(X0, A, N_ITERS, DT)

true_mean = X0.mean(axis=0)
final = history[-1]
print(f"Promedio real de los 4 (NO se debería alcanzar): {true_mean}")
print(f"Posición fija de robot1 (el grupo debería converger aquí): {X0[1]}")
print(f"Posiciones finales:\n{final}")
print(f"robot1 no se movió: {np.allclose(final[1], X0[1], atol=1e-9)}")
print(f"¿Convergió a la posición de robot1 (no al promedio real)? "
      f"{np.allclose(final, X0[1], atol=1e-2)}")

# ── Error: desviación respecto al centroide instantáneo del grupo ──────────
centroid = history.mean(axis=1, keepdims=True)
err = history - centroid
error_x = [err[:, i, 0] for i in range(N)]
error_y = [err[:, i, 1] for i in range(N)]

trajectories = [history[:, i, :] for i in range(N)]

# ── Visualización ─────────────────────────────────────────────────────────────
save_path = os.path.abspath(SAVE_PATH)
print(f"Guardando en: {save_path}")
assert os.path.isdir(os.path.dirname(save_path)), "Directorio destino no existe"

plot_holonomic_demo(save_path, LABELS, trajectories, A, error_x, error_y,
                     tri_d_step=0.15, tri_size=0.09)
print("Listo.")
