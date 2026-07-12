#!/usr/bin/env python3
"""
S08 — Consenso promedio, grafo parcialmente conexo (estrella centrada en robot0).

Mismas condiciones iniciales que S07, pero con solo 3 aristas (robot0-robot1,
robot0-robot2, robot0-robot3) en vez de las 6 del grafo completo. Al ser
simétrico y conexo, converge al MISMO promedio real, pero más lento (menor
conectividad algebraica / valor de Fiedler más bajo).
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
N_ITERS = 500

SAVE_PATH = os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'documentacion', 'docs', 'seminario_iv', 'Pics',
    'consenso_promedio_mid (1).png'
)

# ── Grafo en estrella centrado en robot0: 0-1, 0-2, 0-3 (simétrico) ────────
N = len(LABELS)
A = np.zeros((N, N))
for j in (1, 2, 3):
    A[0, j] = 1.0
    A[j, 0] = 1.0

# ── Simulación ────────────────────────────────────────────────────────────────
print("=" * 55)
print("S08 — Consenso promedio: grafo parcialmente conexo (estrella)")
print("=" * 55)

history = simulate_consensus(X0, A, N_ITERS, DT)

true_mean = X0.mean(axis=0)
final = history[-1]
print(f"Promedio real de las condiciones iniciales: {true_mean}")
print(f"Posiciones finales:\n{final}")
print(f"¿Convergió al promedio real? {np.allclose(final, true_mean, atol=1e-3)}")

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
