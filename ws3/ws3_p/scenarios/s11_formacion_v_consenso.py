#!/usr/bin/env python3
"""
S11 — Consenso con mantenimiento de formación (V con anclas virtuales), 8 agentes.

xi_dot = -sum_j a_ij[(xi-ri) - (xj-rj)] - beta*[(xi-ri) - xL]

robot0 es el líder (avanza hacia una única meta). Los 7 seguidores mantienen
sus anclas virtuales rotadas según el heading (implícito) del líder.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np

from algorithms.holonomic import waypoint_leader_step, implied_headings, v_offsets
from viz.holonomic_plotter import plot_holonomic_demo

# ── Parámetros ───────────────────────────────────────────────────────────────
LABELS = ['líder (robot0)', 'robot1', 'robot2', 'robot3',
          'robot4', 'robot5', 'robot6', 'robot7']
N = len(LABELS)

def _heptagon(radius):
    """7 seguidores en un heptágono regular alrededor del líder (origen)."""
    pts = [(0.0, 0.0)]  # robot0 (líder), en el centro
    for k in range(7):
        theta = np.pi / 2 + 2 * np.pi * k / 7   # empieza arriba, sentido horario
        pts.append((radius * np.cos(theta), radius * np.sin(theta)))
    return np.array(pts)


X0 = _heptagon(radius=2.0)

GOAL = np.array([7.0, 0.0])
ANGLE_V = np.pi / 6
D = 1.1
BETA   = 0.12
K_FOLL = 0.08
V_LEADER = 0.10
DT = 1.0
N_ITERS = 220

SAVE_PATH = os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'documentacion', 'docs', 'seminario_iv', 'Pics',
    'formacion_V_n46 (1).png'
)

# ── Grafo (panel de conexiones): líder -> cada seguidor + cadena entre
#    seguidores consecutivos (misma topología del efecto V) ────────────────
A = np.zeros((N, N))
for i in range(1, N):
    A[i, 0] = BETA
n_left = (N - 1) // 2
chain_left  = list(range(1, n_left + 1))
chain_right = list(range(n_left + 1, N))
for chain in (chain_left, chain_right):
    for a, b in zip(chain[:-1], chain[1:]):
        A[a, b] = K_FOLL
        A[b, a] = K_FOLL

# ── Simulación ────────────────────────────────────────────────────────────────
print("=" * 55)
print("S11 — Formación en V con anclas virtuales (8 agentes)")
print("=" * 55)

history = np.zeros((N_ITERS + 1, N, 2))
history[0] = X0
X = X0.copy()

for k in range(N_ITERS):
    X[0], _, _ = waypoint_leader_step(X[0], 0, [tuple(GOAL)], V_LEADER,
                                       tolerance=0.05, dt=DT)

    theta_leader = implied_headings(history[:k + 1, 0, 0], history[:k + 1, 0, 1])[-1] \
        if k > 0 else 0.0
    offsets = v_offsets(N, theta_leader, ANGLE_V, D)

    r = np.zeros((N, 2))
    for i in range(1, N):
        r[i] = offsets.get(i, (0.0, 0.0))

    dX = np.zeros_like(X)
    for chain in (chain_left, chain_right):
        for i in chain:
            for j in chain:
                if i != j:
                    dX[i] += -K_FOLL * ((X[i] - r[i]) - (X[j] - r[j]))
            dX[i] += -BETA * ((X[i] - r[i]) - X[0])

    for i in range(1, N):
        X[i] = X[i] + DT * dX[i]

    history[k + 1] = X

print(f"Posición final del líder: {history[-1, 0]}")
print(f"Meta: {GOAL}")

# ── Error: cada seguidor respecto a su ancla ideal (líder + offset rotado) ──
leader_traj = history[:, 0, :]
theta_hist = implied_headings(leader_traj[:, 0], leader_traj[:, 1])

error_x = [None]
error_y = [None]
for i in range(1, N):
    ex = np.zeros(N_ITERS + 1)
    ey = np.zeros(N_ITERS + 1)
    for k in range(N_ITERS + 1):
        offs = v_offsets(N, theta_hist[k], ANGLE_V, D)
        rix, riy = offs.get(i, (0.0, 0.0))
        ideal = leader_traj[k] + np.array([rix, riy])
        ex[k] = history[k, i, 0] - ideal[0]
        ey[k] = history[k, i, 1] - ideal[1]
    error_x.append(ex)
    error_y.append(ey)

trajectories = [history[:, i, :] for i in range(N)]

# ── Visualización ─────────────────────────────────────────────────────────────
save_path = os.path.abspath(SAVE_PATH)
print(f"Guardando en: {save_path}")
assert os.path.isdir(os.path.dirname(save_path)), "Directorio destino no existe"

plot_holonomic_demo(save_path, LABELS, trajectories, A, error_x, error_y,
                     figsize=(22, 5.5))
print("Listo.")
