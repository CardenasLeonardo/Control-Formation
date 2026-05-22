"""
Generates a single-robot trajectory plot:
  start: (0, 0), heading east (theta = 0)
  end:   (-5, -5)
with orientation triangles, Times New Roman, English labels.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg

matplotlib.rcParams.update({
    "font.family":      "serif",
    "font.serif":       ["Times New Roman"],
    "mathtext.fontset": "stix",
    "font.size":        11,
})


# ── Cubic Bezier trajectory ───────────────────────────────────────────────────
# P0 = start, P3 = end.  P1 steers the initial direction (east = +x).
P0 = np.array([0.0,  0.0])
P1 = np.array([2.5,  0.0])   # initial direction: east
P2 = np.array([-5.0, -1.5])  # approach from above
P3 = np.array([-5.0, -5.0])

N = 500
t_arr = np.linspace(0, 1, N)

# Bezier position
x = ((1-t_arr)**3 * P0[0] + 3*(1-t_arr)**2*t_arr * P1[0]
     + 3*(1-t_arr)*t_arr**2 * P2[0] + t_arr**3 * P3[0])
y = ((1-t_arr)**3 * P0[1] + 3*(1-t_arr)**2*t_arr * P1[1]
     + 3*(1-t_arr)*t_arr**2 * P2[1] + t_arr**3 * P3[1])

# Bezier derivative → heading angle
dx = (3*(1-t_arr)**2 * (P1[0]-P0[0]) + 6*(1-t_arr)*t_arr * (P2[0]-P1[0])
      + 3*t_arr**2 * (P3[0]-P2[0]))
dy = (3*(1-t_arr)**2 * (P1[1]-P0[1]) + 6*(1-t_arr)*t_arr * (P2[1]-P1[1])
      + 3*t_arr**2 * (P3[1]-P2[1]))
theta = np.arctan2(dy, dx)


# ── Triangle helper ───────────────────────────────────────────────────────────
def draw_triangles(ax, x, y, theta, color, d_step=0.6, size=0.22):
    cumul = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    total = cumul[-1]
    if total < d_step:
        return
    for d_target in np.arange(d_step / 2, total, d_step):
        idx = int(np.clip(np.searchsorted(cumul, d_target, side='right') - 1,
                          0, len(x) - 1))
        xi, yi, ti = x[idx], y[idx], theta[idx]
        pts = np.array([
            [ size,         0.0        ],
            [-size * 0.5,   size * 0.45],
            [-size * 0.5,  -size * 0.45],
        ])
        c_t, s_t = np.cos(ti), np.sin(ti)
        R = np.array([[c_t, -s_t], [s_t, c_t]])
        rotated = (R @ pts.T).T + np.array([xi, yi])
        ax.fill(rotated[:, 0], rotated[:, 1], color=color, zorder=3, alpha=0.9)


# ── Plot ──────────────────────────────────────────────────────────────────────
COLOR = '#1f77b4'

fig = Figure(figsize=(5.5, 5.5))
FigureCanvasAgg(fig)
ax = fig.add_subplot(1, 1, 1)

ax.plot(x, y, color=COLOR, linewidth=2.0, zorder=2)

# Initial position — hollow circle
ax.scatter(x[0], y[0], s=100, facecolors='white',
           edgecolors=COLOR, linewidths=2, zorder=4)

# Final position — star
ax.scatter(x[-1], y[-1], color=COLOR, marker='*',
           s=260, zorder=5, edgecolors='black', linewidths=0.5)

draw_triangles(ax, x, y, theta, COLOR)

ax.set_title("Robot trajectory")
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.grid(True)
ax.set_aspect('equal')
fig.tight_layout()

out = "figures_trayectoria_simple/trajectory.pdf"
fig.savefig(out, bbox_inches="tight")
print(f"Saved: {out}")
