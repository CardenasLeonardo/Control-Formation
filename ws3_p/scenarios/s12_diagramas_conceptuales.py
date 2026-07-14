#!/usr/bin/env python3
"""
S12 — Diagramas conceptuales (no simulaciones): apoyo visual para la notación
de cada protocolo de consenso mostrado en Sections/9.tex. Estilo diagrama
técnico: ejes X-Y, agentes como círculos huecos, relación bilateral (flecha
doble) vs unilateral (flecha simple), radio de comunicación como círculo
punteado. Solo negro, sin grid ni leyenda.
"""
import os
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family":      "serif",
    "font.serif":       ["Times New Roman"],
    "mathtext.fontset": "stix",
    "font.size":        13,
})

PICS_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'documentacion', 'docs', 'seminario_iv', 'Pics'
)

NODE_R = 0.28


def _new_ax(figsize=(6.5, 5.5)):
    fig, ax = plt.subplots(figsize=figsize)
    ax.set_aspect('equal')
    ax.axis('off')
    return fig, ax


def _axes(ax, origin, length=1.0, length_y=None, fontsize=16):
    ox, oy = origin
    ly = length_y if length_y is not None else length
    ax.annotate('', xy=(ox + length, oy), xytext=(ox, oy),
                arrowprops=dict(arrowstyle='-|>', color='black', lw=1.4))
    ax.annotate('', xy=(ox, oy + ly), xytext=(ox, oy),
                arrowprops=dict(arrowstyle='-|>', color='black', lw=1.4))
    ax.annotate('X', (ox + length + 0.15, oy), ha='left', va='center', fontsize=fontsize)
    ax.annotate('Y', (ox, oy + ly + 0.15), ha='center', va='bottom', fontsize=fontsize)


def _point(ax, xy, label=None, offset=(8, 8), fontsize=18):
    """Punto (no nodo/círculo grande): un marcador pequeño con su etiqueta al lado."""
    ax.plot(xy[0], xy[1], 'o', color='black', markersize=5.5, zorder=5)
    if label:
        ax.annotate(label, xy, xytext=offset, textcoords='offset points',
                    ha='left', va='bottom', fontsize=fontsize, zorder=6)


def _vector(ax, p, q, label=None, label_pos=0.55, shrink=0.12):
    """Flecha de una sola punta que sale de p hacia q (vector, no relación)."""
    p, q = np.array(p, dtype=float), np.array(q, dtype=float)
    d = q - p
    u = d / np.linalg.norm(d)
    a = p + u * shrink
    b = q - u * shrink
    ax.annotate('', xy=tuple(b), xytext=tuple(a),
                arrowprops=dict(arrowstyle='-|>', color='black', lw=1.2,
                                mutation_scale=14), zorder=3)
    if label:
        mid = a + (b - a) * label_pos
        perp = np.array([-u[1], u[0]]) * 0.28
        ax.annotate(label, mid + perp, ha='center', va='center', fontsize=12, zorder=6)


def _node(ax, xy, label=None, r=NODE_R, marker='o'):
    if marker == 'o':
        ax.add_patch(plt.Circle(xy, r, facecolor='white', edgecolor='black',
                                 linewidth=1.4, zorder=5))
    elif marker == 'D':
        s = r * 1.3
        pts = np.array([[0, s], [s, 0], [0, -s], [-s, 0]]) + np.array(xy)
        ax.add_patch(plt.Polygon(pts, closed=True, facecolor='white',
                                  edgecolor='black', linewidth=1.4, zorder=5))
    if label:
        ax.annotate(label, xy, xytext=(0, r + 0.22), textcoords='offset points',
                    ha='center', va='bottom', fontsize=13, zorder=6)


def _arrow(ax, p, q, bilateral=False, label=None, label_pos=0.5, style='-', lw=None):
    p, q = np.array(p, dtype=float), np.array(q, dtype=float)
    d = q - p
    u = d / np.linalg.norm(d)
    a = p + u * NODE_R
    b = q - u * NODE_R
    arrowstyle = '<|-|>' if bilateral else '-|>'
    ax.annotate('', xy=tuple(b), xytext=tuple(a),
                arrowprops=dict(arrowstyle=arrowstyle, color='black',
                                lw=lw or (1.4 if bilateral else 1.1),
                                linestyle=style, mutation_scale=14),
                zorder=3)
    if label:
        mid = a + (b - a) * label_pos
        perp = np.array([-u[1], u[0]]) * 0.3
        ax.annotate(label, mid + perp, ha='center', va='center', fontsize=12, zorder=6)


def _radius(ax, center, r, label=None):
    ax.add_patch(plt.Circle(center, r, facecolor='none', edgecolor='black',
                             linewidth=1.0, linestyle=(0, (5, 4)), zorder=1))
    if label:
        ax.annotate(label, (center[0], center[1] - r - 0.25),
                    ha='center', va='top', fontsize=11)


def _save(fig, ax, name, points, pad=1.0):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad - 0.6, max(ys) + pad)
    fig.tight_layout()
    path = os.path.join(PICS_DIR, name)
    fig.savefig(path, dpi=150, bbox_inches='tight', pad_inches=0.05)
    plt.close(fig)
    print(f"Guardado: {path}")


# ── Layout compartido: diamante (i=izquierda, b=arriba, c=derecha, a=abajo) ─
DIAMOND = {'i': (0.0, 0.0), 'b': (1.6, 1.4), 'c': (3.2, 0.0), 'a': (1.6, -1.4)}
ORIGIN = (-1.5, -2.3)


# ── 1. Vecindario N_i: el estado de x_i depende de sus vecinos dentro del
#    radio; el punto fuera del radio no participa ──────────────────────────
fig, ax = _new_ax(figsize=(6, 5))
CENTER = (0.0, 0.0)
NEI = {
    'j': (0.65, 1.15),
    'k': (1.15, -0.95),
    'a': (-1.05, -0.20),
}
OUT = (2.35, 0.85)   # fuera del radio: no es vecino
R = 1.75

# Ejes que enmarcan el círculo: origen en su esquina inferior izquierda,
# largo = diámetro, para dar la impresión de estar proporcionados al radio.
AXIS_ORIGIN = (CENTER[0] - R, CENTER[1] - R)
_axes(ax, AXIS_ORIGIN, length=2 * R, fontsize=18)
_radius(ax, CENTER, R)
ax.annotate(r'$\mathcal{N}_i$', (CENTER[0], CENTER[1] + R + 0.15),
            ha='center', va='bottom', fontsize=20)
for k in NEI:
    _vector(ax, CENTER, NEI[k])
ax.annotate(r'$x_i$', CENTER, xytext=(0, -16), textcoords='offset points',
            ha='center', va='top', fontsize=18, zorder=6)
_point(ax, CENTER)
for k, lbl in [('j', r'$x_j$'), ('k', r'$x_k$'), ('a', r'$x_a$')]:
    _point(ax, NEI[k], lbl)
_point(ax, OUT, r'$x_l$')
_save(fig, ax, 'diagrama_grafo_completo.png',
      list(NEI.values()) + [OUT, CENTER, AXIS_ORIGIN,
                             (CENTER[0], CENTER[1] + R)], pad=1.0)

# ── 2. Grafo parcialmente conexo (estrella en x_i) ─────────────────────────
fig, ax = _new_ax()
_axes(ax, ORIGIN)
for k in ('b', 'c', 'a'):
    _arrow(ax, DIAMOND['i'], DIAMOND[k], bilateral=True)
_arrow(ax, DIAMOND['i'], DIAMOND['b'], bilateral=True, label=r'$a_{ij}$')
mid = (np.array(DIAMOND['b']) + np.array(DIAMOND['c'])) / 2
ax.plot([DIAMOND['b'][0], DIAMOND['c'][0]], [DIAMOND['b'][1], DIAMOND['c'][1]],
        color='black', lw=0.9, linestyle=':', zorder=1)
ax.annotate(r'$a_{jk}=0$', mid + np.array([0.35, 0.25]), ha='left', va='center', fontsize=11)
for k, lbl in [('i', r'$x_i$'), ('b', r'$x_j$'), ('c', r'$x_k$'), ('a', r'$x_l$')]:
    _node(ax, DIAMOND[k], lbl)
_save(fig, ax, 'diagrama_grafo_disperso.png',
      list(DIAMOND.values()) + [ORIGIN])

# ── 3. Agente aislado (fuera del radio de comunicación) ────────────────────
fig, ax = _new_ax(figsize=(7.5, 5.5))
GROUP = {'i': (0.0, 0.0), 'b': (1.5, 1.2), 'c': (1.5, -1.2)}
ISOL = (4.3, 0.0)
_axes(ax, (-1.6, -2.3))
_arrow(ax, GROUP['i'], GROUP['b'], bilateral=True, label=r'$a_{ij}$')
_arrow(ax, GROUP['i'], GROUP['c'], bilateral=True)
_radius(ax, GROUP['i'], 2.3, label='radio de comunicación')
_arrow(ax, ISOL, GROUP['c'], bilateral=False, label=r'$a_{ck}$', label_pos=0.35)
for k, lbl in [('i', r'$x_i$'), ('b', r'$x_j$'), ('c', r'$x_k$')]:
    _node(ax, GROUP[k], lbl)
_node(ax, ISOL, r'$x_l$')
ax.annotate('(aislado)', (ISOL[0], ISOL[1] - 0.55), ha='center', va='top', fontsize=11)
ax.annotate(r'$[A]_{l,:}=\mathbf{0}^T$', (ISOL[0], ISOL[1] - 1.0),
            ha='center', va='top', fontsize=11)
_save(fig, ax, 'diagrama_grafo_aislado.png',
      list(GROUP.values()) + [ISOL, (-1.6, -2.3)], pad=1.1)

# ── 4. Líder-seguidor ────────────────────────────────────────────────────────
fig, ax = _new_ax()
LEADER = (1.6, 1.6)
FOLL = {'i': (0.0, -1.2), 'j': (1.6, -1.8), 'k': (3.2, -1.2)}
_axes(ax, (-1.6, -2.6))
for k in ('i', 'j', 'k'):
    _arrow(ax, LEADER, FOLL[k], bilateral=False)
_arrow(ax, LEADER, FOLL['i'], bilateral=False, label=r'$\beta$', label_pos=0.55)
_arrow(ax, FOLL['i'], FOLL['j'], bilateral=True, label=r'$a_{ij}$')
_arrow(ax, FOLL['j'], FOLL['k'], bilateral=True)
_node(ax, LEADER, r'$x_L$')
for k, lbl in [('i', r'$x_i$'), ('j', r'$x_j$'), ('k', r'$x_k$')]:
    _node(ax, FOLL[k], lbl)
ax.annotate(r'$[A]_{L,:}=\mathbf{0}^T$', (LEADER[0], LEADER[1] + 0.55),
            ha='center', va='bottom', fontsize=12)
_save(fig, ax, 'diagrama_lider_seguidor.png',
      [LEADER] + list(FOLL.values()) + [(-1.6, -2.6)])

# ── 5. Formación en V con ancla virtual ────────────────────────────────────
fig, ax = _new_ax(figsize=(7, 5))
LEADER = (0.0, 0.0)
IDEAL = (3.0, 1.4)      # x_L + r_i (meta de formación)
ACTUAL = (3.6, 0.2)     # x_i real
_axes(ax, (-1.6, -1.8))
ax.plot([LEADER[0], IDEAL[0]], [LEADER[1], IDEAL[1]], color='black',
        lw=1.0, linestyle=':', zorder=2)
ax.annotate(r'$r_i$', (1.5, 0.9), ha='center', va='center', fontsize=12)
_arrow(ax, LEADER, ACTUAL, bilateral=False, label=r'$\beta$', label_pos=0.4)
ax.plot([IDEAL[0], ACTUAL[0]], [IDEAL[1], ACTUAL[1]], color='black',
        lw=1.0, linestyle=(0, (1, 2)), zorder=2)
ax.annotate('error', (IDEAL[0] + 0.55, (IDEAL[1] + ACTUAL[1]) / 2), ha='left', va='center', fontsize=10)
_node(ax, LEADER, r'$x_L$')
_node(ax, IDEAL, r'$x_L{+}r_i$', marker='D', r=NODE_R * 1.5)
_node(ax, ACTUAL, r'$x_i$')
_save(fig, ax, 'diagrama_formacion_v.png', [LEADER, IDEAL, ACTUAL, (-1.6, -1.8)], pad=1.1)

print("Listo.")
