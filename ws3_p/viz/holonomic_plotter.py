import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family":      "serif",
    "font.serif":       ["Times New Roman"],
    "mathtext.fontset": "stix",
    "font.size":        14,
})

COLORS = plt.cm.tab10.colors


def _draw_triangles(ax, x, y, color, d_step=0.4, size=0.25):
    """Triángulos sólidos cada d_step metros, heading implícito del movimiento local."""
    if len(x) < 2:
        return
    cumul = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    total = cumul[-1]
    if total < d_step:
        return

    from algorithms.holonomic import implied_headings
    theta = implied_headings(x, y)

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


def plot_graph_panel(ax, positions, A, labels):
    """Panel 'Condición inicial y conexiones' — nodos + aristas dirigidas reales de A."""
    n = len(labels)
    for i in range(n):
        c = COLORS[i % len(COLORS)]
        ax.scatter(positions[i, 0], positions[i, 1], s=140, color=c, zorder=5, label=labels[i])
        ax.annotate(labels[i], (positions[i, 0], positions[i, 1]),
                    textcoords="offset points", xytext=(8, 4), fontsize=9)

    for i in range(n):
        for j in range(i + 1, n):
            mutual = (A[i, j] != 0.0) and (A[j, i] != 0.0)
            if mutual:
                # Conexión simétrica: una sola línea, sin punta (no hay
                # dirección real que mostrar y evita duplicar flechas).
                ax.plot([positions[i, 0], positions[j, 0]],
                        [positions[i, 1], positions[j, 1]],
                        color='gray', alpha=0.35, lw=1.0, zorder=1)
            elif A[i, j] != 0.0 or A[j, i] != 0.0:
                # Asimétrica: una sola flecha, del que "presta" el valor
                # hacia el que depende de él.
                src, dst = (j, i) if A[i, j] != 0.0 else (i, j)
                ax.annotate(
                    '', xy=(positions[dst, 0], positions[dst, 1]),
                    xytext=(positions[src, 0], positions[src, 1]),
                    arrowprops=dict(arrowstyle='-|>', color='gray',
                                    alpha=0.6, lw=1.3, shrinkA=10, shrinkB=10,
                                    mutation_scale=14)
                )

    ax.set_title("Condición inicial y conexiones")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True)
    ax.set_aspect('equal', adjustable='datalim')
    ax.legend(fontsize=8)


def plot_trajectory_panel(ax, trajectories, labels, extra_markers=None,
                           tri_d_step=0.4, tri_size=0.25):
    """Panel 'Trayectoria de los robots' — línea + inicio hueco + fin estrella + triángulos."""
    for i, label in enumerate(labels):
        c = COLORS[i % len(COLORS)]
        x = trajectories[i][:, 0]
        y = trajectories[i][:, 1]

        ax.plot(x, y, color=c, linewidth=1.8, label=label, zorder=2)
        ax.scatter(x[0], y[0], s=90, facecolors='white',
                   edgecolors=c, linewidths=2, zorder=4)
        ax.scatter(x[-1], y[-1], color=c, marker='*',
                   s=220, zorder=5, edgecolors='black', linewidths=0.5)
        _draw_triangles(ax, x, y, c, d_step=tri_d_step, size=tri_size)

    if extra_markers:
        for m in extra_markers:
            ax.scatter(m['x'], m['y'], marker='x', color=m['color'],
                       s=m.get('s', 90), linewidths=2.2, zorder=6,
                       label=m.get('label'))

    ax.set_title("Trayectoria de los robots")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True)
    ax.set_aspect('equal', adjustable='datalim')
    ax.legend(fontsize=8)


def plot_error_panel(ax, errors, labels, ylabel, title):
    """Panel genérico de error vs iteración (usado para 'Error en x' y 'Error en y')."""
    ref = next(e for e in errors if e is not None)
    iters = np.arange(ref.shape[0])
    for i, label in enumerate(labels):
        if errors[i] is None:
            continue
        c = COLORS[i % len(COLORS)]
        style = '--' if labels[i].startswith('líder') else '-'
        ax.plot(iters, errors[i], color=c, linewidth=1.6, linestyle=style, label=label)

    ax.axhline(0.0, color='black', linewidth=0.6, alpha=0.3, zorder=1)
    ax.set_title(title)
    ax.set_xlabel("Iteraciones")
    ax.set_ylabel(ylabel)
    ax.grid(True)
    ax.legend(fontsize=8)


def plot_holonomic_demo(save_path, labels, trajectories, A,
                         error_x, error_y, extra_markers=None,
                         figsize=(20, 5), tri_d_step=0.4, tri_size=0.25):
    """
    Arma la figura de 4 paneles y la guarda.

    trajectories : lista de arrays (T,2), uno por robot (índice == labels[i])
    A            : matriz de adyacencia (N,N) para el panel de conexiones
    error_x/y    : listas del mismo largo que labels; error_x[i] es un array
                   (T,) o None si ese robot no aplica (ej. líder sin error propio)
    tri_d_step/tri_size : controlan el tamaño/espaciado de los triángulos de
                   heading — reducir en trayectorias cortas para que no dominen.
    """
    positions0 = np.array([traj[0] for traj in trajectories])

    fig, axes = plt.subplots(1, 4, figsize=figsize)

    plot_graph_panel(axes[0], positions0, A, labels)
    plot_trajectory_panel(axes[1], trajectories, labels, extra_markers,
                           tri_d_step=tri_d_step, tri_size=tri_size)
    plot_error_panel(axes[2], error_x, labels, "Error en x (m)", "Error en x")
    plot_error_panel(axes[3], error_y, labels, "Error en y (m)", "Error en y")

    fig.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)
    return fig
