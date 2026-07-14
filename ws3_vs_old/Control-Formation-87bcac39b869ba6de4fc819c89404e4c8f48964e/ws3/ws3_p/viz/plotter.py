import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import to_rgba


# ─── Utilidades internas ────────────────────────────────────────────────────

def _ideal_offsets(n_followers, theta_leader, angle_v, d):
    """
    Retorna {idx: (rx, ry)} — offsets instantáneos (sin rate-limiting).
    """
    n_left  = n_followers // 2
    n_right = n_followers - n_left
    offsets = {}

    for k in range(1, n_left + 1):
        th = theta_leader + angle_v
        offsets[k] = (-k * d * math.cos(th), -k * d * math.sin(th))
    for k in range(1, n_right + 1):
        th = theta_leader - angle_v
        offsets[n_left + k] = (-k * d * math.cos(th), -k * d * math.sin(th))
    return offsets


def _robot_colors(n):
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i % 10) for i in range(n)]
    colors[0] = (0.85, 0.1, 0.1, 1.0)   # líder en rojo
    return colors


def _formation_edges(n_robots):
    """Pares (i,j) de robots conectados por aristas de la V."""
    n_followers = n_robots - 1
    n_left  = n_followers // 2
    n_right = n_followers - n_left
    edges = []
    for k in range(1, n_left + 1):
        edges.append((k - 1 if k > 1 else 0, k))
    for k in range(1, n_right + 1):
        j = n_left + k
        edges.append((n_left + k - 1 if k > 1 else 0, j))
    return edges


def _draw_formation_frame(ax, states_t, n_robots, colors,
                          arrow_len=0.3, alpha=1.0, lw=1.5):
    """
    Dibuja marcadores, flechas de orientación y líneas de la formación
    para un instante dado.  states_t[i] = (x, y, theta) del robot i.
    """
    edges = _formation_edges(n_robots)
    xs  = [states_t[i][0] for i in range(n_robots)]
    ys  = [states_t[i][1] for i in range(n_robots)]
    ths = [states_t[i][2] for i in range(n_robots)]

    # Líneas de la V
    for (a, b) in edges:
        ax.plot([xs[a], xs[b]], [ys[a], ys[b]],
                color='k', lw=lw, alpha=alpha * 0.7, zorder=2)

    # Marcadores y flechas
    for i in range(n_robots):
        mk = '*' if i == 0 else 'o'
        ms = 10  if i == 0 else 7
        ax.plot(xs[i], ys[i], mk, color=colors[i],
                markersize=ms * (0.7 if alpha < 1 else 1),
                alpha=alpha, zorder=3)
        ax.annotate('',
                    xy=(xs[i] + arrow_len * math.cos(ths[i]),
                        ys[i] + arrow_len * math.sin(ths[i])),
                    xytext=(xs[i], ys[i]),
                    arrowprops=dict(arrowstyle='->', color=colors[i],
                                   lw=1.5 * alpha, alpha=alpha))


# ─── Plot principal: trayectorias ───────────────────────────────────────────

def plot_trajectories(history, n_robots, waypoints=None, title="Trayectorias",
                      arrow_every_s=10.0, arrow_len=0.25,
                      ghost_every_s=10.0, save_path=None):
    """
    Traza las trayectorias de todos los robots en el plano x-y.

    Cada ghost_every_s segundos dibuja un "fantasma" de la formación completa
    (marcadores + líneas de la V) para visualizar la estructura en movimiento.
    """
    fig, ax = plt.subplots(figsize=(11, 8))
    colors = _robot_colors(n_robots)
    dt     = history['time'][1] - history['time'][0] if len(history['time']) > 1 else 0.1
    step_ghost = max(1, int(ghost_every_s / dt))
    n_steps    = len(history['time'])

    # Trayectorias continuas
    for i in range(n_robots):
        xs  = [s[0] for s in history['states'][i]]
        ys  = [s[1] for s in history['states'][i]]
        lbl = 'Líder (robot0)' if i == 0 else f'robot{i}'
        lw  = 2.5 if i == 0 else 1.5
        ax.plot(xs, ys, color=colors[i], linewidth=lw, label=lbl, zorder=2, alpha=0.5)
        ax.plot(xs[0],  ys[0],  'o', color=colors[i], markersize=7, zorder=3)
        ax.plot(xs[-1], ys[-1], 's', color=colors[i], markersize=6, zorder=3)

    # Fantasmas periódicos de la formación
    ghost_indices = list(range(0, n_steps, step_ghost))
    if ghost_indices[-1] != n_steps - 1:
        ghost_indices.append(n_steps - 1)
    for k in ghost_indices:
        states_t = [history['states'][i][k] for i in range(n_robots)]
        _draw_formation_frame(ax, states_t, n_robots, colors,
                              arrow_len=arrow_len, alpha=0.85, lw=1.2)

    # Waypoints del líder
    if waypoints is not None:
        pts = list(waypoints)
        wxs = pts[0::2]
        wys = pts[1::2]
        ax.plot(wxs, wys, 'k*', markersize=9, label='Waypoints', zorder=5, alpha=0.5)
        ax.plot(wxs, wys, 'k--', linewidth=0.6, alpha=0.25, zorder=1)

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(title)
    ax.legend(loc='best', fontsize=9)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)

    return fig


# ─── Plot de error de formación ─────────────────────────────────────────────

def compute_formation_errors(history, n_robots, angle_v, d):
    """
    Calcula ‖xᵢ − (x₀ + rᵢ(θ₀))‖ para cada seguidor en cada paso de tiempo.

    Retorna dict {robot_index: [error_t0, error_t1, ...]}
    """
    n_followers = n_robots - 1
    errors = {i: [] for i in range(1, n_robots)}

    for t_idx in range(len(history['time'])):
        x0, y0, theta0 = history['states'][0][t_idx]
        offsets = _ideal_offsets(n_followers, theta0, angle_v, d)

        for i in range(1, n_robots):
            xi, yi, _ = history['states'][i][t_idx]
            rix, riy  = offsets.get(i, (0.0, 0.0))
            ex = xi - (x0 + rix)
            ey = yi - (y0 + riy)
            errors[i].append(math.sqrt(ex*ex + ey*ey))

    return errors


def plot_errors(history, n_robots, angle_v, d,
                title="Error de formación", save_path=None, rho_algo=None):
    """
    Traza el error ‖eᵢ‖ de cada seguidor a lo largo del tiempo.

    rho_algo : instancia con método rho_at(t) → para graficar envolvente PPC
    """
    errors  = compute_formation_errors(history, n_robots, angle_v, d)
    times   = history['time']
    colors  = _robot_colors(n_robots)

    fig, ax = plt.subplots(figsize=(11, 5))

    for i in range(1, n_robots):
        ax.plot(times, errors[i], color=colors[i], linewidth=1.5,
                label=f'robot{i}')

    # Envolvente PPC (opcional)
    if rho_algo is not None:
        rho_vals = [rho_algo.rho_at(t) for t in times]
        ax.plot(times, rho_vals, 'k--', linewidth=2, label='ρ(t) PPC', zorder=5)

    ax.set_xlabel('Tiempo (s)')
    ax.set_ylabel('Error de formación ‖eᵢ‖ (m)')
    ax.set_title(title)
    ax.legend(loc='upper right', fontsize=9)
    ax.set_xlim(0, times[-1])
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)

    return fig


# ─── Snapshot de la formación en varios instantes ───────────────────────────

def plot_snapshots(history, n_robots, times_s, title="Snapshots de formación",
                   save_path=None):
    """
    Muestra la formación completa (marcadores + líneas de la V + flechas)
    en instantes específicos, uno por panel.
    """
    dt      = history['time'][1] - history['time'][0]
    colors  = _robot_colors(n_robots)
    n_snaps = len(times_s)

    fig, axes = plt.subplots(1, n_snaps, figsize=(4 * n_snaps, 4.5),
                             squeeze=False)
    axes = axes[0]

    for ax, t_s in zip(axes, times_s):
        idx      = min(int(t_s / dt), len(history['time']) - 1)
        t_actual = history['time'][idx]
        states_t = [history['states'][i][idx] for i in range(n_robots)]

        _draw_formation_frame(ax, states_t, n_robots, colors,
                              arrow_len=0.3, alpha=1.0, lw=1.8)

        # Etiquetas de robots
        for i in range(n_robots):
            x, y, _ = states_t[i]
            lbl = 'L' if i == 0 else f'r{i}'
            ax.annotate(lbl, (x, y), textcoords='offset points',
                        xytext=(5, 5), fontsize=7, color=colors[i])

        ax.set_title(f't = {t_actual:.0f} s', fontsize=9)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=7)

    fig.suptitle(title, fontsize=11)
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)

    return fig
