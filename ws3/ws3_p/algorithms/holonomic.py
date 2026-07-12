import numpy as np


def consensus_step(X, A, dt=1.0):
    """Un paso de consenso de integrador simple: xi += dt * sum_j a_ij*(xj - xi)."""
    N = X.shape[0]
    dX = np.zeros_like(X)
    for i in range(N):
        for j in range(N):
            if A[i, j] != 0.0:
                dX[i] += A[i, j] * (X[j] - X[i])
    return X + dt * dX


def simulate_consensus(x0, A, n_iters, dt=1.0):
    """Corre consensus_step n_iters veces. Retorna history (n_iters+1, N, 2)."""
    N = x0.shape[0]
    history = np.zeros((n_iters + 1, N, 2))
    history[0] = x0
    X = x0.copy()
    for k in range(n_iters):
        X = consensus_step(X, A, dt)
        history[k + 1] = X
    return history


def implied_headings(x, y):
    """theta[i] = atan2(dy,dx) del movimiento local; mantiene el último heading
    válido cuando el desplazamiento es ~0 (agente detenido). Si nunca se mueve,
    retorna ceros."""
    n = len(x)
    theta = np.zeros(n)
    if n < 2:
        return theta

    dx = np.diff(x)
    dy = np.diff(y)
    moved = np.hypot(dx, dy) > 1e-9

    last = None
    computed = np.zeros(n - 1)
    valid = np.zeros(n - 1, dtype=bool)
    for i in range(n - 1):
        if moved[i]:
            last = np.arctan2(dy[i], dx[i])
        if last is not None:
            computed[i] = last
            valid[i] = True

    # relleno hacia atrás para los primeros pasos si el primer movimiento
    # válido aparece más adelante
    first_valid = np.argmax(valid) if valid.any() else None
    if first_valid is not None and first_valid > 0:
        computed[:first_valid] = computed[first_valid]

    theta[:-1] = computed
    theta[-1] = computed[-1] if n > 1 else 0.0
    return theta


def waypoint_leader_step(x, wp_index, waypoints, v_max, tolerance=0.05, dt=1.0):
    """Líder proporcional simple: avanza hacia waypoints[wp_index] a v_max.
    Si ya llegó (dentro de tolerance), avanza el índice (sin ciclar más allá
    del último). Retorna (x_nuevo, wp_index_nuevo, llegó_a_la_ultima: bool)."""
    goal = np.array(waypoints[wp_index])
    err = goal - x
    dist = np.linalg.norm(err)

    reached_last = False
    if dist < tolerance:
        if wp_index < len(waypoints) - 1:
            wp_index += 1
            goal = np.array(waypoints[wp_index])
            err = goal - x
            dist = np.linalg.norm(err)
        else:
            reached_last = True

    if dist > 1e-9:
        step = min(v_max * dt, dist)
        x_new = x + step * err / dist
    else:
        x_new = x.copy()

    return x_new, wp_index, reached_last


def rotate(theta, vx, vy):
    c, s = np.cos(theta), np.sin(theta)
    return c * vx - s * vy, s * vx + c * vy


def v_offsets(n_robots, theta_leader, angle_v, d):
    """Offsets ideales (índice de seguidor -> (rx, ry)) para una V centrada en
    el líder, rotada según theta_leader. Mismo criterio que
    ws3_p/viz/plotter.py:_ideal_offsets."""
    n_followers = n_robots - 1
    n_left = n_followers // 2
    n_right = n_followers - n_left
    offsets = {}
    for k in range(1, n_left + 1):
        th = theta_leader + angle_v
        offsets[k] = rotate(th, -k * d, 0.0)
    for k in range(1, n_right + 1):
        th = theta_leader - angle_v
        offsets[n_left + k] = rotate(th, -k * d, 0.0)
    return offsets
