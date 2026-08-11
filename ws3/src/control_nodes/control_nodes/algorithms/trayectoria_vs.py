import math


class TrayectoriaVS:
    """
    Trayectoria paramétrica del centroide de la estructura virtual.
    Curva Catmull-Rom que pasa exactamente por los waypoints dados.
    Velocidad constante a lo largo del arco (parametrización por longitud de arco).
    El punto inicial es waypoints[0].

    Módulo de navegación intercambiable: expone x, y, theta, step(dt) y done,
    de forma que el nodo que lo usa (trayectoria_vs_node) no depende de la
    estrategia de generación de trayectoria concreta. Para probar una
    estrategia distinta (p. ej. splines vs. otra interpolación), basta con
    escribir una clase alterna con la misma interfaz en este paquete.
    """

    def __init__(self, waypoints, v_max=0.25):
        # waypoints: lista de (x, y), mínimo 2 puntos
        self._pts  = [tuple(p) for p in waypoints]
        self.v_max = v_max
        self._u    = 0.0      # parámetro global ∈ [0, N-1]
        self.done  = False

        pos, tang  = self._eval(0.0)
        self.x     = pos[0]
        self.y     = pos[1]
        self.theta = math.atan2(tang[1], tang[0]) if math.hypot(*tang) > 1e-6 else 0.0

    # ------------------------------------------------------------------
    # Catmull-Rom
    # ------------------------------------------------------------------

    def _control_points(self, seg):
        pts = self._pts
        n   = len(pts)
        p1  = pts[seg]
        p2  = pts[seg + 1]
        # Phantom points en extremos por reflexión para mantener la tangente
        p0 = pts[seg - 1] if seg > 0     else (2*p1[0] - p2[0], 2*p1[1] - p2[1])
        p3 = pts[seg + 2] if seg + 2 < n else (2*p2[0] - p1[0], 2*p2[1] - p1[1])
        return p0, p1, p2, p3

    def _catmull_rom(self, p0, p1, p2, p3, t):
        t2, t3 = t*t, t*t*t

        x = 0.5 * (2*p1[0]
                   + (-p0[0] + p2[0]) * t
                   + (2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * t2
                   + (-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * t3)
        y = 0.5 * (2*p1[1]
                   + (-p0[1] + p2[1]) * t
                   + (2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * t2
                   + (-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * t3)

        # Tangente (derivada respecto a t) → da el heading
        dx = 0.5 * ((-p0[0] + p2[0])
                    + 2*(2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * t
                    + 3*(-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * t2)
        dy = 0.5 * ((-p0[1] + p2[1])
                    + 2*(2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * t
                    + 3*(-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * t2)

        return (x, y), (dx, dy)

    def _eval(self, u):
        n   = len(self._pts)
        u   = max(0.0, min(u, float(n - 1) - 1e-9))
        seg = min(int(u), n - 2)
        t   = u - seg
        return self._catmull_rom(*self._control_points(seg), t)

    # ------------------------------------------------------------------
    # Avance
    # ------------------------------------------------------------------

    def step(self, dt):
        if self.done:
            return

        _, tang = self._eval(self._u)
        speed   = math.hypot(tang[0], tang[1])
        # Parametrización por longitud de arco: du = v * dt / |tangente|
        du = (self.v_max * dt / speed) if speed > 1e-6 else (self.v_max * dt)

        self._u += du
        n = len(self._pts)
        if self._u >= n - 1.0:
            self._u = float(n - 1)
            self.done = True

        pos, tang = self._eval(self._u)
        self.x = pos[0]
        self.y = pos[1]
        if math.hypot(tang[0], tang[1]) > 1e-6:
            self.theta = math.atan2(tang[1], tang[0])

    @property
    def n_waypoints(self):
        return len(self._pts)


# ------------------------------------------------------------------
# Estrategia de formación (offsets de referencia de cada seguidor
# respecto al centroide de la estructura virtual).
#
# Módulo intercambiable, igual que TrayectoriaVS: hoy es una V rígida
# (ángulo + separación fijos, rotada según el heading del centroide),
# pero el nodo que consume esto (trayectoria_vs_node) no debe conocer
# el detalle — solo llama a compute_formation_offsets() y recibe el
# set completo de posiciones de referencia. Para probar otra estrategia
# de formación basta con escribir una función alterna con la misma
# firma en este módulo.
# ------------------------------------------------------------------

def compute_formation_offsets(n_robots, theta_v, angle_v, d):
    """Devuelve {índice: (ox, oy)} — offset de cada seguidor respecto al
    centroide, en el marco global, para una formación en V rígida."""
    n_followers = n_robots - 1
    n_left  = n_followers // 2
    n_right = n_followers - n_left

    def rot(theta, vx, vy):
        c, s = math.cos(theta), math.sin(theta)
        return c*vx - s*vy, s*vx + c*vy

    offs = {0: (0.0, 0.0)}
    for k in range(1, n_left + 1):
        offs[k] = rot(theta_v + angle_v, -k*d, 0.0)
    for k in range(1, n_right + 1):
        offs[n_left + k] = rot(theta_v - angle_v, -k*d, 0.0)
    return offs
