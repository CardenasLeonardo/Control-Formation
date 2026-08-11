import math


class Deadlock:

    def __init__(self,
                 speed_thr=0.005,
                 distance_tol=0.20,
                 v_z_margin=0.05,
                 vertex_smoothing=0.4,
                 min_track_similarity=0.8):

        self.speed_thr     = speed_thr
        self.distance_tol  = distance_tol
        # Margen que debe bajar V(z) respecto a v_z_previous para
        # confirmar la salida de deadlock (evita oscilar entrando y
        # saliendo por ruido numérico de un tick a otro).
        self.v_z_margin    = v_z_margin
        # Filtro exponencial (0,1] al perseguir el vértice de la arista:
        # 1.0 = saltar directo al vértice cada tick (bang-bang); valores
        # menores promedian con el comando previo para que el cambio de
        # vértice tick a tick no sea un salto brusco.
        self.vertex_smoothing = vertex_smoothing
        # Similitud angular mínima (coseno del ángulo entre normales)
        # para aceptar una restricción como "la evolución" de la
        # rastreada (ver track_constraint). 1.0 = misma dirección exacta;
        # 0.8 ≈ hasta 37° de giro entre ticks consecutivos.
        self.min_track_similarity = min_track_similarity

        self.is_deadlocked = False

        # --- Estrategia de bordeo: estado expuesto para graficar ---
        self.selected_constraint = None   # (A, B, C) rastreada
        self.selected_direction  = 0      # 1 = sR, -1 = sL, 0 = sin definir
        self.selected_vertex     = None   # vértice perseguido (sin suavizar)
        self.other_vertex        = None   # otro extremo de la misma arista
        self._smoothed_command   = None   # último comando YA suavizado

        self.v_z          = 0.0   # V(z) actual
        self.v_z_previous = 0.0   # V(z) capturado al ENTRAR en deadlock

    # --------------------------------------------------
    # LYAPUNOV
    # --------------------------------------------------

    @staticmethod
    def lyapunov(a, alpha):
        """V(z) = 1/2 a^2 + 1/2 alpha^2 — error de posición y de
        orientación combinados."""
        a     = 0.0 if a     is None else a
        alpha = 0.0 if alpha is None else alpha
        return 0.5 * a * a + 0.5 * alpha * alpha

    # --------------------------------------------------
    # DETECTAR DEADLOCK_IN
    # --------------------------------------------------

    def detect_deadlock_in(self, u_star, distance=None):
        """Condición de ENTRADA a deadlock: ‖u*‖ casi cero y (si se
        conoce la distancia a la meta) el robot todavía no llegó."""
        speed     = math.hypot(u_star[0], u_star[1])
        near_goal = distance is not None and distance <= self.distance_tol

        self.is_deadlocked = speed < self.speed_thr and not near_goal
        return self.is_deadlocked

    # --------------------------------------------------
    # DETECTAR DEADLOCK_OUT
    # --------------------------------------------------

    def detect_deadlock_out(self, distance=None):
        """Condición de SALIDA de deadlock, independiente de la
        estrategia de bordeo: el robot ya llegó a la meta, o V(z) bajó
        más de v_z_margin respecto al valor capturado al ENTRAR
        (v_z_previous) — es decir, en el camino de bordeo se encontró un
        punto genuinamente más cerca de la meta y libre. El margen evita
        confirmar la salida por una mejora minúscula/ruido de un tick."""
        near_goal = distance is not None and distance <= self.distance_tol
        improved  = (self.v_z_previous - self.v_z) > self.v_z_margin

        cleared = near_goal or improved
        if cleared:
            self.is_deadlocked = False

        return cleared

    # --------------------------------------------------
    # SELECCIONAR RESTRICCIÓN
    # --------------------------------------------------

    def select_constraint(self, constraints, vertices, tol=1e-4):
        """Restricción bloqueante: la de menor distancia perpendicular
        al origen, entre las que realmente forman una arista del
        polígono (dos de sus vértices sobre la recta A*v + B*w = C)."""
        active = []
        for A, B, C in constraints:
            on_line = 0
            for v, w in vertices:
                if abs(A * v + B * w - C) < tol:
                    on_line += 1
            if on_line >= 2:
                active.append((A, B, C))

        pool = active if active else constraints
        if not pool:
            return None

        best         = None
        best_distance = None
        for A, B, C in pool:
            distance = abs(C) / math.hypot(A, B)
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best          = (A, B, C)

        return best

    # --------------------------------------------------
    # SELECCIONAR DIRECCIÓN
    # --------------------------------------------------

    def select_direction(self, constraint):
        """Dirección de bordeo a partir del ángulo de la restricción
        seleccionada: B >= 0 -> 1 (sR), B < 0 -> -1 (sL)."""
        if constraint is None:
            return 1
        _A, B, _C = constraint
        return 1 if B >= 0 else -1

    # --------------------------------------------------
    # SELECCIONAR VÉRTICE
    # --------------------------------------------------

    def select_vertex(self, vertices, constraint, direction, tol=1e-4):
        """Vértice de la arista definida por constraint más al frente
        en la dirección elegida — se usa solo para graficar hacia dónde
        apunta el bordeo. El otro extremo de la misma arista queda en
        self.other_vertex."""
        if constraint is None:
            self.other_vertex = None
            return None

        A, B, C = constraint
        on_edge = [(v, w) for v, w in vertices if abs(A * v + B * w - C) < tol]
        if len(on_edge) < 2:
            self.other_vertex = None
            return None

        v_hi = max(on_edge, key=lambda p: p[1])
        v_lo = min(on_edge, key=lambda p: p[1])

        selected, other  = (v_lo, v_hi) if direction == 1 else (v_hi, v_lo)
        self.other_vertex = other
        return selected

    # --------------------------------------------------
    # RASTREAR LA MISMA RESTRICCIÓN ENTRE TICKS
    # --------------------------------------------------

    def track_constraint(self, tracked, constraints):
        """Restricción, entre las del tick actual, que corresponde a la
        EVOLUCIÓN de `tracked` (la fijada al entrar en deadlock) — no la
        "más bloqueante" recalculada desde cero. Se compara por
        similitud angular: el producto punto entre las normales (A,B)
        de cada restricción, normalizado, da el coseno del ángulo entre
        ellas. C se ignora a propósito — cambia mucho solo porque el
        robot se acerca o aleja del obstáculo, no porque sea uno
        distinto; la dirección de la normal en cambio varía poco y de
        forma predecible tick a tick si es la misma pared/obstáculo.
        Si ninguna restricción actual queda lo bastante alineada
        (similitud < min_track_similarity), se repite `tracked` tal
        cual, en vez de saltar de identidad."""
        if tracked is None or not constraints:
            return tracked

        tA, tB, _tC = tracked
        t_norm = math.hypot(tA, tB)
        if t_norm < 1e-12:
            return tracked

        best            = None
        best_similarity = None
        for A, B, C in constraints:
            norm = math.hypot(A, B)
            if norm < 1e-12:
                continue
            similarity = (A * tA + B * tB) / (norm * t_norm)   # cos(ángulo)
            if best_similarity is None or similarity > best_similarity:
                best_similarity = similarity
                best            = (A, B, C)

        if best is not None and best_similarity >= self.min_track_similarity:
            return best
        return tracked

    # --------------------------------------------------
    # COMANDO: PERSEGUIR EL VÉRTICE, SUAVIZADO
    # --------------------------------------------------

    def smooth_vertex_command(self, target_vertex):
        """El vértice objetivo se recalcula cada tick sobre el polígono
        actual (select_vertex ya lo hizo, con la arista rastreada) — a
        medida que el robot avanza y el LIDAR cambia, ese vértice se va
        moviendo, y con él el comando. Un vértice SÍ tiene magnitud real
        (está en el borde del polígono, lejos del origen), a diferencia
        de proyectar sobre la propia recta que causó el atasco. El
        filtro exponencial evita que el cambio de un vértice a otro se
        sienta como un salto brusco."""
        if target_vertex is None:
            return self._smoothed_command

        if self._smoothed_command is None:
            self._smoothed_command = target_vertex
        else:
            beta = self.vertex_smoothing
            prev_v, prev_w = self._smoothed_command
            self._smoothed_command = (
                beta * target_vertex[0] + (1.0 - beta) * prev_v,
                beta * target_vertex[1] + (1.0 - beta) * prev_w,
            )

        return self._smoothed_command

    # --------------------------------------------------
    # APPLY
    # --------------------------------------------------

    def apply(self, _u_ref, u_star, constraints, vertices, distance=None, alpha=None):
        self.v_z = self.lyapunov(distance, alpha)

        was_deadlocked = self.is_deadlocked
        if self.is_deadlocked:
            self.detect_deadlock_out(distance)
        else:
            self.detect_deadlock_in(u_star, distance)

        if self.is_deadlocked:
            if not was_deadlocked:
                # V(z) capturado justo al momento de entrar en deadlock,
                # referencia contra la que se mide el progreso del bordeo.
                self.v_z_previous = self.v_z

                # Restricción y dirección se fijan UNA SOLA VEZ, al
                # entrar en deadlock — no en cada tick. Recalcular la
                # "más bloqueante" desde cero cada tick hacía que, justo
                # cuando el punto más cercano cruza el rumbo del robot
                # (bearing≈0°), la dirección se invirtiera de golpe: el
                # robot oscila en el sitio en vez de rodear el obstáculo.
                self.selected_constraint = self.select_constraint(constraints, vertices)
                self.selected_direction  = self.select_direction(self.selected_constraint)
            else:
                # Ya en deadlock: se sigue la EVOLUCIÓN de la misma
                # restricción (el LIDAR cambia tick a tick), nunca se
                # reelige "la más bloqueante" ni se recalcula la dirección.
                self.selected_constraint = self.track_constraint(
                    self.selected_constraint, constraints
                )

            # Vértice de la arista rastreada, recalculado sobre el
            # polígono ACTUAL — al cambiar el LIDAR tick a tick, este
            # vértice se va desplazando (efectivamente "se van
            # seleccionando otros vértices" a medida que el robot avanza
            # y rodea el obstáculo).
            self.selected_vertex = self.select_vertex(
                vertices, self.selected_constraint, self.selected_direction
            )

            u_final = self.smooth_vertex_command(self.selected_vertex)
            if u_final is None:
                # Primer tick del deadlock y la arista no quedó bien
                # definida todavía: respaldo temporal.
                u_final = u_star
        else:
            self.selected_constraint = None
            self.selected_direction  = 0
            self.selected_vertex     = None
            self.other_vertex        = None
            self._smoothed_command   = None
            u_final = u_star

        return u_final, self.is_deadlocked
