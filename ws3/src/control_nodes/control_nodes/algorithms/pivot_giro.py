import math


class PivotGiro:
    """
    Estructura virtual en V como cuerpo rígido que gira sobre un pivote.

    A diferencia de TrayectoriaVS (centroide + offsets), aquí no hay
    centroide móvil: se parte de las posiciones absolutas iniciales de
    los n robots (n impar: 1 central + brazos izquierdo/derecho
    simétricos) y en cada paso se aplica una transformación rígida
    T(pivote)·R(dtheta)·T(-pivote) a TODOS los puntos (incluido el
    pivote, que por construcción queda fijo).

    El pivote activo es la punta del brazo que queda físicamente a ese
    lado del centroide SEGÚN SU ORIENTACIÓN ACTUAL (heading = theta_0),
    no según el índice con el que se construyó initial_positions —
    "izquierda" y "derecha" son relativas al marco local del centroide,
    igual que en un vehículo: ángulo total > 0 (giro antihorario, hacia
    la izquierda) → pivote = punta que está a la izquierda del heading
    actual; ángulo total < 0 (horario, hacia la derecha) → pivote =
    punta a la derecha del heading actual. Se determina proyectando
    cada punta contra el vector perpendicular al heading, no por índice
    fijo, de modo que sigue siendo válido tras giros previos que ya
    cambiaron la orientación de la formación.

    Interfaz: expone x, y, theta (del robot central, para reutilizar
    el mismo publisher/plotter que TrayectoriaVS), step(dt), done, y
    positions() -> {índice: (x, y)} con la posición absoluta de cada
    robot, que es lo que el nodo debe usar para mover cada esfera.
    """

    def __init__(self, initial_positions, angle_total, omega, theta_0=0.0):
        # initial_positions: lista de (x, y), longitud n impar,
        #   ordenada como compute_formation_offsets: [0]=centro,
        #   [1..n_left]=brazo izquierdo (del centro hacia afuera),
        #   [n_left+1..]=brazo derecho (del centro hacia afuera).
        # angle_total: ángulo a girar, en radianes (signo = sentido).
        # omega: velocidad angular, rad/s (> 0).
        # theta_0: heading actual del centroide, usado para determinar
        #   qué punta está físicamente a la izquierda/derecha en este
        #   instante (no el índice con el que se construyó la V).
        n = len(initial_positions)
        if n < 3 or n % 2 == 0:
            raise ValueError("PivotGiro requiere un número impar de robots >= 3")
        if omega <= 0.0:
            raise ValueError("omega debe ser > 0")

        self._pos = {i: tuple(p) for i, p in enumerate(initial_positions)}
        n_per_arm = (n - 1) // 2

        arm_a_tip = n_per_arm   # índice de la punta del brazo "A" (construido con +angle_v)
        arm_b_tip = n - 1       # índice de la punta del brazo "B" (construido con -angle_v)

        # Vector perpendicular al heading actual, apuntando a la izquierda
        # del centroide (rotar el heading +90°).
        cx, cy = self._pos[0]
        left_x, left_y = -math.sin(theta_0), math.cos(theta_0)
        ax, ay = self._pos[arm_a_tip]
        # Proyección de la punta A sobre el eje "izquierda": si es positiva,
        # A está a la izquierda del heading actual.
        arm_a_is_left = ((ax - cx) * left_x + (ay - cy) * left_y) > 0.0
        left_tip  = arm_a_tip if arm_a_is_left else arm_b_tip
        right_tip = arm_b_tip if arm_a_is_left else arm_a_tip

        self._left_tip  = left_tip
        self._pivot_idx = left_tip if angle_total > 0.0 else right_tip

        self._angle_remaining = abs(angle_total)
        self._omega           = omega
        self.done              = False

        self.x, self.y = self._pos[0]
        self.theta      = theta_0

    def positions(self):
        return dict(self._pos)

    def step(self, dt):
        if self.done:
            return

        dtheta = self._omega * dt
        if dtheta >= self._angle_remaining:
            dtheta = self._angle_remaining
            self.done = True
        self._angle_remaining -= dtheta

        sign = 1.0 if self._pivot_idx == self._left_tip else -1.0
        c, s = math.cos(sign * dtheta), math.sin(sign * dtheta)
        px, py = self._pos[self._pivot_idx]

        new_pos = {}
        for i, (x, y) in self._pos.items():
            vx, vy = x - px, y - py
            rx, ry = c * vx - s * vy, s * vx + c * vy
            new_pos[i] = (px + rx, py + ry)
        self._pos = new_pos

        self.x, self.y = self._pos[0]
        self.theta += sign * dtheta


class PivotGiroSecuencia:
    """
    Encadena varias fases sobre la misma estructura virtual rígida:
    fases de giro (PivotGiro) y fases de avance en línea recta en la
    dirección del heading acumulado. Cada fase arranca desde las
    posiciones finales de la anterior.

    Interfaz idéntica a PivotGiro: x, y, theta (del robot central),
    step(dt), done, positions(). El nodo no necesita saber cuántas
    fases hay ni de qué tipo son.
    """

    def __init__(self, initial_positions, fases, omega, v_max, theta_0=0.0):
        # fases: lista de tuplas. Cada elemento es
        #   ('giro', angulo_rad)   -> pivota ese ángulo (signo = sentido)
        #   ('avanzar', distancia) -> la formación avanza en línea recta esa
        #        distancia, en la dirección del heading acumulado (theta) del
        #        robot central al momento de iniciar la fase — es decir, "hacia
        #        donde apunta la formación" tras los giros previos.
        # theta_0: heading inicial de la formación (el que define su
        #        orientación de partida, p.ej. el mismo usado para construir
        #        initial_positions), sobre el que se acumulan los giros.
        if not fases:
            raise ValueError("PivotGiroSecuencia requiere al menos una fase")

        self._pos    = {i: tuple(p) for i, p in enumerate(initial_positions)}
        self._fases  = list(fases)
        self._omega  = omega
        self._v_max  = v_max
        self._fase_i = 0
        self.done    = False

        self.x, self.y = self._pos[0]
        self.theta      = theta_0

        self._iniciar_fase_actual()

    def _iniciar_fase_actual(self):
        tipo, valor = self._fases[self._fase_i]
        if tipo == 'giro':
            self._sub = PivotGiro(
                initial_positions = [self._pos[i] for i in range(len(self._pos))],
                angle_total       = valor,
                omega             = self._omega,
                theta_0           = self.theta,
            )
        elif tipo == 'avanzar':
            self._dir            = (math.cos(self.theta), math.sin(self.theta))
            self._dist_restante  = valor
        else:
            raise ValueError(f"Tipo de fase desconocido: {tipo}")

    def positions(self):
        return dict(self._pos)

    def step(self, dt):
        if self.done:
            return

        tipo, _ = self._fases[self._fase_i]

        if tipo == 'giro':
            self._sub.step(dt)
            self._pos    = self._sub.positions()
            self.x, self.y = self._sub.x, self._sub.y
            self.theta    = self._sub.theta
            fase_done     = self._sub.done

        else:  # avanzar
            paso = self._v_max * dt
            if paso >= self._dist_restante:
                paso = self._dist_restante
                fase_done = True
            else:
                fase_done = False
            self._dist_restante -= paso

            ux, uy = self._dir
            new_pos = {}
            for i, (x, y) in self._pos.items():
                new_pos[i] = (x + ux * paso, y + uy * paso)
            self._pos = new_pos
            self.x, self.y = self._pos[0]

        if fase_done:
            self._fase_i += 1
            if self._fase_i >= len(self._fases):
                self.done = True
            else:
                self._iniciar_fase_actual()


class PivotGiroWaypoints:
    """
    Alcanza una lista de waypoints, uno por uno, con la formación en V
    como cuerpo rígido. Por cada waypoint:
      1. Gira sobre el pivote correspondiente hasta que el heading del
         centroide apunte hacia el waypoint. El giro es un control
         proporcional (P) sobre el ERROR angular: en cada step() se
         recalcula el error entre el heading actual y la dirección al
         waypoint (medida desde el pivote, que permanece fijo durante
         todo el giro) y se avanza dtheta = clip(kp*error, -omega, omega)*dt,
         reduciendo el error de forma continua en vez de precalcular un
         ángulo total fijo.
      2. Avanza en línea recta hasta alcanzarlo.

    La distancia a avanzar se calcula justo AL TERMINAR el giro (no
    antes): pivotar mueve el centroide (no es un giro sobre sí mismo),
    así que una distancia calculada antes del giro ya no sería correcta
    al momento de avanzar. Si el waypoint cae exactamente sobre el
    punto donde el centroide queda tras el giro solo (la "zona en
    forma de maní" delimitada por los radios de giro de ambos pivotes,
    medidos al centroide), la fase de avance se omite.
    """

    TOL      = 1e-3   # metros; por debajo de esto se considera "ya alcanzado"
    ANG_TOL  = 1e-3   # rad; error angular por debajo del cual se da el giro por terminado
    KP       = 2.0    # ganancia proporcional del control de giro

    def __init__(self, initial_positions, waypoints, omega, v_max, theta_0=0.0):
        if not waypoints:
            raise ValueError("PivotGiroWaypoints requiere al menos un waypoint")

        self._pos       = {i: tuple(p) for i, p in enumerate(initial_positions)}
        self._waypoints = list(waypoints)
        self._omega     = omega
        self._v_max     = v_max
        self._wp_i      = 0
        self.done        = False

        self.x, self.y = self._pos[0]
        self.theta      = theta_0

        self._pivot_idx = None   # índice del pivote activo durante el giro
        self._left_tip  = None   # cuál de los dos tips es "izquierda" en este giro
        self._dir       = None   # dirección de avance, fijada al terminar el giro
        self._dist_restante = 0.0
        self._etapa  = None   # 'giro' | 'avanzar' | None (waypoint resuelto)

        self._iniciar_waypoint_actual()

    def _angulo_hacia_waypoint(self):
        wx, wy = self._waypoints[self._wp_i]
        dx, dy = wx - self.x, wy - self.y
        target_theta = math.atan2(dy, dx)
        # Error angular más corto, normalizado a (-pi, pi].
        return (target_theta - self.theta + math.pi) % (2 * math.pi) - math.pi

    def _iniciar_waypoint_actual(self):
        wx, wy = self._waypoints[self._wp_i]
        dist = math.hypot(wx - self.x, wy - self.y)

        if dist < self.TOL:
            self._etapa = None
            return

        error = self._angulo_hacia_waypoint()
        if abs(error) > self.ANG_TOL:
            self._iniciar_giro(error)
        else:
            self._iniciar_avance()

    def _iniciar_giro(self, error_inicial):
        # Determina y fija el pivote (izquierda/derecha) según el sentido
        # del giro más corto hacia el waypoint en este instante. El
        # pivote permanece fijo durante todo el giro; el error se
        # recalcula en cada step() usando la posición actual del
        # centroide, pero el waypoint y el pivote no se mueven.
        n = len(self._pos)
        n_per_arm = (n - 1) // 2
        arm_a_tip = n_per_arm
        arm_b_tip = n - 1

        cx, cy = self._pos[0]
        left_x, left_y = -math.sin(self.theta), math.cos(self.theta)
        ax, ay = self._pos[arm_a_tip]
        arm_a_is_left = ((ax - cx) * left_x + (ay - cy) * left_y) > 0.0
        left_tip  = arm_a_tip if arm_a_is_left else arm_b_tip
        right_tip = arm_b_tip if arm_a_is_left else arm_a_tip

        self._left_tip  = left_tip
        self._pivot_idx = left_tip if error_inicial > 0.0 else right_tip
        self._etapa      = 'giro'

    def _iniciar_avance(self):
        wx, wy = self._waypoints[self._wp_i]
        dx, dy = wx - self.x, wy - self.y
        dist   = math.hypot(dx, dy)

        if dist < self.TOL:
            self._etapa = None
            return

        self._dir           = (dx / dist, dy / dist)
        self._dist_restante = dist
        self._etapa          = 'avanzar'

    def positions(self):
        return dict(self._pos)

    def step(self, dt):
        if self.done:
            return

        if self._etapa == 'giro':
            error = self._angulo_hacia_waypoint()
            if abs(error) <= self.ANG_TOL:
                self._iniciar_avance()
            else:
                omega_cmd = max(-self._omega, min(self._omega, self.KP * error))
                dtheta    = omega_cmd * dt
                if abs(dtheta) > abs(error):
                    dtheta = error

                # dtheta ya trae el signo correcto: mismo signo que el error,
                # que es el sentido de giro hacia el waypoint.
                c, s = math.cos(dtheta), math.sin(dtheta)
                px, py = self._pos[self._pivot_idx]

                new_pos = {}
                for i, (x, y) in self._pos.items():
                    vx, vy = x - px, y - py
                    rx, ry = c * vx - s * vy, s * vx + c * vy
                    new_pos[i] = (px + rx, py + ry)
                self._pos = new_pos

                self.x, self.y = self._pos[0]
                self.theta += dtheta

                if abs(self._angulo_hacia_waypoint()) <= self.ANG_TOL:
                    self._iniciar_avance()

        elif self._etapa == 'avanzar':
            paso = self._v_max * dt
            if paso >= self._dist_restante:
                paso = self._dist_restante
                self._etapa = None
            self._dist_restante -= paso

            ux, uy = self._dir
            new_pos = {}
            for i, (x, y) in self._pos.items():
                new_pos[i] = (x + ux * paso, y + uy * paso)
            self._pos = new_pos
            self.x, self.y = self._pos[0]

        if self._etapa is None:
            self._wp_i += 1
            if self._wp_i >= len(self._waypoints):
                self.done = True
            else:
                self._iniciar_waypoint_actual()


class PivotGiroWaypointsV2:
    """
    Igual objetivo que PivotGiroWaypoints (alcanzar una lista de
    waypoints, uno por uno), pero girar y avanzar ocurren A LA PAR en
    vez de en dos etapas secuenciales: en cada step(dt) se aplica,
    superpuestos, un giro relativo sobre pivote (control proporcional
    del error angular, igual que antes) Y una traslación en línea recta
    hacia el waypoint a v_max constante.

    El pivote ya NO es un punto fijo del mundo durante el giro: es el
    centro relativo de la rotación en ese instante, pero se traslada
    junto con el resto de la formación (porque la traslación se aplica
    a TODOS los puntos por igual, incluido el pivote). El pivote
    (izquierda/derecha) se recalcula en cada step según el signo del
    error angular actual, igual criterio que en PivotGiro/PivotGiroWaypoints
    (proyección de las puntas contra el heading actual), de modo que si
    el error cambia de signo a mitad de camino el pivote conmuta.

    Un waypoint se da por alcanzado cuando AMBOS errores (angular y de
    distancia) caen bajo tolerancia en el mismo step.
    """

    TOL     = 1e-3   # metros
    ANG_TOL = 1e-3   # rad
    KP      = 2.0    # ganancia proporcional del control de giro

    def __init__(self, initial_positions, waypoints, omega, v_max, theta_0=0.0):
        if not waypoints:
            raise ValueError("PivotGiroWaypointsV2 requiere al menos un waypoint")

        self._pos       = {i: tuple(p) for i, p in enumerate(initial_positions)}
        self._waypoints = list(waypoints)
        self._omega     = omega
        self._v_max     = v_max
        self._wp_i      = 0
        self.done        = False

        self.x, self.y = self._pos[0]
        self.theta      = theta_0

        self._resolver_wp_actual()

    def _resolver_wp_actual(self):
        wx, wy = self._waypoints[self._wp_i]
        if math.hypot(wx - self.x, wy - self.y) < self.TOL:
            self._avanzar_wp()

    def _avanzar_wp(self):
        self._wp_i += 1
        if self._wp_i >= len(self._waypoints):
            self.done = True
        else:
            self._resolver_wp_actual()

    def _pivote_activo(self, error_angular):
        n = len(self._pos)
        n_per_arm = (n - 1) // 2
        arm_a_tip = n_per_arm
        arm_b_tip = n - 1

        cx, cy = self._pos[0]
        left_x, left_y = -math.sin(self.theta), math.cos(self.theta)
        ax, ay = self._pos[arm_a_tip]
        arm_a_is_left = ((ax - cx) * left_x + (ay - cy) * left_y) > 0.0
        left_tip  = arm_a_tip if arm_a_is_left else arm_b_tip
        right_tip = arm_b_tip if arm_a_is_left else arm_a_tip

        return (left_tip if error_angular > 0.0 else right_tip)

    def positions(self):
        return dict(self._pos)

    def step(self, dt):
        if self.done:
            return

        wx, wy = self._waypoints[self._wp_i]
        dx, dy = wx - self.x, wy - self.y
        dist   = math.hypot(dx, dy)

        target_theta  = math.atan2(dy, dx) if dist > self.TOL else self.theta
        error_angular = (target_theta - self.theta + math.pi) % (2 * math.pi) - math.pi

        ang_done  = abs(error_angular) <= self.ANG_TOL
        dist_done = dist <= self.TOL

        if ang_done and dist_done:
            self._avanzar_wp()
            return

        # --- 1. Giro relativo sobre pivote (control proporcional) ---
        if not ang_done:
            pivot_idx = self._pivote_activo(error_angular)
            omega_cmd = max(-self._omega, min(self._omega, self.KP * error_angular))
            dtheta    = omega_cmd * dt
            if abs(dtheta) > abs(error_angular):
                dtheta = error_angular

            c, s = math.cos(dtheta), math.sin(dtheta)
            px, py = self._pos[pivot_idx]

            rotated = {}
            for i, (x, y) in self._pos.items():
                vx, vy = x - px, y - py
                rx, ry = c * vx - s * vy, s * vx + c * vy
                rotated[i] = (px + rx, py + ry)
            self._pos = rotated
            self.theta += dtheta

        # --- 2. Traslación en línea recta hacia el waypoint, superpuesta ---
        if not dist_done:
            paso = min(self._v_max * dt, dist)
            ux, uy = (dx / dist, dy / dist) if dist > 1e-9 else (0.0, 0.0)

            translated = {}
            for i, (x, y) in self._pos.items():
                translated[i] = (x + ux * paso, y + uy * paso)
            self._pos = translated

        self.x, self.y = self._pos[0]
