import math
from math import atan2, sqrt, cos, sin

k1 = 0.5
k2 = 1.0


class PolarControlLaw:
    """
    Ley de control polar. Dos modos:

    forward_only=False (defecto): v = k1·a·cos(α).  Garantiza ȧ = −k1·a·cos²(α) ≤ 0
        para todo α — el robot siempre se acerca al objetivo, incluso reversando.
        El aliasing en α ≈ ±π es tolerable porque la convergencia sigue asegurada.

    forward_only=True: v clampada a [0, vmax].  El robot gira en el lugar cuando
        |α| > π/2 y avanza solo cuando el objetivo está en el frente. Más lento
        cerca de α = ±π pero nunca reversa.
    """

    def __init__(self, k1=1.0, k2=1.5, vmax=1.0, wmax=1.0, forward_only=False):
        self.k1           = k1
        self.k2           = k2
        self.vmax         = vmax
        self.wmax         = wmax
        self.forward_only = forward_only

    def compute(self, a, alpha):
        v_ref = self.k1 * a * math.cos(alpha)
        w_ref = self.k2 * alpha + self.k1 * math.sin(alpha) * math.cos(alpha)
        v_min = 0.0 if self.forward_only else -self.vmax
        v_ref = max(v_min,      min(self.vmax, v_ref))
        w_ref = max(-self.wmax, min(self.wmax, w_ref))
        return v_ref, w_ref


class NavControlador:
    def __init__(self, x=0.0, y=0.0, theta=0.0, vmax=1.0, wmax=1.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v_ref = 0.0
        self.w_ref = 0.0
        self.vmax = vmax
        self.wmax = wmax
        

    def ley_control(self, xr, yr):
        self.xr = xr    # Coordenadas de meta
        self.yr = yr
        self.a = sqrt((self.xr - self.x)**2 + (self.yr - self.y)**2) # Distancia euclidiana de error de posición
        self.alpha = atan2(self.yr - self.y, self.xr - self.x) - self.theta # Error de orientación con respecto a X
         
        if self.alpha > math.pi: self.alpha -= 2 * math.pi  # Normalización de ángulo "Cambiar a while para lidiar con aliasing grandes"
        if self.alpha < -math.pi: self.alpha += 2 * math.pi
        
        self.v_ref = k1 * self.a * cos(self.alpha)
        self.v_ref = max(min(self.v_ref, self.vmax), -self.vmax)

        self.w_ref = k2 * self.alpha + k1 * sin(self.alpha) * cos(self.alpha)
        self.w_ref = max(min(self.w_ref, self.wmax), -self.wmax)

        return self.v_ref, self.w_ref
  