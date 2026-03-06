import math
from math import atan2, sqrt, cos, sin

k1 = 0.5
k2 = 1.0

class ControlLaw:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.w = 0.0

    def ley_control(self, xr, yr):
        # Punto objetivo
        self.xr = xr
        self.yr = yr
        
        # Distancia
        self.a = sqrt((self.xr - self.x)**2 + (self.yr - self.y)**2)

        # Ángulo al objetivo
        self.alpha = atan2(self.yr - self.y, self.xr - self.x) - self.theta
        
        # Normalización de ángulo 
        "Cambiar a while para lidiar con aliasing grandes"
        if self.alpha > math.pi:
            self.alpha -= 2 * math.pi
        if self.alpha < -math.pi:
            self.alpha += 2 * math.pi
        
        # Ley de control verificar unidades
        self.v = k1 * self.a * cos(self.alpha)
        self.v = max(min(self.v, 1.0), -1.0)

        self.w = k2 * self.alpha + k1 * sin(self.alpha) * cos(self.alpha)
        self.w = max(min(self.w, 1.0), -1.0)
        
        return self.v, self.w
    
    def actualizar_estado(self, dt):
        # Integración simple
        self.x += self.v * cos(self.theta) * dt
        self.y += self.v * sin(self.theta) * dt
        self.theta += self.w * dt
        
        # Normalización del ángulo
        if self.theta > math.pi:
            self.theta -= 2 * math.pi
        if self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        return self.x, self.y, self.theta
