import math


class Unicycle:
    """
    Modelo cinemático de robot unicycle (tracción diferencial).

    Integración Euler con paso DT:
        x     += v·cos(θ)·dt
        y     += v·sin(θ)·dt
        theta += w·dt
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x     = float(x)
        self.y     = float(y)
        self.theta = float(theta)

    def step(self, v, w, dt):
        self.x     += v * math.cos(self.theta) * dt
        self.y     += v * math.sin(self.theta) * dt
        self.theta += w * dt
        while self.theta >  math.pi: self.theta -= 2 * math.pi
        while self.theta < -math.pi: self.theta += 2 * math.pi

    def state(self):
        return (self.x, self.y, self.theta)
