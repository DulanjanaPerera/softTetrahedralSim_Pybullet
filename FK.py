import math


class FK:
    def __init__(self, l: float = 0.21) -> None:
        self.theta = None
        self.phi = None
        self.L = l
        self.x = None
        self.y = None
        self.z = None

    def coordinates(self, theta: float, phi: float) -> tuple[float, float, float]:
        self.theta = theta
        self.phi = phi

        self.x = (self.L / self.phi) * (math.cos(self.theta) * (1 - math.cos(self.phi)))
        self.y = (self.L / self.phi) * (math.sin(self.theta) * (1 - math.cos(self.phi)))
        self.z = self.L * math.sin(self.phi) / self.phi
        return self.x, self.y, self.z

