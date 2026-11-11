"""
Contador de vueltas y cronómetro (sin dependencias de ROS).

Idea: la línea de meta es la recta que pasa por la posición inicial del
auto y es perpendicular a su orientación inicial. Se calcula
    s = (p - p0) · h        (distancia "hacia adelante" respecto a la meta)
    lat = |(p - p0) x h|    (distancia lateral a lo largo de la línea)
Se cuenta una vuelta cuando s pasa de negativo a positivo, el auto está
cerca de la meta (lat pequeño) y ya recorrió una distancia mínima.
"""
import math


class LapTimer:
    def __init__(self, line_half_width=3.0, min_lap_distance=30.0, total_laps=10):
        self.line_half_width = line_half_width   # [m] ancho útil de la línea de meta
        self.min_lap_distance = min_lap_distance # [m] evita contar dobles
        self.total_laps = total_laps

        self.p0 = None          # punto de la meta
        self.h = None           # dirección de avance en la meta (vector unitario)
        self.prev_s = None
        self.prev_xy = None
        self.dist_since_lap = 0.0

        self.started = False
        self.race_start = None
        self.lap_start = None
        self.lap_times = []

    @property
    def laps(self):
        return len(self.lap_times)

    @property
    def finished(self):
        return self.total_laps > 0 and self.laps >= self.total_laps

    @property
    def best_lap(self):
        return min(self.lap_times) if self.lap_times else None

    def current_lap_time(self, t):
        return 0.0 if self.lap_start is None else t - self.lap_start

    def update(self, x, y, yaw, speed, t):
        """Llamar con cada mensaje de odometría. Devuelve el tiempo de la
        vuelta si se acaba de completar una, si no devuelve None."""
        if self.p0 is None:                       # primera lectura: define la meta
            self.p0 = (x, y)
            self.h = (math.cos(yaw), math.sin(yaw))
            self.prev_s = 0.0
            self.prev_xy = (x, y)
            return None

        if not self.started:                      # el cronómetro arranca al moverse
            if abs(speed) > 0.1:
                self.started = True
                self.race_start = t
                self.lap_start = t
            else:
                return None

        self.dist_since_lap += math.hypot(x - self.prev_xy[0], y - self.prev_xy[1])
        self.prev_xy = (x, y)

        dx, dy = x - self.p0[0], y - self.p0[1]
        s = dx * self.h[0] + dy * self.h[1]
        lat = abs(dx * self.h[1] - dy * self.h[0])

        lap_done = None
        if (self.prev_s < 0.0 <= s and lat < self.line_half_width
                and self.dist_since_lap > self.min_lap_distance
                and not self.finished):
            lap_done = t - self.lap_start
            self.lap_times.append(lap_done)
            self.lap_start = t
            self.dist_since_lap = 0.0
        self.prev_s = s
        return lap_done
