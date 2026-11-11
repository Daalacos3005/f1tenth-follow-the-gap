"""
Algoritmo Follow the Gap (sin dependencias de ROS).

Se deja separado del nodo para poder probarlo y explicarlo por partes.
Cada paso del algoritmo es un método:
    1. preprocess()       -> limpia y suaviza el LIDAR
    2. extend_disparities -> "engorda" los bordes de las paredes
    3. apply_bubble()     -> burbuja de seguridad alrededor del punto más cercano
    4. find_max_gap()     -> gap (hueco) más grande
    5. find_best_point()  -> punto objetivo dentro del gap
    6. steering/speed     -> comando de dirección y velocidad
"""
import numpy as np


DEFAULTS = {
    # --- LIDAR ---
    'fov_deg': 90.0,            # se usa solo de -fov a +fov (medio ángulo)
    'max_range': 10.0,          # lecturas más largas se recortan a este valor [m]
    'smooth_window': 5,         # ventana de promedio móvil [muestras]
    # --- Geometría y seguridad ---
    'car_width': 0.31,          # ancho del auto [m]
    'safety_margin': 0.25,      # margen extra a cada lado [m]
    'disparity_threshold': 0.4, # salto de distancia que se considera un borde [m]
    'bubble_radius': 0.45,      # radio de la burbuja de seguridad [m]
    'gap_min_range': 1.5,       # distancia mínima para considerar un rayo "libre" [m]
    'center_weight': 0.35,      # 0 = ir al punto más lejano, 1 = ir al centro del gap
    # --- Dirección ---
    'steer_gain': 1.0,          # ganancia proporcional sobre el ángulo del objetivo
    'max_steer': 0.4189,        # límite físico del auto [rad] (24°)
    'steer_alpha': 1.0,         # filtro pasa bajos (1.0 = sin filtro)
    # --- Velocidad ---
    'v_max': 12.0,              # velocidad en rectas [m/s]
    'v_min': 4.5,               # velocidad mínima en curvas cerradas [m/s]
    'steer_for_vmin': 0.30,     # con este ángulo de dirección ya se usa v_min [rad]
    'brake_decel': 4.5,         # desaceleración que se asume para frenar [m/s^2]
    'front_margin': 2.0,        # distancia que se deja libre al frente [m]
    'front_cone_deg': 8.0,      # cono frontal para medir distancia libre [°]
}


class FollowTheGap:
    def __init__(self, **params):
        self.p = dict(DEFAULTS)
        self.p.update(params)
        self.prev_steer = 0.0

    # ------------------------------------------------------------------ 1
    def preprocess(self, ranges, angle_min, angle_inc):
        """Recorta el campo de visión, quita NaN/inf, limita y suaviza."""
        r = np.array(ranges, dtype=float)
        r[~np.isfinite(r)] = self.p['max_range']
        r = np.clip(r, 0.0, self.p['max_range'])

        angles = angle_min + np.arange(len(r)) * angle_inc
        fov = np.radians(self.p['fov_deg'])
        mask = np.abs(angles) <= fov
        r, angles = r[mask], angles[mask]

        w = int(self.p['smooth_window'])
        if w > 1:
            kernel = np.ones(w) / w
            r = np.convolve(np.pad(r, w // 2, mode='edge'), kernel, mode='valid')[:len(angles)]
        return r, angles

    # ------------------------------------------------------------------ 2
    def extend_disparities(self, r, angle_inc):
        """Donde hay un salto brusco de distancia (borde de una pared),
        copia la distancia corta hacia el lado largo un ancho de medio auto.
        Así el objetivo nunca queda pegado a una esquina."""
        out = r.copy()
        half = self.p['car_width'] / 2 + self.p['safety_margin']
        for i in range(len(r) - 1):
            a, b = r[i], r[i + 1]
            if abs(a - b) < self.p['disparity_threshold']:
                continue
            near = min(a, b)
            n = int(np.ceil(np.arctan2(half, max(near, 0.05)) / angle_inc))
            if a < b:   # el borde cercano está a la izquierda del índice -> extender hacia i+1..
                end = min(len(r), i + 1 + n)
                out[i + 1:end] = np.minimum(out[i + 1:end], near)
            else:       # el borde cercano está en i+1 -> extender hacia atrás
                start = max(0, i + 1 - n)
                out[start:i + 1] = np.minimum(out[start:i + 1], near)
        return out

    # ------------------------------------------------------------------ 3
    def apply_bubble(self, r, angle_inc):
        """Pone en cero los rayos alrededor del punto más cercano."""
        out = r.copy()
        i_min = int(np.argmin(r))
        d_min = max(r[i_min], 0.05)
        n = int(np.ceil(np.arctan2(self.p['bubble_radius'], d_min) / angle_inc))
        out[max(0, i_min - n):min(len(r), i_min + n + 1)] = 0.0
        return out, i_min

    # ------------------------------------------------------------------ 4
    @staticmethod
    def _longest_run(mask):
        """Devuelve (inicio, fin) del tramo más largo de True consecutivos."""
        best, start, best_s, best_e = 0, None, 0, -1
        for i, v in enumerate(np.append(mask, False)):
            if v and start is None:
                start = i
            elif not v and start is not None:
                if i - start > best:
                    best, best_s, best_e = i - start, start, i - 1
                start = None
        return best_s, best_e

    def find_max_gap(self, r):
        """Gap = secuencia más larga de rayos con distancia > gap_min_range.
        Si no hay ninguno (curva muy cerrada), baja el umbral."""
        thr = self.p['gap_min_range']
        while thr > 0.1:
            s, e = self._longest_run(r > thr)
            if e >= s:
                return s, e
            thr *= 0.5
        return 0, len(r) - 1

    # ------------------------------------------------------------------ 5
    def find_best_point(self, s, e, r):
        """En vez de ir al punto más lejano exacto (que salta mucho),
        toma todos los puntos casi tan lejanos como el máximo, saca el
        centro de ese grupo y lo mezcla con el centro del gap
        (center_weight) para no ir pegado al lado interior de las curvas."""
        seg = r[s:e + 1]
        far = seg >= seg.max() - 0.5
        a, b = self._longest_run(far)
        i_far = s + (a + b) / 2
        i_center = (s + e) / 2
        w = self.p['center_weight']
        return int(round((1 - w) * i_far + w * i_center))

    # ------------------------------------------------------------------ 6
    def compute(self, ranges, angle_min, angle_inc):
        """Ejecuta todos los pasos y devuelve (steer, speed, info)."""
        r, angles = self.preprocess(ranges, angle_min, angle_inc)
        r_ext = self.extend_disparities(r, angle_inc)
        r_bub, i_min = self.apply_bubble(r_ext, angle_inc)
        s, e = self.find_max_gap(r_bub)
        best = self.find_best_point(s, e, r_bub)

        target_angle = angles[best]
        steer = np.clip(self.p['steer_gain'] * target_angle,
                        -self.p['max_steer'], self.p['max_steer'])
        a = self.p['steer_alpha']
        steer = a * steer + (1 - a) * self.prev_steer
        self.prev_steer = steer

        # Velocidad: baja con el ángulo de dirección y con la distancia al frente
        k = np.clip(abs(steer) / self.p['steer_for_vmin'], 0.0, 1.0)
        v = self.p['v_max'] - k * (self.p['v_max'] - self.p['v_min'])
        cone = np.abs(angles) <= np.radians(self.p['front_cone_deg'])
        d_front = float(np.min(r[cone])) if np.any(cone) else float(np.min(r))
        # Límite por frenado: con la distancia libre al frente el auto debe
        # poder bajar hasta v_min:  v^2 = v_min^2 + 2*a*(d - margen)
        free = max(0.0, d_front - self.p['front_margin'])
        v_brake = np.sqrt(self.p['v_min'] ** 2 + 2 * self.p['brake_decel'] * free)
        v = min(v, v_brake)

        info = {
            'target_angle': float(target_angle),
            'target_range': float(r_bub[best]),
            'closest_angle': float(angles[i_min]),
            'closest_range': float(r[i_min]),
            'gap': (float(angles[s]), float(angles[e])),
            'd_front': d_front,
        }
        return float(steer), float(v), info
