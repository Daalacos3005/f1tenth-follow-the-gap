#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarAnalysisNode(Node):
    """Analiza el LaserScan: resolución angular, frecuencia y división frontal/trasera."""

    N_MSGS = 50  # mensajes usados para estimar la frecuencia

    def __init__(self):
        super().__init__('lidar_analysis_node')
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.stamps = []
        self.done = False
        self.get_logger().info('Esperando mensajes de /scan ...')

    def scan_callback(self, msg: LaserScan):
        if self.done:
            return

        # Instante de llegada de cada mensaje
        self.stamps.append(time.monotonic())
        if len(self.stamps) < self.N_MSGS:
            return

        # 1) Resolución angular en grados
        resolucion_deg = math.degrees(msg.angle_increment)

        # 2) Frecuencia a partir del intervalo medio entre mensajes
        intervalo_medio = (self.stamps[-1] - self.stamps[0]) / (len(self.stamps) - 1)
        frecuencia = 1.0 / intervalo_medio

        # 3) División de ranges según el ángulo de cada haz
        front_ranges = []  # 0° a 180°
        rear_ranges = []   # resto (180° a 360°)
        for i, distancia in enumerate(msg.ranges):
            angulo = math.degrees(msg.angle_min + i * msg.angle_increment) % 360.0
            if 0.0 <= angulo <= 180.0:
                front_ranges.append(distancia)
            else:
                rear_ranges.append(distancia)

        # 4) Resultados por consola
        print('\n===== ANÁLISIS DEL LIDAR =====')
        print(f'Resolución angular del LiDAR: {resolucion_deg:.3f} grados')
        print(f'Frecuencia del LiDAR: {frecuencia:.3f} Hz')
        print(f'Cantidad de mediciones frontales (0° a 180°): {len(front_ranges)}')
        print(f'Cantidad de mediciones traseras (180° a 360°): {len(rear_ranges)}')
        print('Primeros valores frontales:', [round(r, 2) for r in front_ranges[:5]], '...')
        print('Primeros valores traseros:', [round(r, 2) for r in rear_ranges[:5]], '...')
        print('==============================\n')

        self.done = True  # el nodo termina tras mostrar los resultados


def main(args=None):
    rclpy.init(args=args)
    node = LidarAnalysisNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
