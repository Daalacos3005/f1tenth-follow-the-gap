"""
Nodo ROS 2: controlador reactivo Follow the Gap + contador de vueltas + cronómetro.

Suscripciones:
    /scan               (sensor_msgs/LaserScan)   -> LIDAR
    /ego_racecar/odom   (nav_msgs/Odometry)       -> posición para contar vueltas
Publicaciones:
    /drive              (ackermann_msgs/AckermannDriveStamped) -> comando al auto
    /lap_info           (visualization_msgs/MarkerArray)       -> texto y objetivo en RViz
"""
import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray

from controllers.ftg_core import FollowTheGap, DEFAULTS
from controllers.lap_timer import LapTimer


class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_node')

        # ---------- Parámetros (se pueden cambiar con --ros-args -p nombre:=valor)
        ftg_params = {}
        for name, value in DEFAULTS.items():
            ftg_params[name] = self.declare_parameter(name, value).value
        self.ftg = FollowTheGap(**ftg_params)

        total_laps = self.declare_parameter('total_laps', 10).value
        self.lap = LapTimer(
            line_half_width=self.declare_parameter('line_half_width', 3.0).value,
            min_lap_distance=self.declare_parameter('min_lap_distance', 30.0).value,
            total_laps=total_laps)
        self.results_dir = self.declare_parameter(
            'results_dir', os.path.join(os.getcwd(), 'resultados')).value

        # ---------- Comunicación
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, sensor_qos)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/lap_info', 10)
        self.create_timer(0.1, self.publish_markers)

        # ---------- Estado
        self.speed_cmd = 0.0
        self.speed_real = 0.0
        self.last_info = None
        self.collisions = 0
        self.stuck_since = None
        self.saved = False

        self.get_logger().info(
            f'Follow the Gap listo. Meta = posición inicial del auto. '
            f'Objetivo: {total_laps} vueltas.')

    # ================================================================ LIDAR
    def lidar_callback(self, scan):
        if self.lap.finished:
            self.publish_drive(0.0, 0.0)
            return
        steer, speed, info = self.ftg.compute(scan.ranges, scan.angle_min, scan.angle_increment)
        self.last_info = info
        self.speed_cmd = speed
        self.publish_drive(steer, speed)

    def publish_drive(self, steer, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ego_racecar/base_link'
        msg.drive.steering_angle = float(steer)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    # ================================================================ ODOMETRÍA
    def odom_callback(self, odom):
        t = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.speed_real = odom.twist.twist.linear.x
        self.now = t

        lap_time = self.lap.update(x, y, yaw, self.speed_real, t)
        if lap_time is not None:
            self.get_logger().info(
                f'VUELTA {self.lap.laps}/{self.lap.total_laps}: {lap_time:7.3f} s   '
                f'(mejor: {self.lap.best_lap:.3f} s)')
            if self.lap.finished:
                self.finish()

        self.check_collision(t)

    def check_collision(self, t):
        """El simulador frena el auto en seco cuando choca. Si pedimos
        velocidad y el auto lleva 0.5 s detenido, se cuenta como choque."""
        if not self.lap.started or self.lap.finished:
            return
        if self.speed_cmd > 1.0 and abs(self.speed_real) < 0.05:
            if self.stuck_since is None:
                self.stuck_since = t
            elif t - self.stuck_since > 0.5:
                self.collisions += 1
                self.get_logger().error(
                    f'COLISIÓN detectada en la vuelta {self.lap.laps + 1} '
                    f'(total colisiones: {self.collisions})')
                self.stuck_since = float('inf')   # no volver a contar el mismo choque
        else:
            self.stuck_since = None

    # ================================================================ RESULTADOS
    def finish(self):
        times = self.lap.lap_times
        lines = ['', '=' * 42, f'  CARRERA TERMINADA: {len(times)} vueltas', '=' * 42]
        for i, lt in enumerate(times, 1):
            mark = '  <- mejor' if lt == self.lap.best_lap else ''
            lines.append(f'  Vuelta {i:2d}: {lt:7.3f} s{mark}')
        lines += ['-' * 42,
                  f'  Tiempo total : {sum(times):8.3f} s',
                  f'  Mejor vuelta : {self.lap.best_lap:8.3f} s',
                  f'  Promedio     : {sum(times) / len(times):8.3f} s',
                  f'  Colisiones   : {self.collisions}',
                  '=' * 42]
        self.get_logger().info('\n'.join(lines))
        self.save_csv()

    def save_csv(self):
        if self.saved:
            return
        os.makedirs(self.results_dir, exist_ok=True)
        name = datetime.now().strftime('vueltas_%Y%m%d_%H%M%S.csv')
        path = os.path.join(self.results_dir, name)
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['vuelta', 'tiempo_s'])
            for i, lt in enumerate(self.lap.lap_times, 1):
                w.writerow([i, f'{lt:.3f}'])
            w.writerow(['mejor', f'{self.lap.best_lap:.3f}'])
            w.writerow(['colisiones', self.collisions])
        self.saved = True
        self.get_logger().info(f'Tiempos guardados en {path}')

    # ================================================================ RVIZ
    def publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Texto con contador y cronómetro, flotando sobre el auto
        text = Marker()
        text.header.frame_id = 'ego_racecar/base_link'
        text.header.stamp = stamp
        text.ns, text.id = 'lap_info', 0
        text.type = Marker.TEXT_VIEW_FACING
        text.pose.position.z = 2.0
        text.pose.orientation.w = 1.0
        text.scale.z = 1.2
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0

        lap_now = min(self.lap.laps + 1, self.lap.total_laps)
        current = self.lap.current_lap_time(getattr(self, 'now', 0.0))
        last = f'{self.lap.lap_times[-1]:.2f} s' if self.lap.lap_times else '--'
        best = f'{self.lap.best_lap:.2f} s' if self.lap.best_lap else '--'
        if self.lap.finished:
            text.text = (f'TERMINADO {self.lap.laps}/{self.lap.total_laps}\n'
                         f'Mejor: {best}  Total: {sum(self.lap.lap_times):.1f} s')
        elif not self.lap.started:
            text.text = 'Esperando arranque...'
        else:
            text.text = (f'Vuelta {lap_now}/{self.lap.total_laps}  |  {current:6.2f} s\n'
                         f'Ultima: {last}  Mejor: {best}\n'
                         f'v = {self.speed_real:4.1f} m/s  Choques: {self.collisions}')
        markers.markers.append(text)

        # Punto objetivo elegido por Follow the Gap
        if self.last_info is not None:
            goal = Marker()
            goal.header.frame_id = 'ego_racecar/base_link'
            goal.header.stamp = stamp
            goal.ns, goal.id = 'ftg_target', 1
            goal.type = Marker.SPHERE
            d = min(self.last_info['target_range'], 4.0)
            a = self.last_info['target_angle']
            goal.pose.position.x = d * math.cos(a)
            goal.pose.position.y = d * math.sin(a)
            goal.pose.orientation.w = 1.0
            goal.scale.x = goal.scale.y = goal.scale.z = 0.3
            goal.color.g = goal.color.a = 1.0
            markers.markers.append(goal)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.lap.lap_times and not node.saved:
            node.save_csv()
        node.publish_drive(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
