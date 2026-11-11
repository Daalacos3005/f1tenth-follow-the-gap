import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class MiGap(Node):

    def __init__(self):
        super().__init__('mi_gap')
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Parameters of the algorithm
        self.max_range = 10.0             # [m] distances are capped at this value
        self.car_width = 0.31             # [m] width of the F1Tenth car
        self.safety_margin = 0.15         # [m] extra space on each side of the car
        self.disparity_threshold = 0.4    # [m] jump between two rays that means "corner"
        self.bubble_radius = 0.45         # [m] safety bubble around the closest point
        self.gap_threshold = 1.5          # [m] a ray is "free" if it is farther than this
        self.speed = 2.0                  # [m/s] constant speed for now

        self.get_logger().info('My controller started')


    def preprocess(self, scan):
        ranges = np.array(scan.ranges)
        ranges[~np.isfinite(ranges)] = self.max_range
        ranges = np.clip(ranges, 0.0, self.max_range)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        mask = np.abs(angles) <= np.radians(90)
        return ranges[mask], angles[mask]


    def extend_disparities(self, ranges, angle_inc):
        extended = ranges.copy()
        half_car = self.car_width / 2 + self.safety_margin
        corners = 0

        for i in range(len(ranges) - 1):
            a = ranges[i]
            b = ranges[i + 1]


            if abs(a - b) < self.disparity_threshold:
                continue

            corners += 1
            near = min(a, b)


            n = int(np.ceil(np.arctan(half_car / max(near, 0.05)) / angle_inc))

            if a < b:

                end = min(len(ranges), i + 1 + n)
                extended[i + 1:end] = np.minimum(extended[i + 1:end], near)
            else:

                start = max(0, i + 1 - n)
                extended[start:i + 1] = np.minimum(extended[start:i + 1], near)

        return extended, corners

    def apply_bubble(self, ranges, angle_inc):
        ranges = ranges.copy()
        i_min = np.argmin(ranges)
        d_min = max(ranges[i_min], 0.05)
        n = int(np.arctan(self.bubble_radius / d_min) / angle_inc)
        start = max(0, i_min - n)
        end = min(len(ranges), i_min + n + 1)
        ranges[start:end] = 0.0
        return ranges, i_min

  
    def find_max_gap(self, ranges):
        best_len, best_start, best_end = 0, 0, 0
        start = None

        for i in range(len(ranges)):
            if ranges[i] > self.gap_threshold:
                if start is None:
                    start = i
            else:
                if start is not None:
                    length = i - start
                    if length > best_len:
                        best_len, best_start, best_end = length, start, i - 1
                    start = None

        if start is not None and len(ranges) - start > best_len:
            best_len, best_start, best_end = len(ranges) - start, start, len(ranges) - 1

        return best_start, best_end, best_len

    def find_best_point(self, start, end, ranges):
        segment = ranges[start:end + 1]
        far_idx = np.where(segment >= segment.max() - 0.1)[0]
        return start + far_idx[len(far_idx) // 2]


    def scan_cb(self, scan):
        ranges, angles = self.preprocess(scan)
        ranges, corners = self.extend_disparities(ranges, scan.angle_increment)
        ranges, i_min = self.apply_bubble(ranges, scan.angle_increment)
        start, end, length = self.find_max_gap(ranges)

        if length == 0:
            steering, speed = 0.0, 0.5
            best = len(angles) // 2
        else:
            best = self.find_best_point(start, end, ranges)
            steering = np.clip(angles[best], -0.4189, 0.4189)
            speed = self.speed

        self.get_logger().info(
            f'Corners: {corners:2d}  '
            f'Closest: {np.degrees(angles[i_min]):6.1f} deg  '
            f'Gap: [{np.degrees(angles[start]):6.1f}, {np.degrees(angles[end]):6.1f}] deg  '
            f'Target: {np.degrees(angles[best]):6.1f} deg  '
            f'Steering: {np.degrees(steering):6.1f} deg',
            throttle_duration_sec=0.5)

        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MiGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            stop = AckermannDriveStamped()
            stop.drive.speed = 0.0
            node.pub.publish(stop)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()