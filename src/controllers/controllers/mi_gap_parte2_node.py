import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

class MiGap(Node):

    def __init__(self):
        super().__init__('mi_gap_parte2')
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Parameters of the algorithm
        self.max_range = 10.0             
        self.car_width = 0.31             
        self.safety_margin = 0.15         
        self.disparity_threshold = 0.4    
        self.bubble_radius = 0.45         
        self.gap_threshold = 1.5          
        self.v_max = 9.5               
        self.v_min = 5                
        self.steer_for_vmin = 0.30   
        
        self.brake_decel = 5.5
        self.front_margin = 2.0
        self.front_cone_deg = 10.0

        self.get_logger().info('My controller started')
        self.center_weight = 0.35
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_cb, 10)
        self.start_pos=  None
        self.heading = None
        self.prev_pos = None
        self.prev_s = 0.0
        self.dist_since_line = 0.0
        self.lap_count = 0
        self.lap_start_time = None
        self.best_lap = None
        self.last_lap = None
        self.trail = []
        self.last_trail_pos = None
        self.lap_pub = self.create_publisher(MarkerArray, '/lap_info', 10)
        self.create_timer(0.2, self.status_cb)


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
        i_far = start + far_idx[len(far_idx) // 2]
        i_center = (start + end) / 2
        w= self.center_weight
        return int(round((1-w) * i_far + w * i_center))



    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        now = self.get_clock().now()
        pos = np.array([x, y])
        if self.start_pos is None:
            yaw = 2.0 * np.arctan2(q.z, q.w)
            self.start_pos = pos
            self.heading = np.array([np.cos(yaw), np.sin(yaw)])
            self.prev_pos = pos
            self.lap_start_time = now
            self.get_logger().info('Start line saved. Stopwatch started.')
            return
        self.dist_since_line += float(np.linalg.norm(pos - self.prev_pos))
        self.prev_pos = pos
        rel = pos - self.start_pos
        s = float(np.dot(rel, self.heading))
        lateral = abs(float(-rel[0] * self.heading[1] + rel[1] * self.heading[0]))
        crossed = self.prev_s < 0.0 and s >= 0.0
        if crossed and lateral < 3.0 and self.dist_since_line > 30.0:
            lap_time = (now - self.lap_start_time).nanoseconds / 1e9
            self.lap_count += 1
            self.last_lap = lap_time
            if self.best_lap is None or lap_time < self.best_lap:
                self.best_lap = lap_time
            self.get_logger().info(
                f'Lap {self.lap_count} completed: {lap_time:.3f} s  '
                f'(best: {self.best_lap:.3f} s)')
            self.lap_start_time = now
            self.dist_since_line = 0.0
            self.trail = []
        self.prev_s = s
        if self.last_trail_pos is None or math.hypot(x - self.last_trail_pos[0], y - self.last_trail_pos[1]) > 0.3:
            self.trail.append(Point(x=x, y=y, z=0.05))
            self.last_trail_pos = (x, y)
    
    def status_cb(self):
        if self.start_pos is None:
            return
        now = self.get_clock().now()
        current = (now - self.lap_start_time).nanoseconds /1e9
        last = f'{self.last_lap: .3f} s' if self.last_lap is not None else '--'
        best = f'{self.best_lap: .3f} s' if self.best_lap is not None else '--'
        marker = Marker()
        marker.header.frame_id = 'ego_racecar/base_link'
        marker.header.stamp = now.to_msg()
        marker.ns = 'lap_info'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 2.5
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 1.2
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.text = (f'Laps: {self.lap_count} | Time: {current: .1f} s |'
                        f'Last: {last} | Best: {best}')
        array = MarkerArray()
        array.markers.append(marker)

        trail = Marker()
        trail.header.frame_id = 'map'
        trail.header.stamp = self.get_clock().now().to_msg()
        trail.ns = 'trail'
        trail.id = 10
        trail.type = Marker.LINE_STRIP
        trail.scale.x = 0.08
        trail.color.r = 0.0
        trail.color.g = 1.0
        trail.color.b = 0.0
        trail.color.a = 1.0
        trail.points = list(self.trail)
        if len(self.trail) < 2:
            trail.action = Marker.DELETE
        else:
            trail.action = Marker.ADD
        array.markers.append(trail)

        self.lap_pub.publish(array)


    def scan_cb(self, scan):
        raw, angles = self.preprocess(scan)
        ranges, corners = self.extend_disparities(raw, scan.angle_increment)
        ranges, i_min = self.apply_bubble(ranges, scan.angle_increment)
        start, end, length = self.find_max_gap(ranges)

        if length == 0:
            steering, speed = 0.0, 0.5
            best = len(angles) // 2
        else:
            best = self.find_best_point(start, end, ranges)
            steering = np.clip(angles[best], -0.4189, 0.4189)
            ratio = min(abs(steering) / self.steer_for_vmin, 1.0)
            speed_a = self.v_max - (self.v_max - self.v_min) * ratio
            front = raw[np.abs(angles) <= np.radians(self.front_cone_deg)]
            free = max(float(front.min()) - self.front_margin, 0.0)
            speed_b = float(np.sqrt(self.v_min ** 2 + 2.0 * self.brake_decel * free))
            speed = min(speed_a, speed_b)

        self.get_logger().info(
            f'Corners: {corners:2d}  '
            f'Closest: {np.degrees(angles[i_min]):6.1f} deg  '
            f'Gap: [{np.degrees(angles[start]):6.1f}, {np.degrees(angles[end]):6.1f}] deg  '
            f'Target: {np.degrees(angles[best]):6.1f} deg  '
            f'Steering: {np.degrees(steering):6.1f} deg  '
            f'Speed: {speed:4.1f} m/s'
            f'Laps: {self.lap_count}'
            f'Best: {(self.best_lap or 0.0):7.3f} s',
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