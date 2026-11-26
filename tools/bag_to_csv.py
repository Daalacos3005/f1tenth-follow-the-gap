import sys, os, csv, math
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]
out = bag + '_csv'
scan_every = 10
os.makedirs(out, exist_ok=True)

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}

odom_files = {'/ego_racecar/odom': 'odom.csv', '/opp_racecar/odom': 'opp_odom.csv'}
drive_files = {'/drive': 'drive.csv', '/opp_drive': 'opp_drive.csv'}
scan_files = {'/scan': 'scan.csv', '/opp_scan': 'opp_scan.csv'}

files = {}
def writer(name, header):
    if name not in files:
        f = open(os.path.join(out, name), 'w', newline='')
        w = csv.writer(f)
        w.writerow(header)
        files[name] = (f, w)
    return files[name][1]

t0 = None
scan_count = {}

while reader.has_next():
    topic, data, t = reader.read_next()
    if t0 is None:
        t0 = t
    ts = (t - t0) / 1e9
    msg = deserialize_message(data, get_message(types[topic]))

    if topic in odom_files:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        v = msg.twist.twist
        w = writer(odom_files[topic], ['time_s', 'x', 'y', 'yaw', 'linear_vel_x', 'linear_vel_y', 'angular_vel_z'])
        w.writerow([f'{ts:.4f}', p.x, p.y, yaw, v.linear.x, v.linear.y, v.angular.z])

    elif topic in drive_files:
        w = writer(drive_files[topic], ['time_s', 'speed', 'steering_angle'])
        w.writerow([f'{ts:.4f}', msg.drive.speed, msg.drive.steering_angle])

    elif topic in scan_files:
        n = scan_count.get(topic, 0)
        if n % scan_every == 0:
            w = writer(scan_files[topic], ['time_s', 'angle_min', 'angle_increment'] +
                       [f'r{i}' for i in range(len(msg.ranges))])
            w.writerow([f'{ts:.4f}', msg.angle_min, msg.angle_increment] +
                       [f'{r:.3f}' for r in msg.ranges])
        scan_count[topic] = n + 1

for f, w in files.values():
    f.close()
print('CSV files saved in', out, ':', ', '.join(sorted(files)))