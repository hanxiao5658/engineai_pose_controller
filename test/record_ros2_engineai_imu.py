#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 Humble: record /hardware/imu_info ImuInfo with spike diagnostics.
Run with python3 directly inside sourced ROS2 environment. Ctrl+C flushes files.
"""
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from interface_protocol.msg import ImuInfo

DEFAULT_TOPIC = '/hardware/imu_info'
GAP_MS = 15.0


def default_csv_path():
    return '~/imu_logs/ros2_hardware_imu_{}.csv'.format(
        time.strftime('%Y%m%d_%H%M%S'))


# ===== Common standalone analysis (identical in both scripts) =====
import csv
import math
import os
import time
from datetime import datetime, timezone
from collections import Counter

G0 = 9.80665
FIELDS = [
    'utc_time', 'recv_elapsed_s', 'stamp_sec', 'stamp_nsec', 'stamp_time_s',
    'dt_header_s', 'dt_recv_s', 'dt_used_s', 'dt_source',
    'ax', 'ay', 'az', 'wx', 'wy', 'wz',
    'qx', 'qy', 'qz', 'qw', 'roll', 'pitch', 'yaw',
    'accel_norm_mps2', 'accel_norm_g', 'gyro_norm_rads', 'gyro_norm_dps',
    'accel_delta_mps2', 'jerk_mps3', 'angular_accel_rads2', 'flags',
]


def norm3(vec):
    return math.sqrt(sum(float(v) * float(v) for v in vec))


def rpy_from_quat(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-9:
        return (float('nan'),) * 3
    x, y, z, w = (v/n for v in (x, y, z, w))
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1., min(1., 2*(w*y - z*x))))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


class ImuCsvAnalyzer:
    def __init__(self, args, title, log):
        self.args = args
        self.title = title
        self.log = log
        self.start_monotonic = time.monotonic()
        self.last_report = self.start_monotonic
        self.prev_mono = None
        self.prev_stamp_ns = None
        self.prev_a = None
        self.prev_w = None
        self.n = 0
        self.flags_count = Counter()
        self.maxima = {'accel_norm_g': 0., 'gyro_norm_dps': 0.,
                       'jerk_mps3': 0., 'angular_accel_rads2': 0.,
                       'dt_used_s': 0.}
        self.window_n = 0
        self.window_max_accel_g = 0.
        self.window_max_gyro_dps = 0.
        self.window_max_jerk = 0.
        self.window_max_dt_ms = 0.
        self.window_flags = Counter()
        self.closed = False
        self.file_path = os.path.abspath(os.path.expanduser(args.output))
        os.makedirs(os.path.dirname(self.file_path), exist_ok=True)
        root, ext = os.path.splitext(self.file_path)
        self.events_path = root + '_events.csv'
        self.summary_path = root + '_summary.txt'
        self.fp = open(self.file_path, 'w', newline='', buffering=1)
        self.evfp = open(self.events_path, 'w', newline='', buffering=1)
        self.writer = csv.DictWriter(self.fp, fieldnames=FIELDS)
        self.writer.writeheader()
        self.evwriter = csv.writer(self.evfp)
        self.evwriter.writerow(['utc_time', 'recv_elapsed_s', 'flags',
                                'accel_norm_g', 'gyro_norm_dps',
                                'jerk_mps3', 'dt_used_s', 'stamp_time_s'])
        self.log('Recording {} -> {}'.format(title, self.file_path))
        self.log('Event log -> {}'.format(self.events_path))
        if args.accel_range_g <= 0 or args.gyro_range_dps <= 0:
            self.log('Saturation check disabled for unspecified axes/ranges; '
                     'provide sensor full-scale limits to enable it.')

    def add(self, stamp_ns, acc, gyro, quat, rpy):
        if self.closed:
            return
        now = time.monotonic()
        wall_iso = datetime.now(timezone.utc).isoformat(timespec='milliseconds')
        acc = tuple(float(x) for x in acc)
        gyro = tuple(float(x) for x in gyro)
        quat = tuple(float(x) for x in quat)
        rpy = tuple(float(x) for x in rpy)
        flags = []
        if not all(math.isfinite(v) for v in acc+gyro):
            flags.append('NONFINITE')
        recv_dt = (now-self.prev_mono) if self.prev_mono is not None else float('nan')
        header_dt = float('nan')
        if stamp_ns > 0 and self.prev_stamp_ns is not None and self.prev_stamp_ns > 0:
            header_dt = (stamp_ns-self.prev_stamp_ns)/1e9
            if header_dt < 0:
                flags.append('STAMP_RESET')
            elif header_dt == 0:
                flags.append('STAMP_STALL')
        # Use sensor/header time for derivatives if advancing sensibly.
        # This is important when Gazebo sim rate differs from wall rate.
        if math.isfinite(header_dt) and 0 < header_dt <= 1.0:
            dt, dt_src = header_dt, 'header'
        elif math.isfinite(recv_dt) and recv_dt > 0:
            dt, dt_src = recv_dt, 'receive'
        else:
            dt, dt_src = float('nan'), 'none'
        a_norm = norm3(acc)
        w_norm = norm3(gyro)
        accel_g = a_norm/G0
        gyro_dps = math.degrees(w_norm)
        if self.prev_a is not None and math.isfinite(dt) and dt > 1e-7:
            da = norm3(tuple(x-y for x, y in zip(acc, self.prev_a)))
            jerk = da/dt
            angular_accel = norm3(tuple(x-y for x,y in zip(gyro, self.prev_w)))/dt
        else:
            da = jerk = angular_accel = float('nan')
        if not math.isfinite(a_norm) or not math.isfinite(w_norm):
            flags.append('INVALID_IMU')
        else:
            if accel_g >= self.args.accel_spike_g:
                flags.append('ACC_PEAK')
            if gyro_dps >= self.args.gyro_spike_dps:
                flags.append('GYRO_PEAK')
            if math.isfinite(jerk) and jerk >= self.args.jerk_threshold:
                flags.append('JERK_PEAK')
            if self.args.accel_range_g > 0 and any(
                abs(v) >= self.args.saturation_fraction*self.args.accel_range_g*G0
                for v in acc
            ):
                flags.append('ACC_NEAR_RANGE')
            if self.args.gyro_range_dps > 0 and any(
                abs(math.degrees(v)) >= self.args.saturation_fraction*self.args.gyro_range_dps
                for v in gyro
            ):
                flags.append('GYRO_NEAR_RANGE')
        if math.isfinite(header_dt) and header_dt >= self.args.gap_ms/1000:
            flags.append('HEADER_GAP')
        # CSV has every message even if a threshold is not exceeded.
        sec = stamp_ns//1000000000 if stamp_ns > 0 else 0
        nsec = stamp_ns%1000000000 if stamp_ns > 0 else 0
        values = dict(
            utc_time=wall_iso, recv_elapsed_s=now-self.start_monotonic,
            stamp_sec=sec, stamp_nsec=nsec,
            stamp_time_s=stamp_ns/1e9 if stamp_ns > 0 else '',
            dt_header_s=header_dt, dt_recv_s=recv_dt,
            dt_used_s=dt, dt_source=dt_src,
            ax=acc[0], ay=acc[1], az=acc[2],
            wx=gyro[0], wy=gyro[1], wz=gyro[2],
            qx=quat[0], qy=quat[1], qz=quat[2], qw=quat[3],
            roll=rpy[0], pitch=rpy[1], yaw=rpy[2],
            accel_norm_mps2=a_norm, accel_norm_g=accel_g,
            gyro_norm_rads=w_norm, gyro_norm_dps=gyro_dps,
            accel_delta_mps2=da, jerk_mps3=jerk,
            angular_accel_rads2=angular_accel,
            flags=';'.join(flags),
        )
        self.writer.writerow(values)
        if flags:
            self.evwriter.writerow([
                wall_iso, now-self.start_monotonic, ';'.join(flags),
                accel_g, gyro_dps, jerk, dt,
                stamp_ns/1e9 if stamp_ns > 0 else '',
            ])
        self.n += 1
        self.window_n += 1
        for flag in flags:
            self.flags_count[flag] += 1
            self.window_flags[flag] += 1
        for key, v in (('accel_norm_g', accel_g),
                       ('gyro_norm_dps', gyro_dps),
                       ('jerk_mps3', jerk),
                       ('angular_accel_rads2', angular_accel),
                       ('dt_used_s', dt)):
            if math.isfinite(v):
                self.maxima[key] = max(self.maxima[key], v)
        if math.isfinite(accel_g):
            self.window_max_accel_g = max(self.window_max_accel_g, accel_g)
        if math.isfinite(gyro_dps):
            self.window_max_gyro_dps = max(self.window_max_gyro_dps, gyro_dps)
        if math.isfinite(jerk):
            self.window_max_jerk = max(self.window_max_jerk, jerk)
        if math.isfinite(dt):
            self.window_max_dt_ms = max(self.window_max_dt_ms, dt*1000)
        self.prev_mono = now
        self.prev_stamp_ns = stamp_ns
        self.prev_a = acc
        self.prev_w = gyro
        if now-self.last_report >= self.args.report_period:
            duration = now-self.last_report
            self.log('[{}] {} msgs ({:.1f} Hz wall), peak_a={:.2f}g, '
                     'peak_gyro={:.0f} deg/s, peak_jerk={:.0f} m/s^3, '
                     'max_dt={:.1f}ms, events={}'.format(
                         self.title, self.window_n,
                         self.window_n/max(duration, 1e-6),
                         self.window_max_accel_g,
                         self.window_max_gyro_dps,
                         self.window_max_jerk,
                         self.window_max_dt_ms,
                         dict(self.window_flags)))
            self.fp.flush()
            self.evfp.flush()
            self.window_n = 0
            self.window_max_accel_g = 0.
            self.window_max_gyro_dps = 0.
            self.window_max_jerk = 0.
            self.window_max_dt_ms = 0.
            self.window_flags.clear()
            self.last_report = now

    def close(self):
        if self.closed:
            return
        self.closed = True
        duration = max(0.00001, time.monotonic()-self.start_monotonic)
        summary = [
            self.title,
            'CSV: ' + self.file_path,
            'Events: ' + self.events_path,
            'Duration (real seconds): {:.3f}'.format(duration),
            'Messages: {}'.format(self.n),
            'Average receive frequency: {:.2f} Hz'.format(self.n/duration),
        ]
        for k,v in self.maxima.items():
            summary.append('Max {}: {:.6f}'.format(k, v))
        summary.append('Event counts: {}'.format(dict(self.flags_count)))
        summary.append('Thresholds: accel_norm >= {} g, gyro_norm >= {} deg/s, '
                       'jerk >= {} m/s^3, header gap >= {} ms'.format(
                           self.args.accel_spike_g, self.args.gyro_spike_dps,
                           self.args.jerk_threshold, self.args.gap_ms))
        summary.append('Near-range events are only hints, not proof of saturation.')
        try:
            self.fp.flush()
            self.evfp.flush()
            self.fp.close()
            self.evfp.close()
            with open(self.summary_path, 'w') as fh:
                fh.write('\n'.join(summary)+'\n')
            self.log('\n'.join(summary))
        except Exception as e:
            self.log('Error closing output files: {}'.format(e))


class RecorderNode(Node):
    def __init__(self, args):
        super().__init__('ros2_imu_impact_recorder')
        self.monitor = ImuCsvAnalyzer(
            args, 'ROS2 EngineAI MuJoCo IMU',
            lambda text: self.get_logger().info(text))
        # BEST_EFFORT subscriber receives both BEST_EFFORT and RELIABLE IMU publishers.
        qos = QoSProfile(depth=200,
                         history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(ImuInfo, args.topic, self.on_imu, qos)
        self.get_logger().info('Subscribed to {} (interface_protocol/msg/ImuInfo)'.format(args.topic))

    def on_imu(self, m):
        q = m.quaternion
        a = m.linear_acceleration
        w = m.angular_velocity
        ts = m.header.stamp
        self.monitor.add(ts.sec*1000000000 + ts.nanosec,
                         (a.x, a.y, a.z), (w.x, w.y, w.z),
                         (q.x, q.y, q.z, q.w),
                         (m.rpy.x, m.rpy.y, m.rpy.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument('--topic', default=DEFAULT_TOPIC)
    parser.add_argument('--output', default=default_csv_path())
    parser.add_argument('--report-period', type=float, default=1.0,
                        help='Seconds between console summaries (wall time)')
    parser.add_argument('--accel-spike-g', type=float, default=2.5,
                        help='Magnitude threshold in g: exploratory only')
    parser.add_argument('--gyro-spike-dps', type=float, default=300.0,
                        help='Angular speed magnitude threshold deg/s')
    parser.add_argument('--jerk-threshold', type=float, default=500.0,
                        help='Acceleration step / sample dt, m/s^3')
    parser.add_argument('--gap-ms', type=float, default=GAP_MS,
                        help='Header stamp gap event threshold in ms')
    parser.add_argument('--accel-range-g', type=float, default=0.0,
                        help='Sensor accelerometer full-scale per-axis +/-g; 0=unknown')
    parser.add_argument('--gyro-range-dps', type=float, default=0.0,
                        help='Sensor gyro full-scale per-axis +/-deg/s; 0=unknown')
    parser.add_argument('--saturation-fraction', type=float, default=0.98,
                        help='Flag axes approaching reported full-scale range')

    args = parser.parse_args()
    # Parse normal options above; ROS middleware environment is already sourced.
    rclpy.init(args=[])
    node = None
    try:
        node = RecorderNode(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.monitor.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
