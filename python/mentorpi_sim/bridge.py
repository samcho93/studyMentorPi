"""Virtual MentorPi sensor publishers: /odom, /imu, /scan, /tf, /ground_truth."""
import math

import numpy as np

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose2D, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

N_RAYS = 360
ANGLES = -math.pi + np.arange(N_RAYS) * (2 * math.pi / N_RAYS)

__all__ = ["publish_odom", "publish_scan", "Pose2D"]


def stamp(t):
    sec = int(t)
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)))


def _quat(msg_q, yaw):
    msg_q.x, msg_q.y = 0.0, 0.0
    msg_q.z, msg_q.w = math.sin(yaw / 2), math.cos(yaw / 2)


def publish_odom(rt, r):
    st = stamp(rt.t)
    od = Odometry()
    od.header.stamp = st
    od.header.frame_id = "odom"
    od.child_frame_id = "base_footprint"
    od.pose.pose.position.x = r.ox
    od.pose.pose.position.y = r.oy
    _quat(od.pose.pose.orientation, r.oyaw)
    od.twist.twist.linear.x = r.v[0]
    od.twist.twist.linear.y = r.v[1]
    od.twist.twist.angular.z = r.v[2]
    rt.publish("/odom", od)

    imu = Imu()
    imu.header.stamp = st
    imu.header.frame_id = "imu_link"
    yaw_imu = r.yaw - rt.start[2]                 # IMU integrates true rotation (no slip)
    _quat(imu.orientation, math.atan2(math.sin(yaw_imu), math.cos(yaw_imu)))
    imu.angular_velocity.z = r.v[2] * r.slip_ang + (float(rt.rng.normal(0, 0.002)) if rt.noise else 0.0)
    imu.linear_acceleration.z = 9.81
    rt.publish("/imu", imu)

    tf = TransformStamped()
    tf.header.stamp = st
    tf.header.frame_id = "odom"
    tf.child_frame_id = "base_footprint"
    tf.transform.translation.x = r.ox
    tf.transform.translation.y = r.oy
    _quat(tf.transform.rotation, r.oyaw)
    rt.publish("/tf", tf)


def publish_scan(rt, r):
    world_angles = ANGLES + r.yaw
    d = rt.world.raycast(r.x, r.y, world_angles, max_range=12.0)
    if rt.noise:
        d = d + rt.rng.normal(0, 0.008, size=d.shape)
    d[d < 0.05] = np.inf
    sc = LaserScan()
    sc.header.stamp = stamp(rt.t)
    sc.header.frame_id = "lidar_frame"
    sc.angle_min = -math.pi
    sc.angle_max = -math.pi + (N_RAYS - 1) * 2 * math.pi / N_RAYS
    sc.angle_increment = 2 * math.pi / N_RAYS
    sc.scan_time = 0.1
    sc.time_increment = 0.1 / N_RAYS
    sc.range_min = 0.05
    sc.range_max = 12.0
    sc.ranges = [float(v) for v in d]
    sc.intensities = [0.0 if math.isinf(v) else 100.0 for v in d]
    rt.publish("/scan", sc)
    return d
