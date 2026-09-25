"""Virtual MentorPi sensor publishers: /odom, /imu, /scan, /tf, /ground_truth."""
import math

import numpy as np

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose2D, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan

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


CAM_NS = "/ascamera/camera_publisher"


def publish_camera(rt, r):
    """RGB (rgb8), depth (16UC1, mm), camera_info and an xyz point cloud — HP60C topic names."""
    from . import rgbd as camera
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    w, h = rt.cam_res
    rgb, depth = camera.render(rt.world, (r.x, r.y, r.yaw), w, h, rt.noise, rt.rng)
    st = stamp(rt.t)
    fx, fy, cx, cy = camera.intrinsics(w, h)
    subs = rt.subs

    def info(frame):
        ci = CameraInfo()
        ci.header.stamp, ci.header.frame_id = st, frame
        ci.height, ci.width, ci.distortion_model = h, w, "plumb_bob"
        ci.d = [0.0] * 5
        ci.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return ci

    def image(arr, enc, frame):
        m = Image()
        m.header.stamp, m.header.frame_id = st, frame
        m.height, m.width, m.encoding = arr.shape[0], arr.shape[1], enc
        m.step = arr.strides[0]
        m.data = arr.tobytes()
        return m

    if CAM_NS + "/rgb0/image" in subs:
        rt.publish(CAM_NS + "/rgb0/image", image(np.ascontiguousarray(rgb), "rgb8", "ascamera_color_0"))
    if CAM_NS + "/depth0/image_raw" in subs:
        mm = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
        rt.publish(CAM_NS + "/depth0/image_raw", image(mm, "16UC1", "ascamera_camera_link_0"))
    if CAM_NS + "/rgb0/camera_info" in subs:
        rt.publish(CAM_NS + "/rgb0/camera_info", info("ascamera_color_0"))
    if CAM_NS + "/depth0/camera_info" in subs:
        rt.publish(CAM_NS + "/depth0/camera_info", info("ascamera_camera_link_0"))
    if CAM_NS + "/depth0/points" in subs:
        from std_msgs.msg import Header
        hd = Header()
        hd.stamp, hd.frame_id = st, "ascamera_camera_link_0"
        rt.publish(CAM_NS + "/depth0/points", create_cloud_xyz32(hd, camera.backproject(depth, step=2)))
