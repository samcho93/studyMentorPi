"""Subset of ROS 2 sensor_msgs_py.point_cloud2 (float32 x, y, z [+ rgb] clouds)."""
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField


def create_cloud_xyz32(header, points):
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    m = PointCloud2()
    m.header = header
    m.height, m.width = 1, len(pts)
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
    m.is_bigendian = False
    m.point_step, m.row_step = 12, 12 * len(pts)
    m.data = pts.tobytes()
    m.is_dense = True
    return m


def read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=False):
    step = cloud.point_step
    raw = np.frombuffer(bytes(cloud.data), dtype=np.uint8).reshape(-1, step)
    offs = {f.name: f.offset for f in cloud.fields}
    cols = [raw[:, offs[n]:offs[n] + 4].copy().view(np.float32)[:, 0] for n in field_names]
    out = np.stack(cols, axis=1)
    if skip_nans:
        out = out[~np.isnan(out).any(axis=1)]
    return out


def read_points(cloud, field_names=("x", "y", "z"), skip_nans=False, uvs=None):
    return [tuple(float(v) for v in row) for row in read_points_numpy(cloud, field_names, skip_nans)]


def read_points_list(cloud, field_names=("x", "y", "z"), skip_nans=False):
    return read_points(cloud, field_names, skip_nans)
