from mentorpi_sim._msg import msg
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3

LaserScan = msg("sensor_msgs", "msg", "LaserScan", header=Header, angle_min=0.0, angle_max=0.0,
                angle_increment=0.0, time_increment=0.0, scan_time=0.0, range_min=0.0,
                range_max=0.0, ranges=list, intensities=list)
Imu = msg("sensor_msgs", "msg", "Imu", header=Header, orientation=Quaternion,
          orientation_covariance=lambda: [0.0] * 9, angular_velocity=Vector3,
          angular_velocity_covariance=lambda: [0.0] * 9, linear_acceleration=Vector3,
          linear_acceleration_covariance=lambda: [0.0] * 9)

RegionOfInterest = msg("sensor_msgs", "msg", "RegionOfInterest", x_offset=0, y_offset=0, height=0, width=0,
                       do_rectify=False)
Image = msg("sensor_msgs", "msg", "Image", header=Header, height=0, width=0, encoding="", is_bigendian=0,
            step=0, data=bytes)
CompressedImage = msg("sensor_msgs", "msg", "CompressedImage", header=Header, format="", data=bytes)
CameraInfo = msg("sensor_msgs", "msg", "CameraInfo", header=Header, height=0, width=0, distortion_model="",
                 d=list, k=lambda: [0.0] * 9, r=lambda: [0.0] * 9, p=lambda: [0.0] * 12, binning_x=0,
                 binning_y=0, roi=RegionOfInterest)
PointField = msg("sensor_msgs", "msg", "PointField", name="", offset=0, datatype=0, count=0)
PointField.INT8, PointField.UINT8, PointField.INT16, PointField.UINT16 = 1, 2, 3, 4
PointField.INT32, PointField.UINT32, PointField.FLOAT32, PointField.FLOAT64 = 5, 6, 7, 8
PointCloud2 = msg("sensor_msgs", "msg", "PointCloud2", header=Header, height=0, width=0, fields=list,
                  is_bigendian=False, point_step=0, row_step=0, data=bytes, is_dense=False)
