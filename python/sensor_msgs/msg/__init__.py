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
