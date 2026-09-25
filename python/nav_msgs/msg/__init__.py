from mentorpi_sim._msg import msg
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose

Odometry = msg("nav_msgs", "msg", "Odometry", header=Header, child_frame_id="",
               pose=PoseWithCovariance, twist=TwistWithCovariance)
Path = msg("nav_msgs", "msg", "Path", header=Header, poses=list)
MapMetaData = msg("nav_msgs", "msg", "MapMetaData", map_load_time=Time, resolution=0.0,
                  width=0, height=0, origin=Pose)
OccupancyGrid = msg("nav_msgs", "msg", "OccupancyGrid", header=Header, info=MapMetaData, data=list)
