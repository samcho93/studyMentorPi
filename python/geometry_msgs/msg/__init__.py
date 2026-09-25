from mentorpi_sim._msg import msg
from std_msgs.msg import Header

Vector3 = msg("geometry_msgs", "msg", "Vector3", x=0.0, y=0.0, z=0.0)
Point = msg("geometry_msgs", "msg", "Point", x=0.0, y=0.0, z=0.0)
Quaternion = msg("geometry_msgs", "msg", "Quaternion", x=0.0, y=0.0, z=0.0, w=1.0)
Pose = msg("geometry_msgs", "msg", "Pose", position=Point, orientation=Quaternion)
Pose2D = msg("geometry_msgs", "msg", "Pose2D", x=0.0, y=0.0, theta=0.0)
PoseStamped = msg("geometry_msgs", "msg", "PoseStamped", header=Header, pose=Pose)
PoseWithCovariance = msg("geometry_msgs", "msg", "PoseWithCovariance", pose=Pose,
                         covariance=lambda: [0.0] * 36)
PoseWithCovarianceStamped = msg("geometry_msgs", "msg", "PoseWithCovarianceStamped",
                                header=Header, pose=PoseWithCovariance)
Twist = msg("geometry_msgs", "msg", "Twist", linear=Vector3, angular=Vector3)
TwistWithCovariance = msg("geometry_msgs", "msg", "TwistWithCovariance", twist=Twist,
                          covariance=lambda: [0.0] * 36)
TwistStamped = msg("geometry_msgs", "msg", "TwistStamped", header=Header, twist=Twist)
Transform = msg("geometry_msgs", "msg", "Transform", translation=Vector3, rotation=Quaternion)
TransformStamped = msg("geometry_msgs", "msg", "TransformStamped", header=Header,
                       child_frame_id="", transform=Transform)
