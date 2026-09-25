# title: 파라미터와 TF 조회 (odom → lidar_frame)
# group: ROS 2 기초
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

import mentorpi_sim as sim                        # 시뮬 전용
sim.setup(world="empty", duration=6)              # 시뮬 전용


class Mover(Node):
    def __init__(self):
        super().__init__("param_tf_demo")
        self.declare_parameter("speed", 0.15)
        self.declare_parameter("turn", 0.4)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.create_timer(0.1, self.drive)
        self.create_timer(1.0, self.report)

    def drive(self):
        m = Twist()
        m.linear.x = self.get_parameter("speed").value
        m.angular.z = self.get_parameter("turn").value
        self.pub.publish(m)

    def report(self):
        tf = self.buf.lookup_transform("odom", "lidar_frame", rclpy.time.Time())
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.get_logger().info("lidar_frame in odom: x=%.2f y=%.2f z=%.3f yaw=%.0f°" % (t.x, t.y, t.z, math.degrees(yaw)))


rclpy.init()
node = Mover()
node.set_parameters([Parameter("turn", Parameter.Type.DOUBLE, -0.4)])   # 실행 중 변경
print("speed =", node.get_parameter("speed").value, "turn =", node.get_parameter("turn").value)
try:
    rclpy.spin(node)
finally:
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()
