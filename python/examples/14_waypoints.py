# title: 다중 지점 순회 (오도메트리 기반 P 제어)
# group: SLAM·내비게이션
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

import mentorpi_sim as sim                                   # 시뮬 전용
sim.setup(world="empty", duration=40, noise=False)           # 시뮬 전용

WAYPOINTS = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]   # odom 좌표 [m]
TOL = 0.05


class Waypoints(Node):
    def __init__(self):
        super().__init__("waypoint_follower")
        self.pub = self.create_publisher(Twist, "controller/cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.i = 0

    def on_odom(self, msg):
        if self.i >= len(WAYPOINTS):
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        gx, gy = WAYPOINTS[self.i]
        dx, dy = gx - p.x, gy - p.y
        dist = math.hypot(dx, dy)
        cmd = Twist()
        if dist < TOL:
            self.get_logger().info("지점 %d (%.1f, %.1f) 도착" % (self.i + 1, gx, gy))
            self.i += 1
            if self.i == len(WAYPOINTS):
                self.get_logger().info("모든 지점 완료")
                rclpy.shutdown()
        else:
            err = math.atan2(math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw))
            cmd.angular.z = max(-1.0, min(1.0, 2.0 * err))
            cmd.linear.x = 0.0 if abs(err) > 0.5 else min(0.25, 0.6 * dist)
        self.pub.publish(cmd)


rclpy.init()
node = Waypoints()
try:
    rclpy.spin(node)
finally:
    node.pub.publish(Twist())
    node.destroy_node()
