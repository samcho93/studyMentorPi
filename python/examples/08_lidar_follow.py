# title: 라이다 추종 — 움직이는 물체 따라가기
# group: 라이다
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

import mentorpi_sim as sim                               # 시뮬 전용
sim.setup(world="empty", duration=30, start=(-2, 0, 0))  # 시뮬 전용
sim.add_mover(-1.0, 0.0, vx=0.12, vy=0.08)               # 시뮬 전용: 움직이는 목표

KEEP = 0.5            # 유지할 거리 [m]
KP_LIN, KP_ANG = 0.8, 1.5


class Follow(Node):
    def __init__(self):
        super().__init__("lidar_follow")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)

    def on_scan(self, scan):
        # 가장 가까운 점 = 추종 대상 (벽이 멀리 있다고 가정)
        best, idx = float("inf"), 0
        for i, r in enumerate(scan.ranges):
            if scan.range_min < r < best:
                best, idx = r, i
        cmd = Twist()
        if best < 2.0:
            ang = scan.angle_min + idx * scan.angle_increment
            cmd.linear.x = KP_LIN * (best - KEEP)
            cmd.angular.z = KP_ANG * ang
        self.pub.publish(cmd)


rclpy.init()
node = Follow()
try:
    rclpy.spin(node)
finally:
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()
