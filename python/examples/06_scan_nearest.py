# title: /scan 읽기 — 가장 가까운 장애물
# group: 라이다
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

import mentorpi_sim as sim                          # 시뮬 전용
sim.setup(world="room", duration=1.05)              # 시뮬 전용


class Nearest(Node):
    def __init__(self):
        super().__init__("scan_nearest")
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)

    def on_scan(self, scan):
        best_i, best_r = None, float("inf")
        for i, r in enumerate(scan.ranges):
            if scan.range_min < r < best_r:
                best_i, best_r = i, r
        ang = scan.angle_min + best_i * scan.angle_increment
        front = scan.ranges[len(scan.ranges) // 2]          # angle 0 = 전방
        self.get_logger().info("점 %d개, 가장 가까운 물체 %.2f m @ %+.0f°, 정면 %.2f m"
                               % (len(scan.ranges), best_r, math.degrees(ang), front))


rclpy.init()
node = Nearest()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
