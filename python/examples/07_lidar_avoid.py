# title: 라이다 장애물 회피
# group: 라이다
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

import mentorpi_sim as sim                              # 시뮬 전용
sim.setup(world="arena", duration=30)                   # 시뮬 전용

THRESHOLD = 0.45       # 이보다 가까우면 회피 [m]
SPEED = 0.2


def sector_min(scan, deg_from, deg_to):
    """[deg_from, deg_to] (0 = 전방, 왼쪽 +) 구간의 최소 거리."""
    out = float("inf")
    for i, r in enumerate(scan.ranges):
        a = math.degrees(scan.angle_min + i * scan.angle_increment)
        if deg_from <= a <= deg_to and scan.range_min < r < out:
            out = r
    return out


class Avoid(Node):
    def __init__(self):
        super().__init__("lidar_avoid")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.turning = 0          # 회전 방향 기억 (+1 왼쪽, -1 오른쪽) — 좌우로 떨지 않게

    def on_scan(self, scan):
        front = sector_min(scan, -30, 30)
        left = sector_min(scan, 30, 90)
        right = sector_min(scan, -90, -30)
        cmd = Twist()
        if self.turning and front < THRESHOLD + 0.15:
            cmd.angular.z = 0.5 * self.turning                         # 앞이 충분히 트일 때까지 계속 회전
        elif front > THRESHOLD:
            self.turning = 0
            cmd.linear.x = SPEED
            cmd.angular.z = 0.3 * (min(left, 1.0) - min(right, 1.0))   # 넓은 쪽으로 살짝
        else:
            self.turning = 1 if left > right else -1                   # 제자리 회전 시작
            cmd.angular.z = 0.5 * self.turning
        self.pub.publish(cmd)


rclpy.init()
node = Avoid()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()
