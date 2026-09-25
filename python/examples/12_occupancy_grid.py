# title: 점유 격자 지도 만들기 (SLAM의 매핑 절반)
# group: SLAM·내비게이션
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan

import mentorpi_sim as sim                             # 시뮬 전용
sim.setup(world="room", duration=25)                   # 시뮬 전용

RES = 0.05                     # 격자 해상도 5 cm
X0, Y0, W, H = -3.2, -2.2, 128, 88
L_OCC, L_FREE = 0.85, -0.4     # 로그 오즈 증분
logodds = np.zeros((H, W))


def to_cell(x, y):
    return int((x - X0) / RES), int((y - Y0) / RES)


class Mapper(Node):
    def __init__(self):
        super().__init__("mini_mapper")
        self.pose = None
        self.front = self.left = self.right = 9.0
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Pose2D, "ground_truth", self.on_pose, 10)   # 위치는 안다고 가정(시뮬 전용)
        self.create_subscription(LaserScan, "scan", self.on_scan, 10)
        self.create_timer(0.1, self.explore)

    def on_pose(self, m):
        self.pose = (m.x, m.y, m.theta)

    def explore(self):                               # 간단한 회피 탐색
        m = Twist()
        if self.front > 0.5:
            m.linear.x, m.angular.z = 0.2, 0.25 * (min(self.left, 1.5) - min(self.right, 1.5))
        else:
            m.angular.z = 0.5 if self.left > self.right else -0.5
        self.pub.publish(m)

    def on_scan(self, scan):
        if self.pose is None:
            return
        deg = lambda i: math.degrees(scan.angle_min + i * scan.angle_increment)
        near = lambda a, b: min([r for i, r in enumerate(scan.ranges) if a <= deg(i) <= b] + [9.0])
        self.front, self.left, self.right = near(-35, 35), near(35, 100), near(-100, -35)
        x, y, th = self.pose
        cx, cy = to_cell(x, y)
        for i in range(0, len(scan.ranges), 3):
            r = scan.ranges[i]
            hit = math.isfinite(r)
            r = min(r, 4.0) if hit else 4.0
            a = th + scan.angle_min + i * scan.angle_increment
            ex, ey = to_cell(x + r * math.cos(a), y + r * math.sin(a))
            n = max(abs(ex - cx), abs(ey - cy), 1)
            for k in range(n):                       # 빔이 지나간 칸 = 비어 있음
                gx, gy = cx + (ex - cx) * k // n, cy + (ey - cy) * k // n
                if 0 <= gx < W and 0 <= gy < H:
                    logodds[gy, gx] += L_FREE
            if hit and 0 <= ex < W and 0 <= ey < H:  # 빔 끝 = 점유
                logodds[ey, ex] += L_OCC


rclpy.init()
node = Mapper()
try:
    rclpy.spin(node)
finally:
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()

p = 1 - 1 / (1 + np.exp(np.clip(logodds, -10, 10)))
occ, free = (p > 0.65).sum(), (p < 0.35).sum()
print("격자 %d×%d, 점유 %d칸, 빈칸 %d칸, 미확인 %d칸" % (W, H, occ, free, W * H - occ - free))
try:
    import cv2
    img = np.full((H, W), 128, np.uint8)
    img[p < 0.35] = 255
    img[p > 0.65] = 0
    cv2.imshow("occupancy grid", cv2.resize(np.flipud(img), (W * 4, H * 4), interpolation=cv2.INTER_NEAREST))
except ImportError:
    pass
