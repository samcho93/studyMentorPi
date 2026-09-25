# title: 애커만 — 원 주행과 조향각
# group: 모션 제어
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import mentorpi_sim as sim                                                   # 시뮬 전용
sim.setup(chassis="ackermann", world="empty", duration=25, start=(0, -1.2, 0))  # 시뮬 전용

L = 0.145                      # wheelbase [m] (controller/ackermann.py)
V, R = 0.2, 0.8                # 속도, 원하는 회전 반경
W = V / R                      # ω = v / R
delta = math.atan(L * W / V)   # 조향각
print("ω = %.3f rad/s, 조향각 δ = %.1f°, 서보 펄스 = %d us" % (W, math.degrees(delta), 1500 + 2000 * math.degrees(-delta) / 180))


class Circle(Node):
    def __init__(self):
        super().__init__("ackermann_circle")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.create_timer(0.1, self.tick)
        self.xy = []

    def tick(self):
        m = Twist()
        m.linear.x, m.angular.z = V, W
        self.pub.publish(m)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.xy.append((p.x, p.y))


rclpy.init()
node = Circle()
try:
    rclpy.spin(node)
finally:
    node.pub.publish(Twist())
    xs = [p[0] for p in node.xy]
    ys = [p[1] for p in node.xy]
    print("odom x 범위 %.2f~%.2f m, y 범위 %.2f~%.2f m → 지름 ≈ %.2f m" % (min(xs), max(xs), min(ys), max(ys), max(ys) - min(ys)))
    sim.plot(xs, ys, "odom 궤적 (x-y)")          # 시뮬 전용
    node.destroy_node()
    rclpy.shutdown()
