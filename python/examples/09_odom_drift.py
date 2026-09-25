# title: 오도메트리 오차 측정과 보정 계수
# group: 모션 제어
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry

import mentorpi_sim as sim                                          # 시뮬 전용
sim.setup(world="empty", duration=20, start=(-2.5, 0, 0))           # 시뮬 전용

rclpy.init()
node = Node("odom_check")
pub = node.create_publisher(Twist, "cmd_vel", 10)
state = {}
node.create_subscription(Odometry, "odom", lambda m: state.update(odom=m.pose.pose.position.x), 10)
node.create_subscription(Pose2D, "ground_truth", lambda m: state.update(gt=m.x), 10)   # 시뮬 전용 토픽

rclpy.spin_once(node, timeout_sec=0.2)
rclpy.spin_once(node, timeout_sec=0.2)
gt0 = state["gt"]
cmd = Twist()
cmd.linear.x = 0.2
t_end = time.time() + 10.0                 # 0.2 m/s × 10 s = 2.0 m 명령
while time.time() < t_end:
    pub.publish(cmd)
    rclpy.spin_once(node, timeout_sec=0.1)
pub.publish(Twist())
for _ in range(10):
    rclpy.spin_once(node, timeout_sec=0.1)

odom_d = state["odom"]
real_d = state["gt"] - gt0
print("오도메트리가 말한 거리: %.3f m" % odom_d)
print("실제로 간 거리(줄자):   %.3f m" % real_d)
print("linear_correction_factor ≈ 실제/오도메트리 = %.3f" % (real_d / odom_d))
node.destroy_node()
rclpy.shutdown()
