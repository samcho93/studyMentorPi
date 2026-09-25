# title: 메카넘 전방향 이동 + 바퀴 속도 계산
# group: 모션 제어
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import mentorpi_sim as sim                                   # 시뮬 전용
sim.setup(chassis="mecanum", world="empty", duration=16)     # 시뮬 전용

# MentorPi controller/mecanum.py 와 같은 역기구학
A, B, D = 0.1368, 0.1446, 0.065        # wheelbase, track_width, wheel diameter


def wheel_rps(vx, vy, wz):
    k = (A + B) / 2
    m1 = vx - vy - wz * k      # 왼쪽 앞
    m2 = vx + vy - wz * k      # 왼쪽 뒤
    m3 = vx + vy + wz * k      # 오른쪽 앞
    m4 = vx - vy + wz * k      # 오른쪽 뒤
    return [v / (math.pi * D) for v in (-m1, -m2, m3, m4)]   # 모터 장착 방향 때문에 왼쪽 부호 반전


rclpy.init()
node = Node("omni")
pub = node.create_publisher(Twist, "cmd_vel", 10)
moves = [("전진", 0.2, 0.0, 0.0), ("왼쪽 옆으로", 0.0, 0.2, 0.0), ("대각선 뒤-오른쪽", -0.14, -0.14, 0.0),
         ("제자리 회전", 0.0, 0.0, 0.5)]
try:
    for name, vx, vy, wz in moves:
        rps = wheel_rps(vx, vy, wz)
        node.get_logger().info("%s: 바퀴 rps = %s" % (name, ", ".join("%+.2f" % r for r in rps)))
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        for _ in range(30):
            pub.publish(msg)
            time.sleep(0.1)
        pub.publish(Twist())
        time.sleep(0.5)
finally:
    pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()
