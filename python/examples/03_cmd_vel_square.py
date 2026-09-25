# title: /cmd_vel — 정사각형 주행 (메카넘)
# group: 모션 제어
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import mentorpi_sim as sim                                  # 시뮬 전용
sim.setup(chassis="mecanum", world="empty", duration=30)    # 시뮬 전용

SIDE = 1.0          # 한 변 [m]
V = 0.2             # 선속도 [m/s] (실물 /cmd_vel 상한)
W = 0.5             # 각속도 [rad/s]

rclpy.init()
node = Node("square_driver")
pub = node.create_publisher(Twist, "cmd_vel", 10)


def drive(vx=0.0, wz=0.0, seconds=1.0):
    msg = Twist()
    msg.linear.x, msg.angular.z = vx, wz
    t_end = time.time() + seconds
    while time.time() < t_end:          # 10 Hz로 계속 보내야 한다 (명령이 끊기면 멈추는 드라이버 대비)
        pub.publish(msg)
        time.sleep(0.1)


try:
    for k in range(4):
        node.get_logger().info("변 %d: 직진" % (k + 1))
        drive(vx=V, seconds=SIDE / V)
        drive(seconds=0.3)                             # 잠깐 정지
        drive(wz=W, seconds=(math.pi / 2) / W)         # 90° 회전
        drive(seconds=0.3)
except KeyboardInterrupt:
    pass
finally:
    pub.publish(Twist())                               # 반드시 정지 명령
    node.destroy_node()
    rclpy.shutdown()
