# title: 토픽 발행·구독 (String)
# group: ROS 2 기초
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__("talker")
        self.pub = self.create_publisher(String, "chatter", 10)
        self.i = 0
        self.create_timer(1.0, self.tick)

    def tick(self):
        msg = String()
        msg.data = "hello %d" % self.i
        self.pub.publish(msg)
        self.get_logger().info('발행: "%s"' % msg.data)
        self.i += 1


class Listener(Node):
    def __init__(self):
        super().__init__("listener")
        self.create_subscription(String, "chatter", self.on_msg, 10)

    def on_msg(self, msg):
        self.get_logger().info('수신: "%s"' % msg.data)


import mentorpi_sim as sim           # 시뮬 전용
sim.setup(duration=4.5)              # 시뮬 전용

rclpy.init()
talker, listener = Talker(), Listener()
# 실물에서는 두 노드를 별도 터미널에서 실행하거나 MultiThreadedExecutor를 쓴다
from rclpy.executors import SingleThreadedExecutor
ex = SingleThreadedExecutor()
ex.add_node(talker)
ex.add_node(listener)
try:
    ex.spin()
except KeyboardInterrupt:
    pass
finally:
    talker.destroy_node()
    listener.destroy_node()
    rclpy.shutdown()
