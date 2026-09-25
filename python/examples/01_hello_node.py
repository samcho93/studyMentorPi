# title: 첫 노드 — 타이머와 로거
# group: ROS 2 기초
import rclpy
from rclpy.node import Node


class Hello(Node):
    def __init__(self):
        super().__init__("hello_mentorpi")
        self.count = 0
        self.timer = self.create_timer(0.5, self.on_timer)   # 2 Hz

    def on_timer(self):
        self.count += 1
        self.get_logger().info("Hello MentorPi! %d" % self.count)
        if self.count == 5:
            self.timer.cancel()
            rclpy.shutdown()                                # spin()을 끝낸다


rclpy.init()
node = Hello()
rclpy.spin(node)
node.destroy_node()
