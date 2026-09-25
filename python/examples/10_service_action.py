# title: 서비스(AddTwoInts)와 액션(Fibonacci)
# group: ROS 2 기초
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from example_interfaces.srv import AddTwoInts
from example_interfaces.action import Fibonacci


class Server(Node):
    def __init__(self):
        super().__init__("server")
        self.create_service(AddTwoInts, "add_two_ints", self.add)
        ActionServer(self, Fibonacci, "fibonacci", self.execute)

    def add(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info("요청 a=%d b=%d → sum=%d" % (request.a, request.b, response.sum))
        return response

    def execute(self, goal_handle):
        seq = [0, 1]
        for i in range(1, goal_handle.request.order):
            seq.append(seq[i] + seq[i - 1])
            fb = Fibonacci.Feedback()
            fb.sequence = seq
            goal_handle.publish_feedback(fb)
            time.sleep(0.5)
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = seq
        return result


rclpy.init()
server = Server()
client = Node("client")

cli = client.create_client(AddTwoInts, "add_two_ints")
cli.wait_for_service(timeout_sec=1.0)
future = cli.call_async(AddTwoInts.Request(a=41, b=1))
rclpy.spin_until_future_complete(client, future)
print("서비스 응답:", future.result().sum)

ac = ActionClient(client, Fibonacci, "fibonacci")
ac.wait_for_server()
goal = Fibonacci.Goal()
goal.order = 6
goal_handle = ac.send_goal_async(goal, feedback_callback=lambda m: print("피드백:", m.feedback.sequence)).result()
result = goal_handle.get_result_async().result().result
print("액션 결과:", result.sequence)
server.destroy_node()
client.destroy_node()
rclpy.shutdown()
