from mentorpi_sim.core import RT


class Executor:
    def __init__(self, num_threads=None, context=None):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)
        return True

    def remove_node(self, node):
        if node in self.nodes:
            self.nodes.remove(node)

    def spin(self):
        RT.spin()

    def spin_once(self, timeout_sec=None):
        RT.spin_once(timeout_sec)

    def spin_until_future_complete(self, future, timeout_sec=None):
        import rclpy
        rclpy.spin_until_future_complete(None, future, timeout_sec=timeout_sec)

    def shutdown(self, timeout_sec=None):
        return True


class SingleThreadedExecutor(Executor):
    pass


class MultiThreadedExecutor(Executor):
    pass
