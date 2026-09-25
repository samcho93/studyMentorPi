"""rclpy shim for the studyMentorPi Playground (simulated time, single process).

Same import paths and call signatures as ROS 2 Humble rclpy for the subset listed in
docs/PLAYGROUND_API.md. Code written against it runs unchanged on a real MentorPi.
"""
from mentorpi_sim.core import RT, SimTimeUp

from . import qos, parameter, time, duration, executors  # noqa: F401


def init(args=None, context=None):
    RT.inited = True
    RT.shutdown_flag = False


def ok(context=None):
    return RT.ok()


def shutdown(context=None):
    RT.shutdown_flag = True


def try_shutdown(context=None):
    shutdown()


def spin(node, executor=None):
    RT.spin()


def spin_once(node, executor=None, timeout_sec=None):
    RT.spin_once(timeout_sec)


def spin_until_future_complete(node, future, executor=None, timeout_sec=None):
    end = None if timeout_sec is None else RT.t + timeout_sec
    while not future.done():
        if end is not None and RT.t >= end:
            return
        try:
            RT.spin_once(0.05)
        except SimTimeUp:
            return


def create_node(node_name, **kwargs):
    from .node import Node
    return Node(node_name, **kwargs)


def get_default_context():
    return None
