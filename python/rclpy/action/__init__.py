"""rclpy.action shim: goals execute synchronously in simulated time.

The server's execute_callback runs when the client asks for the result
(get_result_async) or at the next spin, so time.sleep() inside it advances sim time.
"""
from mentorpi_sim.core import RT
from ..task import Future


class GoalStatus:
    STATUS_UNKNOWN = 0
    STATUS_ACCEPTED = 1
    STATUS_EXECUTING = 2
    STATUS_CANCELING = 3
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5
    STATUS_ABORTED = 6


class GoalResponse:
    REJECT = 0
    ACCEPT = 1


class CancelResponse:
    REJECT = 0
    ACCEPT = 1


class _FeedbackMessage:
    def __init__(self, feedback):
        self.feedback = feedback


class _ResultResponse:
    def __init__(self, result, status):
        self.result = result
        self.status = status


class ServerGoalHandle:
    def __init__(self, server, request):
        self.request = request
        self._server = server
        self._status = GoalStatus.STATUS_EXECUTING
        self.feedback_callback = None
        self.is_cancel_requested = False

    @property
    def is_active(self):
        return self._status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)

    @property
    def status(self):
        return self._status

    def publish_feedback(self, feedback):
        if self.feedback_callback:
            self.feedback_callback(_FeedbackMessage(feedback))

    def succeed(self):
        self._status = GoalStatus.STATUS_SUCCEEDED

    def abort(self):
        self._status = GoalStatus.STATUS_ABORTED

    def canceled(self):
        self._status = GoalStatus.STATUS_CANCELED

    def execute(self):
        pass


class ClientGoalHandle:
    def __init__(self, server, goal, feedback_callback):
        self.accepted = True
        self.status = GoalStatus.STATUS_ACCEPTED
        self._server = server
        self._goal = goal
        self._fb = feedback_callback
        self._result_future = None

    def _run(self):
        if self._result_future is not None:
            return self._result_future
        self._result_future = Future()
        sgh = ServerGoalHandle(self._server, self._goal)
        sgh.feedback_callback = self._fb
        result = self._server.execute_callback(sgh)
        if sgh._status == GoalStatus.STATUS_EXECUTING:
            sgh._status = GoalStatus.STATUS_ABORTED
        self.status = sgh._status
        self._result_future.set_result(_ResultResponse(result, sgh._status))
        return self._result_future

    def get_result_async(self):
        return self._run()

    def get_result(self):
        return self._run().result()

    def cancel_goal_async(self):
        f = Future()
        f.set_result(None)
        return f


class ActionServer:
    def __init__(self, node, action_type, action_name, execute_callback,
                 goal_callback=None, cancel_callback=None, **kwargs):
        self.action_type = action_type
        self.name = action_name if action_name.startswith("/") else "/" + action_name
        self.execute_callback = execute_callback
        self.goal_callback = goal_callback
        RT.actions[self.name] = self

    def destroy(self):
        RT.actions.pop(self.name, None)


class ActionClient:
    def __init__(self, node, action_type, action_name, **kwargs):
        self.action_type = action_type
        self.name = action_name if action_name.startswith("/") else "/" + action_name

    def server_is_ready(self):
        return self.name in RT.actions

    def wait_for_server(self, timeout_sec=None):
        return self.server_is_ready()

    def send_goal_async(self, goal, feedback_callback=None, goal_uuid=None):
        fut = Future()
        server = RT.actions.get(self.name)
        if server is None:
            return fut
        if server.goal_callback is not None and server.goal_callback(goal) == GoalResponse.REJECT:
            h = ClientGoalHandle(server, goal, feedback_callback)
            h.accepted = False
            fut.set_result(h)
            return fut
        h = ClientGoalHandle(server, goal, feedback_callback)
        RT.soon.append(h._run)          # runs at next spin if nobody asks for the result
        fut.set_result(h)
        return fut

    def send_goal(self, goal, feedback_callback=None):
        h = self.send_goal_async(goal, feedback_callback).result()
        return h.get_result() if h and h.accepted else None

    def destroy(self):
        pass
