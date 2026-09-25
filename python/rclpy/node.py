"""rclpy.node.Node shim."""
import copy

from mentorpi_sim.core import RT
from .clock import Clock
from .logging import RcutilsLogger
from .parameter import Parameter, SetParametersResult
from .task import Future


def _topic(name, node=None):
    if name.startswith("~") and node is not None:
        return "/" + node.get_name() + name[1:]
    return name if name.startswith("/") else "/" + name


def _reliability(qos):
    from .qos import ReliabilityPolicy
    r = getattr(qos, "reliability", ReliabilityPolicy.RELIABLE)
    return "best_effort" if r == ReliabilityPolicy.BEST_EFFORT else "reliable"


def _depth(qos):
    if isinstance(qos, int):
        return max(1, qos)
    return max(1, getattr(qos, "depth", 10) or 10)


class Publisher:
    def __init__(self, node, msg_type, topic, qos):
        self.msg_type = msg_type
        self.topic_name = _topic(topic, node)
        self.node = node
        self.reliability = _reliability(qos)

    def publish(self, msg):
        if not isinstance(msg, self.msg_type):
            raise TypeError("The message type provided is not the same as the publisher's type: "
                            "expected %s, got %s" % (self.msg_type.__name__, type(msg).__name__))
        RT.publish(self.topic_name, msg, reliability=self.reliability)

    def get_subscription_count(self):
        return len(RT.subs.get(self.topic_name, [])) + len(RT.internal.get(self.topic_name, []))

    def destroy(self):
        pass


class Subscription:
    def __init__(self, node, msg_type, topic, callback, qos):
        self.msg_type = msg_type
        self.topic_name = _topic(topic, node)
        self.callback = callback
        self.depth = _depth(qos)
        self.reliability = _reliability(qos)
        self.queue = []
        RT.subs.setdefault(self.topic_name, []).append(self)

    def _enqueue(self, msg, reliability="reliable"):
        # QoS compatibility: a RELIABLE subscription never matches a BEST_EFFORT publisher
        if self.reliability == "reliable" and reliability == "best_effort":
            return
        self.queue.append((RT.t, msg))
        if len(self.queue) > self.depth:
            self.queue.pop(0)

    def destroy(self):
        lst = RT.subs.get(self.topic_name, [])
        if self in lst:
            lst.remove(self)


class Timer:
    def __init__(self, period, callback):
        self.period = float(period)
        self.callback = callback
        self.next_t = RT.t + self.period
        self.cancelled = False
        RT.timers.append(self)

    def cancel(self):
        self.cancelled = True

    def reset(self):
        self.cancelled = False
        self.next_t = RT.t + self.period

    def is_canceled(self):
        return self.cancelled

    @property
    def timer_period_ns(self):
        return int(self.period * 1e9)

    def destroy(self):
        self.cancel()


class Service:
    def __init__(self, srv_type, name, callback, node=None):
        self.srv_type = srv_type
        self.srv_name = _topic(name, node)
        self.callback = callback
        RT.services[self.srv_name] = self

    def _handle(self, request):
        return self.callback(request, self.srv_type.Response())

    def destroy(self):
        RT.services.pop(self.srv_name, None)


class Client:
    def __init__(self, srv_type, name, node=None):
        self.srv_type = srv_type
        self.srv_name = _topic(name, node)

    def service_is_ready(self):
        return self.srv_name in RT.services

    def wait_for_service(self, timeout_sec=None):
        if self.service_is_ready():
            return True
        if timeout_sec is not None and timeout_sec > 0:
            try:
                RT.advance(timeout_sec)
            except KeyboardInterrupt:
                pass
        return self.service_is_ready()

    def call_async(self, request):
        fut = Future()
        srv = RT.services.get(self.srv_name)
        if srv is None:
            return fut        # never completes, like a missing server
        fut.set_result(srv._handle(copy.deepcopy(request)))
        return fut

    def call(self, request, timeout_sec=None):
        fut = self.call_async(request)
        return fut.result()

    def destroy(self):
        pass


class Node:
    def __init__(self, node_name, *, context=None, cli_args=None, namespace=None,
                 use_global_arguments=True, enable_rosout=True, start_parameter_services=True,
                 parameter_overrides=None, allow_undeclared_parameters=False,
                 automatically_declare_parameters_from_overrides=False):
        if not RT.inited:
            raise RuntimeError("rclpy.init() has not been called")
        self._name = node_name
        self._namespace = namespace or "/"
        self._params = {}
        self._param_callbacks = []
        self._overrides = {p.name: p for p in (parameter_overrides or [])}
        self._logger = RcutilsLogger(node_name)
        self._clock = Clock()
        self.publishers, self.subscriptions, self.timers = [], [], []
        RT.nodes.append(self)

    # ---- info
    def get_name(self):
        return self._name

    def get_namespace(self):
        return self._namespace

    def get_fully_qualified_name(self):
        return (self._namespace.rstrip("/") + "/" + self._name)

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    # ---- comms
    def create_publisher(self, msg_type, topic, qos_profile, **kwargs):
        p = Publisher(self, msg_type, topic, qos_profile)
        self.publishers.append(p)
        return p

    def create_subscription(self, msg_type, topic, callback, qos_profile, **kwargs):
        s = Subscription(self, msg_type, topic, callback, qos_profile)
        self.subscriptions.append(s)
        return s

    def create_timer(self, timer_period_sec, callback, callback_group=None, clock=None):
        t = Timer(timer_period_sec, callback)
        self.timers.append(t)
        return t

    def create_service(self, srv_type, srv_name, callback, **kwargs):
        return Service(srv_type, srv_name, callback, self)

    def create_client(self, srv_type, srv_name, **kwargs):
        return Client(srv_type, srv_name, self)

    def destroy_publisher(self, p):
        return True

    def destroy_subscription(self, s):
        s.destroy()
        return True

    def destroy_timer(self, t):
        t.cancel()
        return True

    def count_subscribers(self, topic):
        return len(RT.subs.get(_topic(topic), []))

    def count_publishers(self, topic):
        return 1 if _topic(topic) in RT._pubs_seen else 0

    def get_topic_names_and_types(self):
        names = set(RT.subs) | set(RT._pubs_seen) | set(RT.internal)
        return sorted((n, []) for n in names)

    # ---- parameters
    def declare_parameter(self, name, value=None, descriptor=None, ignore_override=False):
        if name in self._params:
            from .exceptions import ParameterAlreadyDeclaredException
            raise ParameterAlreadyDeclaredException(name)
        if name in self._overrides and not ignore_override:
            value = self._overrides[name].value
        p = Parameter(name, value=value)
        self._params[name] = p
        return p

    def declare_parameters(self, namespace, parameters, ignore_override=False):
        out = []
        for item in parameters:
            name, value = item[0], (item[1] if len(item) > 1 else None)
            full = (namespace + "." + name) if namespace else name
            out.append(self.declare_parameter(full, value))
        return out

    def has_parameter(self, name):
        return name in self._params

    def get_parameter(self, name):
        if name not in self._params:
            from .exceptions import ParameterNotDeclaredException
            raise ParameterNotDeclaredException(name)
        return self._params[name]

    def get_parameter_or(self, name, alternative_value=None):
        return self._params.get(name, alternative_value)

    def get_parameters(self, names):
        return [self.get_parameter(n) for n in names]

    def set_parameters(self, parameter_list):
        results = []
        for p in parameter_list:
            ok = SetParametersResult(successful=True)
            for cb in self._param_callbacks:
                r = cb([p])
                if r is not None and not r.successful:
                    ok = r
                    break
            if ok.successful:
                self._params[p.name] = Parameter(p.name, value=p.value)
            results.append(ok)
        return results

    def add_on_set_parameters_callback(self, callback):
        self._param_callbacks.append(callback)

    def destroy_node(self):
        for s in self.subscriptions:
            s.destroy()
        for t in self.timers:
            t.cancel()
        if self in RT.nodes:
            RT.nodes.remove(self)
        return True
