"""Minimal ROS 2 message base used by the Playground shim.

Every message class is built from a field table so that construction with keyword
arguments, attribute defaults, equality and repr behave like rclpy-generated classes.
"""
import copy


class Message:
    _fields = ()        # tuple of (name, default_factory)
    _type = "msg/Message"

    def __init__(self, **kwargs):
        for name, factory in self._fields:
            object.__setattr__(self, name, factory())
        for k, v in kwargs.items():
            if k not in self.get_fields_and_field_types():
                raise AssertionError("Invalid arguments passed to constructor: %s" % k)
            setattr(self, k, v)

    def __setattr__(self, name, value):
        if name not in dict(self._fields):
            raise AttributeError("'%s' object has no attribute '%s'" % (type(self).__name__, name))
        object.__setattr__(self, name, value)

    @classmethod
    def get_fields_and_field_types(cls):
        return {n: type(f()).__name__ for n, f in cls._fields}

    def __eq__(self, other):
        return type(self) is type(other) and all(
            getattr(self, n) == getattr(other, n) for n, _ in self._fields)

    def __repr__(self):
        body = ", ".join("%s=%r" % (n, getattr(self, n)) for n, _ in self._fields)
        return "%s(%s)" % (self._type.replace("/", "."), body)

    def __deepcopy__(self, memo):
        new = type(self).__new__(type(self))
        for n, _ in self._fields:
            object.__setattr__(new, n, copy.deepcopy(getattr(self, n), memo))
        return new


def msg(pkg, kind, name, **fields):
    """Create a message class. fields: name -> default value or zero-arg factory."""
    table = []
    for n, d in fields.items():
        if callable(d) and not isinstance(d, (int, float, str, bool)):
            table.append((n, d))
        else:
            table.append((n, (lambda v=d: copy.copy(v))))
    return type(name, (Message,), {"_fields": tuple(table), "_type": "%s/%s/%s" % (pkg, kind, name)})


def service(pkg, name, request, response):
    req = msg(pkg, "srv", name + "_Request", **request)
    res = msg(pkg, "srv", name + "_Response", **response)
    return type(name, (), {"Request": req, "Response": res, "_type": "%s/srv/%s" % (pkg, name)})


def action(pkg, name, goal, result, feedback):
    g = msg(pkg, "action", name + "_Goal", **goal)
    r = msg(pkg, "action", name + "_Result", **result)
    f = msg(pkg, "action", name + "_Feedback", **feedback)
    return type(name, (), {"Goal": g, "Result": r, "Feedback": f, "_type": "%s/action/%s" % (pkg, name)})
