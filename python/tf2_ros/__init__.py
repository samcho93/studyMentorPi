"""tf2_ros shim: broadcasters publish on /tf and /tf_static; Buffer walks the frame tree."""
import copy

from geometry_msgs.msg import TransformStamped
from mentorpi_sim.core import RT
from tf_transformations import quaternion_multiply, quaternion_inverse


class TransformException(Exception):
    pass


class LookupException(TransformException):
    pass


class ConnectivityException(TransformException):
    pass


class ExtrapolationException(TransformException):
    pass


class _TFMessage:
    def __init__(self, transforms):
        self.transforms = transforms


class TransformBroadcaster:
    def __init__(self, node, qos=None):
        self.node = node

    def sendTransform(self, transform):
        ts = transform if isinstance(transform, list) else [transform]
        RT.publish("/tf", _TFMessage([copy.deepcopy(t) for t in ts]))


class StaticTransformBroadcaster(TransformBroadcaster):
    def sendTransform(self, transform):
        ts = transform if isinstance(transform, list) else [transform]
        RT.publish("/tf_static", _TFMessage([copy.deepcopy(t) for t in ts]))


def _rotate(q, v):
    p = [v[0], v[1], v[2], 0.0]
    r = quaternion_multiply(quaternion_multiply(q, p), quaternion_inverse(q))
    return r[:3]


def _compose(a, b):
    """a then b: T = a * b, each (t, q)."""
    ta, qa = a
    tb, qb = b
    rt = _rotate(qa, tb)
    return ([ta[0] + rt[0], ta[1] + rt[1], ta[2] + rt[2]], quaternion_multiply(qa, qb))


def _invert(a):
    t, q = a
    qi = quaternion_inverse(q)
    ti = _rotate(qi, t)
    return ([-ti[0], -ti[1], -ti[2]], qi)


class Buffer:
    def __init__(self, cache_time=None, node=None):
        pass

    def _edges(self):
        e = dict(RT.static_tf)
        e.update(RT.tf)
        return e

    def _chain(self, frame):
        """Transform root<-frame and the root name."""
        edges = self._edges()
        T = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
        seen = set()
        while frame in edges:
            if frame in seen:
                raise ConnectivityException("loop in tf tree at %s" % frame)
            seen.add(frame)
            parent, t, q = edges[frame]
            T = _compose((list(t), list(q)), T)
            frame = parent
        return frame, T

    def all_frames(self):
        e = self._edges()
        return sorted(set(e) | {v[0] for v in e.values()})

    def can_transform(self, target_frame, source_frame, time=None, timeout=None):
        try:
            self.lookup_transform(target_frame, source_frame, time)
            return True
        except TransformException:
            return False

    def lookup_transform(self, target_frame, source_frame, time=None, timeout=None):
        frames = self.all_frames()
        for f in (target_frame, source_frame):
            if f not in frames:
                raise LookupException('"%s" passed to lookupTransform argument does not exist.' % f)
        r1, Ts = self._chain(source_frame)
        r2, Tt = self._chain(target_frame)
        if r1 != r2:
            raise ConnectivityException("Could not find a connection between '%s' and '%s'"
                                        % (target_frame, source_frame))
        t, q = _compose(_invert(Tt), Ts)
        out = TransformStamped()
        out.header.frame_id = target_frame
        out.child_frame_id = source_frame
        out.header.stamp.sec = int(RT.t)
        out.header.stamp.nanosec = int((RT.t - int(RT.t)) * 1e9)
        out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = t
        (out.transform.rotation.x, out.transform.rotation.y,
         out.transform.rotation.z, out.transform.rotation.w) = q
        return out


class TransformListener:
    def __init__(self, buffer, node, spin_thread=False, **kw):
        self.buffer = buffer
