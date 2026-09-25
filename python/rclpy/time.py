from builtin_interfaces.msg import Time as TimeMsg


class Time:
    def __init__(self, *, seconds=0, nanoseconds=0, clock_type=1):
        self.nanoseconds = int(seconds * 1e9) + int(nanoseconds)
        self.clock_type = clock_type

    def seconds_nanoseconds(self):
        return divmod(self.nanoseconds, 1000000000)

    def to_msg(self):
        s, ns = self.seconds_nanoseconds()
        return TimeMsg(sec=s, nanosec=ns)

    @classmethod
    def from_msg(cls, msg, clock_type=1):
        return cls(seconds=msg.sec, nanoseconds=msg.nanosec)

    def __sub__(self, other):
        from .duration import Duration
        if isinstance(other, Time):
            return Duration(nanoseconds=self.nanoseconds - other.nanoseconds)
        return Time(nanoseconds=self.nanoseconds - other.nanoseconds)

    def __add__(self, other):
        return Time(nanoseconds=self.nanoseconds + other.nanoseconds)

    def __lt__(self, o):
        return self.nanoseconds < o.nanoseconds

    def __le__(self, o):
        return self.nanoseconds <= o.nanoseconds

    def __gt__(self, o):
        return self.nanoseconds > o.nanoseconds

    def __ge__(self, o):
        return self.nanoseconds >= o.nanoseconds

    def __eq__(self, o):
        return isinstance(o, Time) and self.nanoseconds == o.nanoseconds

    def __hash__(self):
        return hash(self.nanoseconds)

    def __repr__(self):
        return "Time(nanoseconds=%d, clock_type=ROS_TIME)" % self.nanoseconds
