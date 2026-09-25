from builtin_interfaces.msg import Duration as DurationMsg


class Duration:
    def __init__(self, *, seconds=0, nanoseconds=0):
        self.nanoseconds = int(seconds * 1e9) + int(nanoseconds)

    def to_msg(self):
        s, ns = divmod(self.nanoseconds, 1000000000)
        return DurationMsg(sec=s, nanosec=ns)

    def __lt__(self, o):
        return self.nanoseconds < o.nanoseconds

    def __gt__(self, o):
        return self.nanoseconds > o.nanoseconds

    def __le__(self, o):
        return self.nanoseconds <= o.nanoseconds

    def __ge__(self, o):
        return self.nanoseconds >= o.nanoseconds

    def __repr__(self):
        return "Duration(nanoseconds=%d)" % self.nanoseconds
