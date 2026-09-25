from enum import Enum


class ReliabilityPolicy(Enum):
    SYSTEM_DEFAULT = 0
    RELIABLE = 1
    BEST_EFFORT = 2


class HistoryPolicy(Enum):
    SYSTEM_DEFAULT = 0
    KEEP_LAST = 1
    KEEP_ALL = 2


class DurabilityPolicy(Enum):
    SYSTEM_DEFAULT = 0
    TRANSIENT_LOCAL = 1
    VOLATILE = 2


QoSReliabilityPolicy = ReliabilityPolicy
QoSHistoryPolicy = HistoryPolicy
QoSDurabilityPolicy = DurabilityPolicy


class QoSProfile:
    def __init__(self, *, depth=10, history=HistoryPolicy.KEEP_LAST,
                 reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, **kw):
        self.depth = depth
        self.history = history
        self.reliability = reliability
        self.durability = durability

    def __repr__(self):
        return "QoSProfile(depth=%d, reliability=%s, durability=%s)" % (
            self.depth, self.reliability.name, self.durability.name)


qos_profile_sensor_data = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
qos_profile_system_default = QoSProfile(depth=10)
qos_profile_services_default = QoSProfile(depth=10)
qos_profile_parameters = QoSProfile(depth=1000)
