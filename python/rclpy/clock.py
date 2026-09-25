from mentorpi_sim.core import RT
from .time import Time


class ClockType:
    ROS_TIME = 1
    SYSTEM_TIME = 2
    STEADY_TIME = 3


class Clock:
    def __init__(self, clock_type=ClockType.ROS_TIME):
        self.clock_type = clock_type

    def now(self):
        return Time(nanoseconds=int(round(RT.t * 1e9)))
