from mentorpi_sim.core import RT


class LoggingSeverity:
    DEBUG, INFO, WARN, ERROR, FATAL = 10, 20, 30, 40, 50


class RcutilsLogger:
    def __init__(self, name):
        self.name = name
        self.level = LoggingSeverity.INFO

    def _log(self, lvl, tag, msg):
        if lvl >= self.level:
            print("[%s] [%.3f] [%s]: %s" % (tag, RT.t, self.name, msg))
        return True

    def debug(self, msg, **kw):
        return self._log(10, "DEBUG", msg)

    def info(self, msg, **kw):
        return self._log(20, "INFO", msg)

    def warn(self, msg, **kw):
        return self._log(30, "WARN", msg)

    warning = warn

    def error(self, msg, **kw):
        return self._log(40, "ERROR", msg)

    def fatal(self, msg, **kw):
        return self._log(50, "FATAL", msg)

    def set_level(self, level):
        self.level = level
        return True

    def get_child(self, name):
        return RcutilsLogger(self.name + "." + name)


def get_logger(name):
    return RcutilsLogger(name)
