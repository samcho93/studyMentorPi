class Future:
    def __init__(self):
        self._done = False
        self._result = None
        self._exc = None
        self._cbs = []

    def done(self):
        return self._done

    def result(self):
        if self._exc:
            raise self._exc
        return self._result

    def exception(self):
        return self._exc

    def set_result(self, result):
        self._result = result
        self._done = True
        for cb in self._cbs:
            cb(self)

    def set_exception(self, exc):
        self._exc = exc
        self._done = True
        for cb in self._cbs:
            cb(self)

    def add_done_callback(self, cb):
        if self._done:
            cb(self)
        else:
            self._cbs.append(cb)

    def cancel(self):
        pass
