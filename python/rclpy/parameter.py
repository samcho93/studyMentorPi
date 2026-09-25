from enum import Enum


def _list_of(value, kind):
    return (isinstance(value, (list, tuple)) and len(value) > 0
            and all(type(v) is kind for v in value))


class _ParameterValue:
    def __init__(self, value, type_):
        self.type = type_.value
        self.bool_value = value if isinstance(value, bool) else False
        self.integer_value = value if type(value) is int else 0
        self.double_value = float(value) if isinstance(value, float) else 0.0
        self.string_value = value if isinstance(value, str) else ""
        self.bool_array_value = list(value) if _list_of(value, bool) else []
        self.integer_array_value = list(value) if _list_of(value, int) else []
        self.double_array_value = list(value) if _list_of(value, float) else []
        self.string_array_value = list(value) if _list_of(value, str) else []


class Parameter:
    class Type(Enum):
        NOT_SET = 0
        BOOL = 1
        INTEGER = 2
        DOUBLE = 3
        STRING = 4
        BYTE_ARRAY = 5
        BOOL_ARRAY = 6
        INTEGER_ARRAY = 7
        DOUBLE_ARRAY = 8
        STRING_ARRAY = 9

        @classmethod
        def from_parameter_value(cls, v):
            if v is None:
                return cls.NOT_SET
            if isinstance(v, bool):
                return cls.BOOL
            if isinstance(v, int):
                return cls.INTEGER
            if isinstance(v, float):
                return cls.DOUBLE
            if isinstance(v, str):
                return cls.STRING
            if isinstance(v, (list, tuple)):
                if not v:
                    return cls.STRING_ARRAY
                return {bool: cls.BOOL_ARRAY, int: cls.INTEGER_ARRAY, float: cls.DOUBLE_ARRAY,
                        str: cls.STRING_ARRAY}.get(type(v[0]), cls.STRING_ARRAY)
            raise TypeError("unsupported parameter value %r" % (v,))

    def __init__(self, name, type_=None, value=None):
        if type_ is not None and not isinstance(type_, Parameter.Type) and value is None:
            value, type_ = type_, None      # Parameter("x", 1.0) convenience
        if type_ is None:
            type_ = Parameter.Type.from_parameter_value(value)
        if type_ == Parameter.Type.DOUBLE and type(value) is int:
            value = float(value)
        self._name = name
        self._type = type_
        self._value = value

    @property
    def name(self):
        return self._name

    @property
    def value(self):
        return self._value

    @property
    def type_(self):
        return self._type

    def get_parameter_value(self):
        return _ParameterValue(self._value, self._type)

    def __repr__(self):
        return "Parameter(name=%r, value=%r)" % (self._name, self._value)


class SetParametersResult:
    def __init__(self, successful=True, reason=""):
        self.successful = successful
        self.reason = reason

    def __repr__(self):
        return "SetParametersResult(successful=%s, reason=%r)" % (self.successful, self.reason)
