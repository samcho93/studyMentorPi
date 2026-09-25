"""rcl_interfaces.msg subset."""
from rclpy.parameter import SetParametersResult  # noqa: F401
from mentorpi_sim._msg import msg

ParameterDescriptor = msg("rcl_interfaces", "msg", "ParameterDescriptor", name="", type=0, description="",
                          additional_constraints="", read_only=False, dynamic_typing=False)
