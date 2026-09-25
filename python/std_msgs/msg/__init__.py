from mentorpi_sim._msg import msg
from builtin_interfaces.msg import Time

String = msg("std_msgs", "msg", "String", data="")
Bool = msg("std_msgs", "msg", "Bool", data=False)
Int32 = msg("std_msgs", "msg", "Int32", data=0)
Int64 = msg("std_msgs", "msg", "Int64", data=0)
Float32 = msg("std_msgs", "msg", "Float32", data=0.0)
Float64 = msg("std_msgs", "msg", "Float64", data=0.0)
Empty = msg("std_msgs", "msg", "Empty")
Header = msg("std_msgs", "msg", "Header", stamp=Time, frame_id="")
ColorRGBA = msg("std_msgs", "msg", "ColorRGBA", r=0.0, g=0.0, b=0.0, a=0.0)
