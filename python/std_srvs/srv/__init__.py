from mentorpi_sim._msg import service

Trigger = service("std_srvs", "Trigger", {}, {"success": False, "message": ""})
SetBool = service("std_srvs", "SetBool", {"data": False}, {"success": False, "message": ""})
Empty = service("std_srvs", "Empty", {}, {})
