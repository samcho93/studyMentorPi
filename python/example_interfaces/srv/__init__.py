from mentorpi_sim._msg import service

AddTwoInts = service("example_interfaces", "AddTwoInts", {"a": 0, "b": 0}, {"sum": 0})
Trigger = service("example_interfaces", "Trigger", {}, {"success": False, "message": ""})
SetBool = service("example_interfaces", "SetBool", {"data": False}, {"success": False, "message": ""})
