px4_add_module(
	MODULE modules__attitude_monitor
	MAIN attitude_monitor
	SRCS
		AttitudeMonitor.cpp
		AttitudeMonitor.hpp
	DEPENDS
		px4_work_queue
	)
