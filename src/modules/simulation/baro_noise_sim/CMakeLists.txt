px4_add_module(
	MODULE modules__simulation__baro_noise_sim
	MAIN baro_noise_sim
	SRCS
		BaroNoiseSim.cpp
		BaroNoiseSim.hpp
	DEPENDS
		px4_work_queue
	)
