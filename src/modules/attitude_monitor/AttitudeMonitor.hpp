/****************************************************************************
 *
 *   Copyright (c) 2026 Batuhan Kilicli. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *
 ****************************************************************************/

/**
 * @file AttitudeMonitor.hpp
 *
 * Event-driven work queue module.
 * Runs on every new vehicle_attitude sample, reads the latest gyro data
 * from sensor_combined, and publishes Euler angles, tilt, gyro rate and
 * sample timing on its own attitude_monitor topic.
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/attitude_monitor.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

using namespace time_literals;

class AttitudeMonitor : public ModuleBase<AttitudeMonitor>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	AttitudeMonitor();
	~AttitudeMonitor() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	// Publication
	uORB::Publication<attitude_monitor_s> _attitude_monitor_pub{ORB_ID(attitude_monitor)};

	// Event source: every new vehicle_attitude schedules Run() on the work queue.
	// Same idea as waking a FreeRTOS task from a queue instead of polling with osDelay.
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};

	// Plain subscription: read the latest value when needed, it does not wake the module.
	uORB::Subscription _sensor_combined_sub{ORB_ID(sensor_combined)};

	// Parameter changes are checked at most once per second.
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// Performance counters, visible with "perf" or "attitude_monitor status"
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::AM_TILT_WARN>) _param_am_tilt_warn
	)

	attitude_monitor_s _last{};
	hrt_abstime _last_sample_time{0};
	uint32_t _dt_max_us{0};
	float _gyro_norm{0.f};
	bool _tilt_warning{false};
};
