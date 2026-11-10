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

#include "AttitudeMonitor.hpp"

using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

AttitudeMonitor::AttitudeMonitor() :
	ModuleParams(nullptr),
	// Low priority work queue: monitoring must never delay the estimator or the controllers.
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

AttitudeMonitor::~AttitudeMonitor()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool AttitudeMonitor::init()
{
	// Register the callback: from now on Run() is scheduled by new vehicle_attitude data.
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void AttitudeMonitor::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// Latest gyro rate (not a wake-up source, just read the newest sample)
	sensor_combined_s sensor_combined;

	if (_sensor_combined_sub.update(&sensor_combined)) {
		_gyro_norm = Vector3f(sensor_combined.gyro_rad).norm();
	}

	vehicle_attitude_s att;

	if (_vehicle_attitude_sub.update(&att)) {
		const Quatf q(att.q);
		const Eulerf euler(q);

		// Tilt: angle between body z axis and NED down axis.
		// R(2,2) is the z component of the body z axis expressed in the earth frame.
		const Dcmf R(q);
		const float tilt = acosf(math::constrain(R(2, 2), -1.f, 1.f));

		// Sample timing, based on the sample time of the data, not on when Run() executed.
		uint32_t dt_us = 0;

		if (_last_sample_time != 0 && att.timestamp_sample > _last_sample_time) {
			dt_us = static_cast<uint32_t>(att.timestamp_sample - _last_sample_time);

			if (dt_us > _dt_max_us) {
				_dt_max_us = dt_us;
			}
		}

		_last_sample_time = att.timestamp_sample;

		// Tilt warning with 2 degree hysteresis to avoid toggling at the threshold.
		const float warn = math::radians(_param_am_tilt_warn.get());
		const float hyst = math::radians(2.f);

		if (!_tilt_warning && tilt > warn) {
			_tilt_warning = true;
			PX4_WARN("tilt %.1f deg above limit %.1f deg", (double)math::degrees(tilt), (double)_param_am_tilt_warn.get());

		} else if (_tilt_warning && tilt < (warn - hyst)) {
			_tilt_warning = false;
			PX4_INFO("tilt back below limit");
		}

		attitude_monitor_s out{};
		out.timestamp_sample = att.timestamp_sample;
		out.roll = euler.phi();
		out.pitch = euler.theta();
		out.yaw = euler.psi();
		out.tilt = tilt;
		out.gyro_norm = _gyro_norm;
		out.dt_us = dt_us;
		out.dt_max_us = _dt_max_us;
		out.tilt_warning = _tilt_warning;
		out.timestamp = hrt_absolute_time();
		_attitude_monitor_pub.publish(out);

		_last = out;
	}

	perf_end(_loop_perf);
}

int AttitudeMonitor::task_spawn(int argc, char *argv[])
{
	AttitudeMonitor *instance = new AttitudeMonitor();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int AttitudeMonitor::print_status()
{
	PX4_INFO("roll %.1f  pitch %.1f  yaw %.1f  tilt %.1f deg",
		 (double)math::degrees(_last.roll), (double)math::degrees(_last.pitch),
		 (double)math::degrees(_last.yaw), (double)math::degrees(_last.tilt));
	PX4_INFO("gyro norm %.3f rad/s, dt %u us, dt max %u us, tilt warning %s",
		 (double)_last.gyro_norm, _last.dt_us, _last.dt_max_us, _last.tilt_warning ? "yes" : "no");
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int AttitudeMonitor::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AttitudeMonitor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Event-driven work queue module. It runs on every new vehicle_attitude sample,
reads the latest gyro data from sensor_combined and publishes Euler angles,
tilt, gyro rate and sample timing on the attitude_monitor topic.
A warning is printed when the tilt exceeds AM_TILT_WARN.

### Example
$ attitude_monitor start
$ listener attitude_monitor
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("attitude_monitor", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int attitude_monitor_main(int argc, char *argv[])
{
	return AttitudeMonitor::main(argc, argv);
}
