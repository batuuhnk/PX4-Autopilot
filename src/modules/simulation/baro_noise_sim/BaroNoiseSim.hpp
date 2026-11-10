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
 * @file BaroNoiseSim.hpp
 *
 * Simulated barometer driver with a configurable error model.
 * Converts the SIH ground truth altitude to static pressure with the ISA
 * model, adds white noise, bias drift and optional faults, and publishes
 * sensor_baro like a real driver. Used to study how EKF2 handles
 * barometer noise and failures.
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/perf/perf_counter.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/vehicle_global_position.h>

using namespace time_literals;

class BaroNoiseSim : public ModuleBase<BaroNoiseSim>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	BaroNoiseSim();
	~BaroNoiseSim() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

	enum class Fault : int32_t {
		None    = 0,
		Stuck   = 1, // output frozen at the last value
		Step    = 2, // constant offset of BNS_STEP_PA added
		Dropout = 3, // no samples published
	};

private:
	void Run() override;

	float gaussian(); // N(0,1) sample, Box-Muller

	// A real driver publishes on a new sensor_baro instance, so this one does too.
	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};

	uORB::Subscription _gpos_truth_sub{ORB_ID(vehicle_global_position_groundtruth)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _publish_perf{perf_alloc(PC_COUNT, MODULE_NAME": published")};
	perf_counter_t _dropped_perf{perf_alloc(PC_COUNT, MODULE_NAME": dropped (fault)")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BNS_NOISE>) _param_noise,   // white noise std (Pa)
		(ParamFloat<px4::params::BNS_DRIFT>) _param_drift,   // bias drift (Pa/s)
		(ParamInt<px4::params::BNS_FAULT>)   _param_fault,   // Fault enum
		(ParamFloat<px4::params::BNS_STEP>)  _param_step,    // offset for Fault::Step (Pa)
		(ParamInt<px4::params::BNS_RATE>)    _param_rate     // output rate (Hz)
	)

	device::Device::DeviceId _device_id{};

	hrt_abstime _last_sample_time{0};
	float _bias_pa{0.f};
	float _last_pressure{0.f};
	float _last_temperature{0.f};
	int32_t _rate_hz{0};
	bool _have_output{false};
};
