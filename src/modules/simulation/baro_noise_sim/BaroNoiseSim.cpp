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

#include "BaroNoiseSim.hpp"

#include <math.h>
#include <stdlib.h>

BaroNoiseSim::BaroNoiseSim() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	// Unique device id, different from sensor_baro_sim, so the sensors module
	// treats this as a separate barometer with its own calibration slot.
	_device_id.devid_s.bus_type = device::Device::DeviceBusType_SIMULATION;
	_device_id.devid_s.bus = 2;
	_device_id.devid_s.address = 0x05;
	_device_id.devid_s.devtype = DRV_BARO_DEVTYPE_BAROSIM;

	srand(4321);
}

BaroNoiseSim::~BaroNoiseSim()
{
	perf_free(_loop_perf);
	perf_free(_publish_perf);
	perf_free(_dropped_perf);
}

bool BaroNoiseSim::init()
{
	_rate_hz = math::constrain(_param_rate.get(), (int32_t)1, (int32_t)100);
	ScheduleOnInterval(1_s / _rate_hz);
	return true;
}

float BaroNoiseSim::gaussian()
{
	// Box-Muller transform, rand() is good enough for a simulation noise source
	const float u1 = ((float)rand() + 1.f) / ((float)RAND_MAX + 2.f);
	const float u2 = (float)rand() / (float)RAND_MAX;
	return sqrtf(-2.f * logf(u1)) * cosf(2.f * M_PI_F * u2);
}

void BaroNoiseSim::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();

		// Apply a new output rate without restarting the module
		const int32_t rate = math::constrain(_param_rate.get(), (int32_t)1, (int32_t)100);

		if (rate != _rate_hz) {
			_rate_hz = rate;
			ScheduleClear();
			ScheduleOnInterval(1_s / _rate_hz);
		}
	}

	vehicle_global_position_s gpos;

	if (_gpos_truth_sub.update(&gpos)) {

		const float dt = (_last_sample_time > 0) ?
				 math::constrain((gpos.timestamp - _last_sample_time) * 1e-6f, 0.f, 1.f) : 0.f;
		_last_sample_time = gpos.timestamp;

		// ISA troposphere model: true static pressure and temperature at this altitude
		const float lapse_rate = 0.0065f;       // K/m
		const float temperature_msl = 288.15f;  // K
		const float pressure_msl = 101325.f;    // Pa
		const float temperature_k = temperature_msl - lapse_rate * gpos.alt;
		const float pressure_true = pressure_msl * powf(temperature_k / temperature_msl, 5.2559f);

		// Error model: random walk bias (drift) + white noise
		_bias_pa += _param_drift.get() * dt;
		float pressure = pressure_true + _bias_pa + _param_noise.get() * gaussian();
		const float temperature_c = temperature_k - 273.15f;

		const Fault fault = static_cast<Fault>(_param_fault.get());
		bool publish = true;

		switch (fault) {
		case Fault::Stuck:
			if (_have_output) {
				pressure = _last_pressure; // frozen sensor
			}

			break;

		case Fault::Step:
			pressure += _param_step.get();
			break;

		case Fault::Dropout:
			publish = false;
			perf_count(_dropped_perf);
			break;

		case Fault::None:
		default:
			break;
		}

		if (publish) {
			sensor_baro_s report{};
			report.timestamp_sample = gpos.timestamp;
			report.device_id = _device_id.devid;
			report.pressure = pressure;
			report.temperature = temperature_c;
			report.error_count = 0;
			report.timestamp = hrt_absolute_time();
			_sensor_baro_pub.publish(report);
			perf_count(_publish_perf);

			_last_pressure = pressure;
			_last_temperature = temperature_c;
			_have_output = true;
		}
	}

	perf_end(_loop_perf);
}

int BaroNoiseSim::print_status()
{
	static const char *fault_names[] = {"none", "stuck", "step", "dropout"};
	const int32_t f = math::constrain(_param_fault.get(), (int32_t)0, (int32_t)3);

	PX4_INFO("device id %" PRIu32 ", instance %d, rate %" PRId32 " Hz", _device_id.devid,
		 _sensor_baro_pub.get_instance(), _rate_hz);
	PX4_INFO("noise %.2f Pa, drift %.3f Pa/s, bias now %.2f Pa, fault %s",
		 (double)_param_noise.get(), (double)_param_drift.get(), (double)_bias_pa, fault_names[f]);
	PX4_INFO("last pressure %.1f Pa, temperature %.1f C", (double)_last_pressure, (double)_last_temperature);
	perf_print_counter(_loop_perf);
	perf_print_counter(_publish_perf);
	perf_print_counter(_dropped_perf);
	return 0;
}

int BaroNoiseSim::task_spawn(int argc, char *argv[])
{
	BaroNoiseSim *instance = new BaroNoiseSim();

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

int BaroNoiseSim::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BaroNoiseSim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Simulated barometer driver for SIH with a configurable error model.
It converts the ground truth altitude to static pressure (ISA model), adds
white noise (BNS_NOISE) and bias drift (BNS_DRIFT), and can inject faults
(BNS_FAULT: 0 none, 1 stuck, 2 step offset BNS_STEP, 3 dropout).

### Example
$ sensor_baro_sim stop
$ baro_noise_sim start
$ param set BNS_FAULT 2
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("baro_noise_sim", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int baro_noise_sim_main(int argc, char *argv[])
{
	return BaroNoiseSim::main(argc, argv);
}
