/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ParaVario.hpp"

#include <px4_platform_common/log.h>
#include <math.h>
#include <time.h>

static constexpr float G = 9.80665f;

ModuleBase::Descriptor ParaVario::desc{task_spawn, custom_command, print_usage};

ParaVario::ParaVario() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

ParaVario::~ParaVario()
{
	_igc.close();
	perf_free(_loop_perf);
}

bool ParaVario::init()
{
	const int rate = _param_rate_hz.get();
	const uint32_t interval_us = 1000000U / (rate > 0 ? rate : 50);
	ScheduleOnInterval(interval_us);
	return true;
}

void ParaVario::update_parameters()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s pu;
		_parameter_update_sub.copy(&pu);
		updateParams();

		const int rate = _param_rate_hz.get();
		const uint32_t interval_us = 1000000U / (rate > 0 ? rate : 50);
		ScheduleOnInterval(interval_us);
	}
}

void ParaVario::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_igc.close();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

	update_parameters();

	if (!_param_enable.get()) {
		perf_end(_loop_perf);
		return;
	}

	if (_local_pos_sub.update(&_local_pos))   { _local_pos_valid = true; }

	if (_global_pos_sub.update(&_global_pos)) { _global_pos_valid = true; }

	if (_airspeed_sub.update(&_airspeed))     { _airspeed_valid = true; }

	if (_air_data_sub.update(&_air_data))     { _air_data_valid = true; }

	vehicle_status_s vs;

	if (_status_sub.update(&vs)) {
		_armed = (vs.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	}

	update_vario();
	update_igc();

	perf_end(_loop_perf);
}

void ParaVario::update_vario()
{
	if (!_local_pos_valid || !_local_pos.v_z_valid) {
		return;
	}

	const float vario_raw = -_local_pos.vz; // NED -> climb positive

	// TEK term: (V * dV/dt) / g — only when airspeed is valid and finite.
	float vario_tek = vario_raw;
	bool used_airspeed = false;

	if (_param_use_tek.get() && _airspeed_valid) {
		const float V = _airspeed.true_airspeed_m_s;
		const float dV = _airspeed.airspeed_derivative_filtered;

		if (PX4_ISFINITE(V) && PX4_ISFINITE(dV)) {
			vario_tek = vario_raw + (V * dV) / G;
			used_airspeed = true;
		}
	}

	const float vx = _local_pos.v_xy_valid ? _local_pos.vx : 0.f;
	const float vy = _local_pos.v_xy_valid ? _local_pos.vy : 0.f;
	const float gs = sqrtf(vx * vx + vy * vy);

	float glide = 0.f;

	if (vario_raw < -0.1f) {
		glide = gs / (-vario_raw);

		if (glide > 99.f) { glide = 99.f; }
	}

	if (vario_tek <= _param_sink_thr.get()) {
		_state = para_vario_status_s::STATE_SINK;

	} else if (vario_tek >= _param_climb_thr.get()) {
		_state = para_vario_status_s::STATE_CLIMB;

	} else if (gs > 1.0f) {
		_state = para_vario_status_s::STATE_GLIDE;

	} else {
		_state = para_vario_status_s::STATE_IDLE;
	}

	drive_buzzer(vario_tek);

	para_vario_status_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.vario_raw = vario_raw;
	msg.vario_tek = vario_tek;
	msg.airspeed_tas = used_airspeed ? _airspeed.true_airspeed_m_s : NAN;
	msg.airspeed_deriv = used_airspeed ? _airspeed.airspeed_derivative_filtered : 0.f;
	msg.ground_speed = gs;
	msg.glide_ratio = glide;
	msg.baro_alt = _air_data_valid ? _air_data.baro_alt_meter : NAN;
	msg.state = _state;
	msg.airspeed_valid = used_airspeed;
	msg.igc_logging = _igc.is_open();
	_status_pub.publish(msg);
}

void ParaVario::drive_buzzer(float vario_tek)
{
	const hrt_abstime now = hrt_absolute_time();

	tune_control_s tc{};
	tc.timestamp = now;
	tc.tune_id = 0; // CUSTOM (frequency/duration/silence used)
	tc.tune_override = true;
	tc.volume = (uint8_t)_param_volume.get();

	if (vario_tek >= _param_climb_thr.get()) {
		// Climb beep: pitch + duty cycle scale with climb rate. Faster climb -> shorter period.
		const float climb = vario_tek;
		const int freq = _param_tone_base.get() + (int)(_param_tone_gain.get() * climb);
		const float period_s = fmaxf(0.15f, 1.0f / fmaxf(0.5f, climb)); // ~0.15..2 s
		const uint32_t period_us = (uint32_t)(period_s * 1e6f);
		const uint32_t beep_us = period_us / 2;
		const uint32_t silence_us = period_us - beep_us;

		// rate-limit publication to once per period
		if ((now - _last_tune) < period_us) {
			return;
		}

		tc.frequency = (uint16_t)((freq < 100) ? 100 : (freq > 5000 ? 5000 : freq));
		tc.duration = beep_us;
		tc.silence = silence_us;
		_tune_pub.publish(tc);
		_last_tune = now;

	} else if (vario_tek <= _param_sink_thr.get()) {
		// Sink alarm: low continuous growl, refresh every 500 ms
		if ((now - _last_tune) < 500_ms) {
			return;
		}

		tc.frequency = 200;
		tc.duration = 500000;
		tc.silence = 0;
		_tune_pub.publish(tc);
		_last_tune = now;
	}
}

void ParaVario::update_igc()
{
	if (!_param_igc_enable.get()) {
		if (_igc.is_open()) { _igc.close(); }

		return;
	}

	if (!_armed) {
		if (_igc.is_open()) {
			_igc.close();
		}

		return;
	}

	if (!_global_pos_valid || !_global_pos.lat_lon_valid) {
		return;
	}

	const time_t utc = time(nullptr);

	if (!_igc.is_open()) {
		if (!_igc.open(utc)) {
			return;
		}

		_last_igc_record = 0;
	}

	const hrt_abstime now = hrt_absolute_time();

	// 1 Hz cadence
	if ((now - _last_igc_record) < 1_s) {
		return;
	}

	const int32_t pressure_alt = _air_data_valid ? (int32_t)lroundf(_air_data.baro_alt_meter) : 0;
	const int32_t gps_alt = _global_pos.alt_valid ? (int32_t)lroundf(_global_pos.alt) : 0;

	_igc.write_b_record(utc, _global_pos.lat, _global_pos.lon,
			    pressure_alt, gps_alt, _global_pos.lat_lon_valid && _global_pos.alt_valid);

	_last_igc_record = now;
}

int ParaVario::task_spawn(int argc, char *argv[])
{
	ParaVario *instance = new ParaVario();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int ParaVario::print_status()
{
	PX4_INFO("armed: %s, igc: %s (%s), state: %u",
		 _armed ? "yes" : "no",
		 _igc.is_open() ? "writing" : "off",
		 _igc.is_open() ? _igc.path() : "-",
		 _state);
	perf_print_counter(_loop_perf);
	return 0;
}

int ParaVario::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ParaVario::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Paraglider variometer.

Computes total-energy compensated vertical speed from EKF2 vz and airspeed,
drives the buzzer via tune_control with climb-rate-scaled tones, publishes
para_vario_status, and optionally writes an IGC track log to /fs/microsd/log/
while armed.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("para_vario", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int para_vario_main(int argc, char *argv[])
{
	return ModuleBase::main(ParaVario::desc, argc, argv);
}
