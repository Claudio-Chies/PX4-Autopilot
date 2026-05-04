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

#pragma once

#include "IgcWriter.hpp"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/para_vario_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tune_control.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class ParaVario : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	ParaVario();
	~ParaVario() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	void update_parameters();
	void update_vario();
	void drive_buzzer(float vario_tek);
	void update_igc();

	uORB::Publication<para_vario_status_s> _status_pub{ORB_ID(para_vario_status)};
	uORB::Publication<tune_control_s>      _tune_pub{ORB_ID(tune_control)};

	uORB::Subscription         _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription         _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription         _airspeed_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription         _air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription         _status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	IgcWriter _igc;

	// latest copies
	vehicle_local_position_s  _local_pos{};
	vehicle_global_position_s _global_pos{};
	airspeed_validated_s      _airspeed{};
	vehicle_air_data_s        _air_data{};

	bool _local_pos_valid{false};
	bool _global_pos_valid{false};
	bool _airspeed_valid{false};
	bool _air_data_valid{false};

	bool _armed{false};
	hrt_abstime _last_igc_record{0};
	hrt_abstime _last_tune{0};
	uint8_t _state{0};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::PV_ENABLE>)     _param_enable,
		(ParamBool<px4::params::PV_USE_TEK>)    _param_use_tek,
		(ParamFloat<px4::params::PV_SINK_THR>)  _param_sink_thr,
		(ParamFloat<px4::params::PV_CLIMB_THR>) _param_climb_thr,
		(ParamInt<px4::params::PV_TONE_BASE>)   _param_tone_base,
		(ParamInt<px4::params::PV_TONE_GAIN>)   _param_tone_gain,
		(ParamInt<px4::params::PV_VOLUME>)      _param_volume,
		(ParamBool<px4::params::PV_IGC_ENABLE>) _param_igc_enable,
		(ParamInt<px4::params::PV_RATE_HZ>)     _param_rate_hz
	)
};
