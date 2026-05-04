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

#include "IgcWriter.hpp"

#include <px4_platform_common/log.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

bool IgcWriter::open(time_t utc_seconds)
{
	close();

	struct tm tm_utc {};
	gmtime_r(&utc_seconds, &tm_utc);

	snprintf(_path, sizeof(_path), "/fs/microsd/log/%04d-%02d-%02d_%02d%02d%02d.igc",
		 tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday,
		 tm_utc.tm_hour, tm_utc.tm_min, tm_utc.tm_sec);

	_fp = fopen(_path, "w");

	if (_fp == nullptr) {
		PX4_ERR("IGC open failed: %s", _path);
		return false;
	}

	write_header(utc_seconds);
	return true;
}

void IgcWriter::write_header(time_t utc_seconds)
{
	struct tm tm_utc {};
	gmtime_r(&utc_seconds, &tm_utc);

	fprintf(_fp, "AXXXPX4 PX4 paraglider vario\r\n");
	fprintf(_fp, "HFDTE%02d%02d%02d\r\n",
		tm_utc.tm_mday, tm_utc.tm_mon + 1, tm_utc.tm_year % 100);
	fprintf(_fp, "HFFXA050\r\n");
	fprintf(_fp, "HFPLTPILOTINCHARGE:\r\n");
	fprintf(_fp, "HFGTYGLIDERTYPE:Paraglider\r\n");
	fprintf(_fp, "HFGIDGLIDERID:\r\n");
	fprintf(_fp, "HFDTMGPSDATUM:WGS-1984\r\n");
	fprintf(_fp, "HFRFWFIRMWAREVERSION:PX4\r\n");
	fprintf(_fp, "HFRHWHARDWAREVERSION:PX4\r\n");
	fprintf(_fp, "HFFTYFRTYPE:PX4 para_vario\r\n");
	fprintf(_fp, "HFGPSRECEIVER:PX4 EKF2\r\n");
	fprintf(_fp, "HFPRSPRESSALTSENSOR:Baro\r\n");
	fflush(_fp);
}

bool IgcWriter::write_b_record(time_t utc_seconds, double lat_deg, double lon_deg,
			       int32_t pressure_alt_m, int32_t gps_alt_m, bool fix_valid)
{
	if (_fp == nullptr) {
		return false;
	}

	struct tm tm_utc {};

	gmtime_r(&utc_seconds, &tm_utc);

	const char ns = (lat_deg >= 0.0) ? 'N' : 'S';

	const char ew = (lon_deg >= 0.0) ? 'E' : 'W';

	const double abs_lat = fabs(lat_deg);

	const double abs_lon = fabs(lon_deg);

	const int lat_deg_i = (int)abs_lat;

	const int lat_min_thousandths = (int)((abs_lat - lat_deg_i) * 60000.0 + 0.5);

	const int lon_deg_i = (int)abs_lon;

	const int lon_min_thousandths = (int)((abs_lon - lon_deg_i) * 60000.0 + 0.5);

	// Clamp altitudes to IGC 5-digit field (with sign for negative pressure altitudes is non-standard;
	// here we clip to 0..99999).
	auto clip5 = [](int32_t v) -> int32_t {
		if (v < 0) { return 0; }

		if (v > 99999) { return 99999; }

		return v;
	};

	int n = fprintf(_fp,
			"B%02d%02d%02d%02d%05d%c%03d%05d%c%c%05d%05d\r\n",
			tm_utc.tm_hour, tm_utc.tm_min, tm_utc.tm_sec,
			lat_deg_i, lat_min_thousandths, ns,
			lon_deg_i, lon_min_thousandths, ew,
			fix_valid ? 'A' : 'V',
			(int)clip5(pressure_alt_m),
			(int)clip5(gps_alt_m));

	fflush(_fp);

	return n > 0;
}

void IgcWriter::close()
{
	if (_fp != nullptr) {
		fclose(_fp);
		_fp = nullptr;
	}
}
