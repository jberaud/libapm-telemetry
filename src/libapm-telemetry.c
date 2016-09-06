/*
 *  Copyright (C) 2016 Parrot All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <libtelemetry.h>
#include <futils/futils.h>
#include <futils/timetools.h>
#define ULOG_TAG apmtelem
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);
#include <errno.h>

#include <AP_Module_Structures.h>

struct apm_export {
	/* data exported by ahrs at the main loop rate */
	struct tlm_producer *ahrs_producer;
	float body_quaternion[4];
	float gyro_bias[3];
	/* data exported by the inertial sensor module at the gyro rate */
	struct tlm_producer *imu_producer;
	float raw_gyro[3];
	float raw_accel[3];
	uint8_t fsync_flag;
};

static struct apm_export export;

void ap_hook_setup_start(uint64_t time_us)
{
	int ret;
	const struct tlm_producer_reg_entry ahrs_entries[] = {
		{
			.ptr = export.body_quaternion,
			.name = "body_quaternion",
			.type = TLM_TYPE_FLOAT32,
			.size = sizeof(float),
			.count = 4,
			.flags = 0,
		},
		{
			.ptr = export.gyro_bias,
			.name = "gyro_bias",
			.type = TLM_TYPE_FLOAT32,
			.size = sizeof(float),
			.count = 3,
			.flags = 0,
		},
	};

	const struct tlm_producer_reg_entry imu_entries[] = {
		{
			.ptr = export.raw_gyro,
			.name = "raw_gyro",
			.type = TLM_TYPE_FLOAT32,
			.size = sizeof(float),
			.count = 4,
			.flags = 0
		},
		{
			.ptr = &export.fsync_flag,
			.name = "fsync_flag",
			.type = TLM_TYPE_UINT8,
			.size = sizeof(uint8_t),
			.count = 1,
			.flags = 0,
		},
	};

	memset(&export, 0, sizeof(export));

	export.ahrs_producer = tlm_producer_new("ardupilot_ahrs",
			400, 2500);
	if (export.ahrs_producer == NULL) {
		ULOGE("failed to create ardupilot_ahrs producer");
		goto error;
	}

	ret = tlm_producer_reg_array(export.ahrs_producer, ahrs_entries,
		SIZEOF_ARRAY(ahrs_entries));
	if (ret < 0) {
		ULOGE("tlm_producer_reg_array error for ahrs: %s",
				strerror(-ret));
		goto error;
	}

	ret = tlm_producer_reg_complete(export.ahrs_producer);
	if (ret < 0) {
		ULOGE("tlm_producer_reg_complete error for ahrs: %s",
				strerror(-ret));
		goto error;
	}

	export.imu_producer = tlm_producer_new("ardupilot_imu", 1000, 1000);
	if (export.imu_producer == NULL) {
		ULOGE("failed to create ardupilot_imu producer");
		goto error;
	}

	ret = tlm_producer_reg_array(export.imu_producer, imu_entries,
		SIZEOF_ARRAY(imu_entries));
	if (ret < 0) {
		ULOGE("tlm_producer_reg_array error for imu: %s",
				strerror(-ret));
		goto error;
	}

	ret = tlm_producer_reg_complete(export.imu_producer);
	if (ret < 0) {
		ULOGE("tlm_producer_reg_complete error for imu: %s",
				strerror(-ret));
		goto error;
	}
	return;
error:
	if (export.ahrs_producer) {
		tlm_producer_destroy(export.ahrs_producer);
		export.ahrs_producer = NULL;
	}
	if (export.imu_producer) {
		tlm_producer_destroy(export.imu_producer);
		export.imu_producer = NULL;
	}
	return;
}

void ap_hook_setup_complete(uint64_t time_us)
{
	ULOGI("setup_complete called");
}

void ap_hook_AHRS_update(const struct AHRS_state *state)
{
	struct timespec ts;

	/* copy AHRS data to local structures */
	memcpy(export.body_quaternion, state->quat,
			sizeof(export.body_quaternion));
	/*
	 * waiting for gyro_bias to be exported by ardupilot
	 * memcpy(export.gyro_bias, state->gyro_bias, sizeof(export.gyro_bias));
	 */

	/* export the data in telemetry */
	clock_gettime(CLOCK_MONOTONIC, &ts);
	tlm_producer_put_sample(export.ahrs_producer, &ts);
}

void ap_hook_gyro_sample(const struct gyro_sample *state)
{
	struct timespec ts;

	/* copy gyro data to local structures */
	memcpy(export.raw_gyro, state->gyro, sizeof(export.raw_gyro));
	clock_gettime(CLOCK_MONOTONIC, &ts);

	/* export the data in telemetry */
	tlm_producer_put_sample(export.imu_producer, &ts);
}

void ap_hook_accel_sample(const struct accel_sample *state)
{
	/* copy accel data to local structures */
	memcpy(export.raw_accel, state->accel, sizeof(export.raw_accel));

	export.fsync_flag = state->fsync_set;

	/*
	 * HACK. We need gyro and fsync bit to be exported at once and
	 * the gyro sample hook is called in the same context and just
	 * after the accel hook, so it will export both data
	 */
}
