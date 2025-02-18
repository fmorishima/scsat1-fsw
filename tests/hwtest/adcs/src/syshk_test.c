/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/kernel.h>
#include "common.h"
#include "pwrctrl.h"
#include "temp_test.h"
#include "cv_test.h"
#include "imu_test.h"
#include "gnss_test.h"
#include "rw_test.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(syshk_test);

extern struct k_event loop_event;

struct all_test_result {
	uint32_t loop_count;
	uint32_t err_cnt;
};

static void update_rw_idx(uint8_t *rw_idx)
{
	(*rw_idx)++;
	if (*rw_idx >= 3) {
		*rw_idx = 0;
	}
}

static int one_loop(enum rw_pos pos, uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;
	struct rw_count_data rw_data;
	struct adcs_temp_test_result temp_ret;
	struct adcs_cv_test_result cv_ret;
	struct imu_test_result imu_ret;
	struct gnss_test_result gnss_ret;

	LOG_INF("===[RW Count Test Start (total err: %d)]===", *err_cnt);
	rw_get_counts(&rw_data);

	send_syshk(RW, &rw_data, sizeof(rw_data));

	LOG_INF("===[Temp Test Start (total err: %d)]===", *err_cnt);
	ret = temp_test(&temp_ret, err_cnt, LOG_DISABLE);
	if (ret < 0) {
		all_ret = -1;
	}

	send_syshk(TEMP, &temp_ret, sizeof(temp_ret));

	LOG_INF("===[CV Test Start (total err: %d)]===", *err_cnt);
	ret = cv_test(&cv_ret, err_cnt, LOG_DISABLE);
	if (ret < 0) {
		all_ret = -1;
	}

	send_syshk(CURRENT_VOLTAGE, &cv_ret, sizeof(cv_ret));

	LOG_INF("===[IMU Test Start (total err: %d)]===", *err_cnt);
	ret = imu_test(&imu_ret, err_cnt, LOG_DISABLE);
	if (ret < 0) {
		all_ret = -1;
	}

	send_syshk(IMU, &imu_ret, sizeof(imu_ret));

	LOG_INF("===[GNSS Test Start (total err: %d)]===", *err_cnt);
	ret = gnss_test(&gnss_ret, err_cnt, LOG_DISABLE);
	if (ret < 0) {
		all_ret = -1;
	}

	send_syshk(GNSS, &gnss_ret, sizeof(gnss_ret));

	return all_ret;
}

static int verify_status(enum rw_pos pos, uint32_t sec, uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;

	for (int j = 0; j < 5; j++) {
		ret = one_loop(pos, err_cnt);
		if (ret < 0) {
			all_ret = -1;
		}
	}

	return all_ret;
}

static bool is_loop_stop(void)
{
	if (k_event_wait(&loop_event, LOOP_STOP_EVENT, false, K_NO_WAIT) != 0) {
		return true;
	}

	return false;
}

int syshk_test(int32_t loop_count, uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;
	enum rw_pos pos_list[] = {
		RW_POS_X,
		RW_POS_Y,
		RW_POS_Z,
	};
	uint8_t rw_idx = 0;
	struct all_test_result test_ret;

	if (loop_count < 0) {
		loop_count = INT32_MAX;
	}

	for (int i = 1; i <= loop_count; i++) {
		if (is_loop_stop()) {
			break;
		}

		ret = rw_start(pos_list[rw_idx], RW_HALF_POTENTION);
		if (ret < 0) {
			(*err_cnt)++;
			all_ret = -1;
		}

		ret = verify_status(pos_list[rw_idx], 5, err_cnt);
		if (ret < 0) {
			all_ret = -1;
		}

		ret = rw_change_speed(pos_list[rw_idx], 0x20);
		if (ret < 0) {
			(*err_cnt)++;
			all_ret = -1;
		}

		ret = verify_status(pos_list[rw_idx], 5, err_cnt);
		if (ret < 0) {
			all_ret = -1;
		}

		rw_stop(pos_list[rw_idx]);

		test_ret.loop_count = i;
		test_ret.err_cnt = *err_cnt;
		send_syshk(ALL_TEST_RESULT, &test_ret, sizeof(test_ret));

		update_rw_idx(&rw_idx);
	}

	return all_ret;
}
