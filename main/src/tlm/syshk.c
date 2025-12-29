/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/zbus/zbus.h>
#include <csp/drivers/can_zephyr.h>
#include <csp/csp.h>
#include "data_nor.h"
#include "sc_csp.h"
#include "syshk.h"
#include "system.h"
#include "temp.h"
#include "cv.h"
#include "mgnm_mon.h"
#include "sunsens_mon.h"
#include "fram.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(syshk, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

K_THREAD_STACK_DEFINE(syshk_workq_stack, CONFIG_SCSAT1_MAIN_SEND_SYSHK_THREAD_STACK_SIZE);
struct k_work_q syshk_workq;

ZBUS_CHAN_DEFINE(system_chan, struct system_msg, NULL, NULL, ZBUS_OBSERVERS(syshk_sub),
		 ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(temp_chan, struct temp_msg, NULL, NULL, ZBUS_OBSERVERS(syshk_sub),
		 ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(cv_chan, struct cv_msg, NULL, NULL, ZBUS_OBSERVERS(syshk_sub), ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(mgnm_chan, struct mgnm_msg, NULL, NULL, ZBUS_OBSERVERS(syshk_sub),
		 ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(sunsens_chan, struct sunsens_msg, NULL, NULL, ZBUS_OBSERVERS(syshk_sub),
		 ZBUS_MSG_INIT(0));
ZBUS_SUBSCRIBER_DEFINE(syshk_sub, CONFIG_SCSAT1_MAIN_SYSHK_SUB_QUEUE_SIZE);

#define SYSHK_SYSTEM_BLOCK_SIZE  (33U)
#define SYSHK_TEMP_BLOCK_SIZE    (50U)
#define SYSHK_CV_BLOCK_SIZE      (105U)
#define SYSHK_MGNM_BLOCK_SIZE    (24U)
#define SYSHK_SUNSENS_BLOCK_SIZE (28U)

struct syshk_tlm {
	uint8_t telemetry_id;
	uint32_t seq_num;
	uint8_t system_block[SYSHK_SYSTEM_BLOCK_SIZE];
	uint8_t temp_block[SYSHK_TEMP_BLOCK_SIZE];
	uint8_t cv_block[SYSHK_CV_BLOCK_SIZE];
	uint8_t mgnm_block[SYSHK_MGNM_BLOCK_SIZE];
	uint8_t sunsens_block[SYSHK_SUNSENS_BLOCK_SIZE];
} __attribute__((__packed__));

static struct syshk_tlm syshk = {.telemetry_id = CSP_TLM_ID_SYSHK, .seq_num = 0};

static void copy_system_to_syshk(struct system_msg *msg)
{
	uint8_t *system_block = syshk.system_block;

	memcpy(system_block, &msg->wall_clock, sizeof(msg->wall_clock));
	system_block += sizeof(msg->wall_clock);
	memcpy(system_block, &msg->sysup_time, sizeof(msg->sysup_time));
	system_block += sizeof(msg->sysup_time);
	memcpy(system_block, &msg->sw_version, sizeof(msg->sw_version));
	system_block += sizeof(msg->sw_version);
	memcpy(system_block, &msg->boot_count, sizeof(msg->boot_count));
	system_block += sizeof(msg->boot_count);
	memcpy(system_block, &msg->power_status, sizeof(msg->power_status));
	system_block += sizeof(msg->power_status);
	memcpy(system_block, &msg->fpga_version, sizeof(msg->fpga_version));
	system_block += sizeof(msg->fpga_version);
	memcpy(system_block, &msg->fpga_config_bank, sizeof(msg->fpga_config_bank));
	system_block += sizeof(msg->fpga_config_bank);
	memcpy(system_block, &msg->fpga_fallback_state, sizeof(msg->fpga_fallback_state));
	system_block += sizeof(msg->fpga_fallback_state);
	memcpy(system_block, &msg->received_command_count, sizeof(msg->received_command_count));
	system_block += sizeof(msg->received_command_count);
	memcpy(system_block, &msg->last_csp_port, sizeof(msg->last_csp_port));
	system_block += sizeof(msg->last_csp_port);
	memcpy(system_block, &msg->last_command_id, sizeof(msg->last_command_id));
	system_block += sizeof(msg->last_command_id);
	memcpy(system_block, &msg->ecc_error_count_by_auto, sizeof(msg->ecc_error_count_by_auto));
	system_block += sizeof(msg->ecc_error_count_by_auto);
	memcpy(system_block, &msg->ecc_error_count_by_bus, sizeof(msg->ecc_error_count_by_bus));
	system_block += sizeof(msg->ecc_error_count_by_bus);
	memcpy(system_block, &msg->sem_error_count, sizeof(msg->sem_error_count));
	system_block += sizeof(msg->sem_error_count);
}

static void copy_temp_to_syshk(struct temp_msg *msg)
{
	int pos;
	uint8_t *temp_block = syshk.temp_block;

	for (pos = 0; pos < OBC_TEMP_POS_NUM; pos++) {
		memcpy(temp_block, &msg->obc[pos].status, sizeof(msg->obc[pos].status));
		temp_block += sizeof(msg->obc[pos].status);
		memcpy(temp_block, &msg->obc[pos].temp, sizeof(msg->obc[pos].temp));
		temp_block += sizeof(msg->obc[pos].temp);
	}

	memcpy(temp_block, &msg->xadc.status, sizeof(msg->xadc.status));
	temp_block += sizeof(msg->xadc.status);
	memcpy(temp_block, &msg->xadc.temp, sizeof(msg->xadc.temp));
	temp_block += sizeof(msg->xadc.temp);

	for (pos = 0; pos < IO_TEMP_POS_NUM; pos++) {
		memcpy(temp_block, &msg->io[pos].status, sizeof(msg->io[pos].status));
		temp_block += sizeof(msg->io[pos].status);
		memcpy(temp_block, &msg->io[pos].temp, sizeof(msg->io[pos].temp));
		temp_block += sizeof(msg->io[pos].temp);
	}
}

static void copy_cv_to_syshk(struct cv_msg *msg)
{
	int pos;
	uint8_t *cv_block = syshk.cv_block;

	for (pos = 0; pos < OBC_CV_POS_NUM; pos++) {
		memcpy(cv_block, &msg->obc[pos].status, sizeof(msg->obc[pos].status));
		cv_block += sizeof(msg->obc[pos].status);
		memcpy(cv_block, &msg->obc[pos].cv, sizeof(msg->obc[pos].cv));
		cv_block += sizeof(msg->obc[pos].cv);
	}

	for (pos = 0; pos < OBC_XADC_CV_POS_NUM; pos++) {
		memcpy(cv_block, &msg->xadc[pos].status, sizeof(msg->xadc[pos].status));
		cv_block += sizeof(msg->xadc[pos].status);
		memcpy(cv_block, &msg->xadc[pos].cv, sizeof(msg->xadc[pos].cv));
		cv_block += sizeof(msg->xadc[pos].cv);
	}

	for (pos = 0; pos < IO_CV_POS_NUM; pos++) {
		memcpy(cv_block, &msg->io[pos].status, sizeof(msg->io[pos].status));
		cv_block += sizeof(msg->io[pos].status);
		memcpy(cv_block, &msg->io[pos].cv, sizeof(msg->io[pos].cv));
		cv_block += sizeof(msg->io[pos].cv);
	}
}

static void copy_mgnm_to_syshk(struct mgnm_msg *msg)
{
	int pos;
	uint8_t *mgnm_block = syshk.mgnm_block;

	for (pos = 0; pos < MGNM_POS_NUM; pos++) {
		memcpy(mgnm_block, &msg->magnet[pos].status, sizeof(msg->magnet[pos].status));
		mgnm_block += sizeof(msg->magnet[pos].status);
		memcpy(mgnm_block, &msg->magnet[pos].data, sizeof(msg->magnet[pos].data));
		mgnm_block += sizeof(msg->magnet[pos].data);
	}

	for (pos = 0; pos < MGNM_POS_NUM; pos++) {
		memcpy(mgnm_block, &msg->temp[pos].status, sizeof(msg->temp[pos].status));
		mgnm_block += sizeof(msg->temp[pos].status);
		memcpy(mgnm_block, &msg->temp[pos].data, sizeof(msg->temp[pos].data));
		mgnm_block += sizeof(msg->temp[pos].data);
	}
}

static void copy_sunsens_to_syshk(struct sunsens_msg *msg)
{
	int pos;
	uint8_t *sunsens_block = syshk.sunsens_block;

	for (pos = 0; pos < SUNSENS_POS_NUM; pos++) {
		memcpy(sunsens_block, &msg->sun[pos].status, sizeof(msg->sun[pos].status));
		sunsens_block += sizeof(msg->sun[pos].status);
		memcpy(sunsens_block, &msg->sun[pos].data, sizeof(msg->sun[pos].data));
		sunsens_block += sizeof(msg->sun[pos].data);
	}

	for (pos = 0; pos < SUNSENS_POS_NUM; pos++) {
		memcpy(sunsens_block, &msg->temp[pos].status, sizeof(msg->temp[pos].status));
		sunsens_block += sizeof(msg->temp[pos].status);
		memcpy(sunsens_block, &msg->temp[pos].data, sizeof(msg->temp[pos].data));
		sunsens_block += sizeof(msg->temp[pos].data);
	}
}

static void syshk_sub_task(void *sub)
{
	const struct zbus_channel *chan;

	const struct zbus_observer *subscriber = sub;

	struct system_msg system_msg;
	struct temp_msg temp_msg;
	struct cv_msg cv_msg;
	struct mgnm_msg mgnm_msg;
	struct sunsens_msg sunsens_msg;

	LOG_INF("Start the system HK Subscribing thread");

	while (!zbus_sub_wait(subscriber, &chan, K_FOREVER)) {
		if (&system_chan == chan) {
			zbus_chan_read(chan, &system_msg,
					   K_MSEC(CONFIG_SCSAT1_MAIN_ZBUS_READ_TIMEOUT_MSEC));
			LOG_DBG("Subscribe SYSTEM msg %d byte", sizeof(system_msg));
			copy_system_to_syshk(&system_msg);
		} else if (&temp_chan == chan) {
			zbus_chan_read(chan, &temp_msg,
					   K_MSEC(CONFIG_SCSAT1_MAIN_ZBUS_READ_TIMEOUT_MSEC));
			LOG_DBG("Subscribe TEMP msg %d byte", sizeof(temp_msg));
			copy_temp_to_syshk(&temp_msg);
		} else if (&cv_chan == chan) {
			zbus_chan_read(chan, &cv_msg,
					   K_MSEC(CONFIG_SCSAT1_MAIN_ZBUS_READ_TIMEOUT_MSEC));
			LOG_DBG("Subscribe CV msg %d byte", sizeof(cv_msg));
			copy_cv_to_syshk(&cv_msg);
		} else if (&mgnm_chan == chan) {
			zbus_chan_read(chan, &mgnm_msg,
					   K_MSEC(CONFIG_SCSAT1_MAIN_ZBUS_READ_TIMEOUT_MSEC));
			LOG_DBG("Subscribe MGNM msg %d byte", sizeof(mgnm_msg));
			copy_mgnm_to_syshk(&mgnm_msg);
		} else if (&sunsens_chan == chan) {
			zbus_chan_read(chan, &sunsens_msg,
					   K_MSEC(CONFIG_SCSAT1_MAIN_ZBUS_READ_TIMEOUT_MSEC));
			LOG_DBG("Subscribe SUN msg %d byte", sizeof(sunsens_msg));
			copy_sunsens_to_syshk(&sunsens_msg);
		} else {
			LOG_ERR("Wrong channel %p!", chan);
			continue;
		}
	}
}

K_THREAD_DEFINE(syshk_sub_task_id, CONFIG_SCSAT1_MAIN_SUB_SYSHK_THREAD_STACK_SIZE, syshk_sub_task,
		&syshk_sub, NULL, NULL, CONFIG_SCSAT1_MAIN_SUB_SYSHK_THREAD_PRIORITY, 0, 0);

static void send_syshk_to_ground_impl(struct syshk_tlm *syshk_tlm)
{
	csp_conn_t *conn;
	csp_packet_t *packet;

	conn = csp_connect(CSP_PRIO_NORM, CSP_ID_GND, CSP_PORT_TLM,
			   CONFIG_SCSAT1_MAIN_CSP_CONN_TIMEOUT_MSEC, CSP_O_NONE);
	if (conn == NULL) {
		LOG_ERR("Failed to connect to GND");
		goto end;
	}

	packet = csp_buffer_get(CSP_BUFFER_SIZE);
	if (packet == NULL) {
		LOG_ERR("Failed to get CSP buffer");
		goto close;
	}

	memcpy(&packet->data, syshk_tlm, sizeof(struct syshk_tlm));
	packet->length = sizeof(struct syshk_tlm);

	csp_send(conn, packet);
	LOG_DBG("Send HK to GND %d byte, seq: %d", packet->length, syshk_tlm->seq_num);

	csp_buffer_free(packet);
close:
	csp_close(conn);

end:
}

void send_syshk_to_ground(void)
{
	send_syshk_to_ground_impl(&syshk);
}

static volatile bool cancel_syshk_history_flag = false;
void cancel_syshk_history(void)
{
	LOG_INF("current history cancel flag: %d", cancel_syshk_history_flag);
	cancel_syshk_history_flag = true;
}

enum history_req_err_code {
	HISTORY_REQ_ERR_NONE = 0,
	HISTORY_REQ_ERR_INVALID_INTERVAL,
	HISTORY_REQ_ERR_INVALID_SEQ_NUM,
	HISTORY_REQ_ERR_SEQ_NUM_OUT_OF_RANGE,
};

# define MIN_HISTORY_INTERVAL_MS 10
# define MAX_HISTORY_INTERVAL_MS 1000
int check_syshk_history_req_param(uint32_t start_seq_num, uint32_t end_seq_num, uint16_t send_intvl_ms)
{
	const uint32_t curr_seq_num = syshk.seq_num;

	if (start_seq_num > end_seq_num) {
		LOG_ERR("Invalid range, start: %u, end: %u", start_seq_num, end_seq_num);
		return HISTORY_REQ_ERR_INVALID_SEQ_NUM;
	}

	if (send_intvl_ms < MIN_HISTORY_INTERVAL_MS || send_intvl_ms > MAX_HISTORY_INTERVAL_MS) {
		LOG_ERR("Invalid interval: %u ms", send_intvl_ms);
		return HISTORY_REQ_ERR_INVALID_INTERVAL;
	}

	if (end_seq_num >= curr_seq_num){ // at least one behind from the latest
		LOG_ERR("Future seq requested. Req: %u, Curr: %u", end_seq_num, curr_seq_num);
		return HISTORY_REQ_ERR_INVALID_SEQ_NUM;
	}

	uint32_t oldest_seq_num = (curr_seq_num > AVAILABLE_HISTORY_COUNT) ?
				(curr_seq_num - AVAILABLE_HISTORY_COUNT): 0;

	if (start_seq_num < oldest_seq_num){
		LOG_ERR("Req seq too old. Req: %u, Oldest: %u", start_seq_num, oldest_seq_num);
		return HISTORY_REQ_ERR_SEQ_NUM_OUT_OF_RANGE;
	}

	return HISTORY_REQ_ERR_NONE;
}

void send_syshk_history_to_ground(uint32_t start_seq_num, uint32_t end_seq_num, uint16_t skip_count , uint16_t send_intvl_ms)
{
	if (check_syshk_history_req_param(start_seq_num, end_seq_num, send_intvl_ms) != HISTORY_REQ_ERR_NONE){
		return;
	}

	struct syshk_tlm syshk_tlm;
	const struct device *flash_dev = stored_tlm_flash_dev();

	if (!device_is_ready(flash_dev)) {
		LOG_ERR("Flash device %s is not ready", flash_dev->name);
		return;
	}

	cancel_syshk_history_flag = false;
	uint32_t read_pos = start_seq_num % MAX_TLM_STORE_NUM;

	LOG_INF("Start sending history tlm, start: %u, end: %u", start_seq_num, end_seq_num);

	for (int i = start_seq_num; i <= end_seq_num; i += skip_count + 1){
		uint32_t read_addr = read_pos * TLM_BLOCK_SIZE + TLM_START_ADDR;
		int ret = flash_read(flash_dev, read_addr, &syshk_tlm, sizeof(syshk_tlm));
		if (ret == 0){
			send_syshk_to_ground_impl(&syshk_tlm);
			k_sleep(K_MSEC(send_intvl_ms));
		}
		else{
			LOG_ERR("Failed to read history syshk data from flash at address 0x%08X: %d",
					read_addr, ret);
		}

		if (cancel_syshk_history_flag){
			LOG_INF("History retrieval cancelled at seq: %u", i);
			return;
		}
		read_pos = (read_pos + skip_count + 1) % MAX_TLM_STORE_NUM;
	}
}

static void store_syshk(struct syshk_tlm *syshk_tlm)
{
	const struct device* flash_dev = stored_tlm_flash_dev();
	int ret;
	uint32_t write_pos;
	uint32_t write_addr;

	if (!device_is_ready(flash_dev)) {
		LOG_ERR("Flash device %s is not ready", flash_dev->name);
		return;
	}

	write_pos = syshk_tlm->seq_num % MAX_TLM_STORE_NUM;
	write_addr = write_pos * TLM_BLOCK_SIZE + TLM_START_ADDR;

	if(write_addr % ERASE_SECTOR_SIZE == 0){
		LOG_INF("reached to erase unit address 0x%08X, seq_num: %d\n",
			write_addr, syshk_tlm->seq_num);
		ret = flash_erase(flash_dev, write_addr, ERASE_SECTOR_SIZE);
		if(ret != 0){
			LOG_ERR("Failed to erase flash sector at address 0x%08X: %d",
				write_addr, ret);
			return;
		}
	}

	if(!is_flash_erased(flash_dev, write_addr, TLM_BLOCK_SIZE)){
		uint32_t sector_addr = write_addr & ~(ERASE_SECTOR_SIZE - 1);
		LOG_INF("block is not erased, force erase at address 0x%08X, seq_num: %d\n",
				sector_addr, syshk_tlm->seq_num);
		ret = flash_erase(flash_dev, sector_addr, ERASE_SECTOR_SIZE);
		if(ret != 0){
			LOG_ERR("Failed to erase flash sector at address 0x%08X: %d",
				sector_addr, ret);
			return;
		}
	}

	ret = flash_write(flash_dev, write_addr, syshk_tlm, sizeof(*syshk_tlm));
	if(ret != 0){
		LOG_ERR("Failed to write syshk_tlm to flash at address 0x%08X: %d",
				write_addr, ret);
		return;
	}
}

static void handle_syshk(struct k_work *work)
{
	ARG_UNUSED(work);

	uint32_t saved_seq_num;
	if(sc_fram_get_tlm_seq_num(&saved_seq_num) >= 0){
		if (syshk.seq_num == 0) { // the first time sending syshk after boot
			syshk.seq_num = saved_seq_num;
		}
		else if (syshk.seq_num != saved_seq_num) {
			LOG_ERR("Curr seq number differs from the saved value %d != %d",
					syshk.seq_num, saved_seq_num);
			syshk.seq_num = saved_seq_num;
		}
	}
	else {
		LOG_ERR("Failed to get saved seq num from FRAM, curr: %d", syshk.seq_num);
	}

	syshk.seq_num++;

	store_syshk(&syshk);

	if (IS_ENABLED(CONFIG_SCSAT1_MAIN_AUTO_SYSHK_DOWNLINK) &&
		(k_uptime_get_32() / MSEC_PER_SEC) > CONFIG_SCSAT1_MAIN_SYSHK_INHIBIT_PERIOD_SEC){
		send_syshk_to_ground();
	}

	if(sc_fram_update_tlm_seq_num() < 0){
		LOG_ERR("Failed to update saved seq num, curr num : %d", syshk.seq_num);
	}
}

static K_WORK_DEFINE(syshk_work, handle_syshk);

static void syshk_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&syshk_workq, &syshk_work);
}

static K_TIMER_DEFINE(syshk_timer, syshk_handler, NULL);

void start_send_syshk(void)
{
	k_work_queue_start(&syshk_workq, syshk_workq_stack,
			   K_THREAD_STACK_SIZEOF(syshk_workq_stack),
			   CONFIG_SCSAT1_MAIN_SEND_SYSHK_THREAD_PRIORITY, NULL);
	k_thread_name_set(&syshk_workq.thread, "syshk_workq");

	k_timer_start(&syshk_timer, K_SECONDS(CONFIG_SCSAT1_MAIN_SYSHK_INTERVAL_SEC),
			  K_SECONDS(CONFIG_SCSAT1_MAIN_SYSHK_INTERVAL_SEC));
}
