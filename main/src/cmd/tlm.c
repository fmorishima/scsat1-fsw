/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>
#include <csp/csp.h>
#include "sc_csp.h"
#include "reply.h"
#include "syshk.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tlm, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

/* Command size */
#define TLM_CMD_MIN_SIZE   (1U)
#define TLM_HISTORY_CMD_SIZE   (9U)

/* Command ID */
#define TLM_SYSHK_CMD (0U)
#define TLM_HISTORY_CMD (1U)

#define UNKOWN_COMMAND_ID (0xFF)

/* Command argument offset */
#define TLM_HISTORY_SEQ_OFFSET (1U)
#define TLM_HISTORY_CNT_OFFSET (5U)
#define TLM_HISTORY_INTVL_OFFSET (7U)

struct tlm_work_msg {
	struct k_work work;
	csp_packet_t *packet;
};

static struct tlm_work_msg tlm_work_msg;

static void csp_tlm_history_work(struct k_work *item)
{
	struct tlm_work_msg *msg = CONTAINER_OF(item, struct tlm_work_msg, work);
	csp_packet_t *packet = msg->packet;

	uint32_t req_seq_num = sys_le32_to_cpu(*(uint32_t *)&packet->data[TLM_HISTORY_SEQ_OFFSET]);
	uint16_t req_count = sys_le16_to_cpu(*(uint16_t *)&packet->data[TLM_HISTORY_CNT_OFFSET]);
	uint16_t send_intvl_ms = sys_le16_to_cpu(*(uint16_t *)&packet->data[TLM_HISTORY_INTVL_OFFSET]);

	LOG_INF("history teleme command received, seq: %d, count: %d, intvl: %d",
			req_seq_num, req_count, send_intvl_ms);
	send_syshk_history_to_ground(req_seq_num, req_count, send_intvl_ms);

	csp_buffer_free(packet);
}

static int csp_tlm_history_handler(csp_packet_t *packet)
{
	static bool is_init = true;
	if (is_init) {
		k_work_init(&tlm_work_msg.work, csp_tlm_history_work);
		is_init = false;
	}

	if (k_work_is_pending(&tlm_work_msg.work)) {
		LOG_ERR("history teleme command is currently executing, so rejected.");
		csp_buffer_free(packet);
		return -EBUSY;
	}

	if (packet->length != TLM_HISTORY_CMD_SIZE) {
		LOG_ERR("teleme history, invalide command size: %d, exepcted: %d",
				packet->length, TLM_HISTORY_CMD_SIZE);
		csp_buffer_free(packet);
		return -EINVAL;
	}

	tlm_work_msg.packet = packet;
	k_work_submit(&tlm_work_msg.work);

	return 0;
}

int csp_tlm_handler(csp_packet_t *packet)
{
	int ret = 0;
	uint8_t command_id;

	if (packet == NULL) {
		ret = -EINVAL;
		goto end;
	}

	if (packet->length < TLM_CMD_MIN_SIZE) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EINVAL;
		goto free;
	}

	command_id = packet->data[CSP_COMMAND_ID_OFFSET];

	switch (command_id) {
	case TLM_SYSHK_CMD:
		send_syshk_to_ground();
		csp_buffer_free(packet);
		break;
	case TLM_HISTORY_CMD:
		csp_tlm_history_handler(packet);
		break;
	default:
		LOG_ERR("Unkown command code: %d", command_id);
		ret = -EINVAL;
		break;
	}

free:
	if (ret < 0) {
		csp_send_std_reply(packet, UNKOWN_COMMAND_ID, ret);
		csp_buffer_free(packet);
	}

end:
	return ret;
}
