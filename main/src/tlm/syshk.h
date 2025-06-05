/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

void send_syshk_to_ground(void);
void send_syshk_history_to_ground(uint32_t req_seq_num, uint16_t req_count, uint16_t send_intvl_ms);
void start_send_syshk(void);
