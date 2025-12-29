/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#define TLM_BLOCK_SIZE 256
#define TLM_FLASH_SIZE (4 * 1024 * 1024) // 4MiB
#define TLM_START_ADDR 0x00000000
#define ERASE_SECTOR_SIZE 4096
#define MAX_TLM_STORE_NUM (TLM_FLASH_SIZE/TLM_BLOCK_SIZE)
#define AVAILABLE_HISTORY_COUNT (MAX_TLM_STORE_NUM - (ERASE_SECTOR_SIZE / TLM_BLOCK_SIZE))

int datafs_init(void);
int update_boot_count(const char *fname);
int data_nor_erase(uint8_t id);
bool is_flash_erased(const struct device *flash_dev, uint32_t addr, size_t len);
const struct device* stored_tlm_flash_dev();
