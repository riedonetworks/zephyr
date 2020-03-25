/*
 * Copyright (c) 2020 Riedo Networks Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_DRIVERS_FLASH_FLEXSPI_FLASH_W25Q_H_
#define ZEPHYR_DRIVERS_FLASH_FLEXSPI_FLASH_W25Q_H_

struct flash_priv {
#if defined(CONFIG_MULTITHREADING)
	struct k_sem write_lock;
#endif
};

#endif /* ZEPHYR_DRIVERS_FLASH_FLEXSPI_FLASH_W25Q_H_ */
