/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>

int main(void)
{
	while(1) {
		printk("Hello world from %s\n", CONFIG_BOARD_TARGET);
		k_sleep(K_MSEC(2000));
	}


	return 0;
}
