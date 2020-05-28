/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

/** @file
 *  @brief Nordic IPC tester.
 */

#include <zephyr.h>
#include <errno.h>
#include <logging/log.h>



LOG_MODULE_REGISTER(sysctl);
static struct k_sem test_sem;
void main(void)
{
#if IS_ENABLED(CONFIG_NRF_SYSCTL_SYSTEM_CONTROLLER)
	LOG_INF("[System Controller]\n");
#elif IS_ENABLED(CONFIG_NRF_SYSCTL_LOCAL_DOMAIN)
	LOG_INF("[Local Domain]\n");
#endif

    /* Test initialization */
    nrf_sysctl_init();
    //k_sem_init(&test_sem, 0, 10);
    //k_sem_take(&test_sem, K_MSEC(1000));
    while(1)
    {
	//k_sleep(K_MSEC(1000));
	for (uint32_t i = 20000000; i > 0; i--) arch_nop();
	LOG_INF("taking semaphore");
	//k_sem_take(&test_sem, K_MSEC(1000));
    }
}
