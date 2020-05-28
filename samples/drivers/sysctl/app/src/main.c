/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

/** @file
 *  @brief Nordic IPC tester.
 */
#include <zephyr/types.h>
#include <zephyr.h>
#include <errno.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(sysctl);

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

void rcv_msg_callback(void)
{

}

void dispatcher(void)
{
#if IS_ENABLED(CONFIG_NRF_SYSCTL_SYSTEM_CONTROLLER)
	LOG_INF("[System Controller]\n");
#elif IS_ENABLED(CONFIG_NRF_SYSCTL_LOCAL_DOMAIN)
	LOG_INF("[Local Domain]\n");
#endif

    /* Test initialization */
    nrf_sysctl_init(rcv_msg_callback);

    while(1)
    {
	k_sleep(K_MSEC(100));
	//z_nrf_sysctl_send_request1();
	//LOG_INF("taking semaphore");
	//k_sem_take(&test_sem, K_MSEC(1000));
    }
}

K_THREAD_DEFINE(dispatcher_id, STACKSIZE, dispatcher, NULL, NULL, NULL,
		PRIORITY, 0, 0);
