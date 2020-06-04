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

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

LOG_MODULE_REGISTER(sysctl);
static struct k_sem test_sem;
static struct k_sem ipc_kick;

int nrf_sysctl_init(struct k_sem *ipc_kick);
void z_nrf_sysctl_send_request1(void);
static volatile uint32_t sync = false;

void ipc(void)
{

	//__ASSERT_MSG_INFO("IPC thread %u",CONFIG_POLL);
	k_sem_init(&ipc_kick, 0, 1);
	nrf_sysctl_init(&ipc_kick);


	//k_tid_t thr = k_current_get();


}

void main_thread(void)
{
	__ASSERT_MSG_INFO("Main thread");
#if IS_ENABLED(CONFIG_NRF_SYSCTL_SYSTEM_CONTROLLER)
	LOG_INF("[System Controller]\n");
#elif IS_ENABLED(CONFIG_NRF_SYSCTL_LOCAL_DOMAIN)
	LOG_INF("[Local Domain]\n");
#endif

    /* Test initialization */
    sync = 300000;
    while (sync) {
	    sync --;

	    k_yield();
    }
    __ASSERT_MSG_INFO("synced", sync);
    k_sem_init(&test_sem, 0, 10);
    k_tid_t thr = k_current_get();
    //k_sem_take(&test_sem, K_MSEC(1000));
    while(1)
    {
	//k_sleep(K_MSEC(1000));
	for (uint32_t i = 20000000; i > 0; i--) arch_nop();
	__ASSERT_MSG_INFO("taking semaphore %p", thr);
	//k_sem_take(&test_sem, K_MSEC(1000));
	//k_sleep(K_MSEC(1000));
	k_sem_give(&ipc_kick);
	k_sleep(K_FOREVER);
    }
}





K_THREAD_DEFINE(ipc_id, STACKSIZE, ipc, NULL, NULL, NULL,
		1, 0, 0);

K_THREAD_DEFINE(main_thread_id, STACKSIZE, main_thread, NULL, NULL, NULL,
		PRIORITY, 0, 0);
