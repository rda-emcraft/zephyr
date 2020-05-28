/*
 * Copyright (c) 2016 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SYSCTL_NRF_SYSCTL_H_
#define ZEPHYR_INCLUDE_DRIVERS_SYSCTL_NRF_SYSCTL_H_

#include <device.h>

/** @brief Enumerated type of messages between cores. */
typedef u32_t nrf_sysctl_msg_id_t;
typedef enum
{
    SYSTEM_CLOCK_SET_TIMEOUT = 1 /* Set system closk timeout to given value. */
} nrf_sysctl_msg_type_t;

/** @brief Test channel configuration structure.*/
typedef struct
{
    nrf_sysctl_msg_id_t	  id;
    nrf_sysctl_msg_type_t type;
    u8_t                  data_size; /* Size of message data field. */
    u64_t                 timestamp; /* Message data field. */
} nrf_sysctl_msg_t;

/** @brief sends request from local domain to system controller
 *
 * Valid when @ref CONFIG_CLOCK_CONTROL_NRF_CALIBRATION_DEBUG is set.
 *
 * @return Number of calibrations or -1 if feature is disabled.
 */
extern void z_nrf_sysctl_send_request1(void);

#endif /* ZEPHYR_INCLUDE_DRIVERS_SYSCTL_NRF_SYSCTL_H_ */
