# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_NRF_SYSCTL_LOCAL_DOMAIN 		nrf_sysctl_local_domain.c)
#zephyr_library_sources_ifdef(CONFIG_NRF_SYSCTL_SYSTEM_CONTROLLER 	nrf_sysctl_system_controller.c)
zephyr_library_sources_ifdef(CONFIG_NRF_SYSCTL_SYSTEM_CONTROLLER 	nrf_sysctl_local_domain.c)

