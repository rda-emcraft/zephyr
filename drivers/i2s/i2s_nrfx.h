/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Nordic Semiconductor nRF I2S
 */



#ifndef DRIVERS_I2S_I2S_NRFX_H_
#define DRIVERS_I2S_I2S_NRFX_H_

/** @brief I2S driver clock configuration structure. */
struct i2s_clk_settings_t {
	u32_t frequency;		/**< Configured frequency [Hz]. */
	nrf_i2s_ratio_t ratio[3];	/**< Content of CONFIG.RATIO register
					  * for given frequency. Every element
					  * of ratio[3] array corresponds to
					  * one of 3 possible sample width
					  * values
					  * (accordingly 8, 16 and 24 bit).
					  */
	nrf_i2s_mck_t divider[3];	/**< Content of CONFIG.MCKFREQ register
					  * for given frequency. Every element
					  * of divider[3] array corresponds to
					  * one of 3 possible sample width
					  * values
					  *(accordingly 8, 16 and 24 bit).
					  */
};

/**@brief I2S driver clock configfuration table. */

#define NRFX_I2S_AVAILABLE_CLOCK_SETTINGS		\
{							\
	{						\
		.frequency = 1000,			\
		.ratio = {				\
			NRF_I2S_RATIO_256X,		\
			NRF_I2S_RATIO_256X,		\
			NRF_I2S_RATIO_384X		\
		},					\
		.divider = {				\
			NRF_I2S_MCK_32MDIV125,		\
			NRF_I2S_MCK_32MDIV125,		\
			NRF_I2S_MCK_32MDIV63		\
		},					\
	},						\
	{						\
		.frequency = 2000,			\
		.ratio = {				\
			NRF_I2S_RATIO_128X,		\
			NRF_I2S_RATIO_128X,		\
			NRF_I2S_RATIO_384X		\
		},					\
	.	divider = {				\
			NRF_I2S_MCK_32MDIV125,		\
			NRF_I2S_MCK_32MDIV125,		\
			NRF_I2S_MCK_32MDIV42		\
		},					\
	},						\
	{						\
		.frequency = 4000,			\
		.ratio = {				\
			NRF_I2S_RATIO_64X,		\
			NRF_I2S_RATIO_64X,		\
			NRF_I2S_RATIO_192X		\
		},					\
		.divider = {				\
			NRF_I2S_MCK_32MDIV125,		\
			NRF_I2S_MCK_32MDIV125,		\
			NRF_I2S_MCK_32MDIV42		\
		},					\
	},						\
	{						\
		.frequency = 8000,			\
		.ratio = {				\
			NRF_I2S_RATIO_32X,		\
			NRF_I2S_RATIO_32X,		\
			NRF_I2S_RATIO_96X		\
		},					\
		.divider = {				\
			NRF_I2S_MCK_32MDIV125,		\
			NRF_I2S_MCK_32MDIV125,		\
			NRF_I2S_MCK_32MDIV42		\
		},					\
	},						\
	{						\
		.frequency = 12000,			\
		.ratio = {				\
			NRF_I2S_RATIO_64X,		\
			NRF_I2S_RATIO_64X,		\
			NRF_I2S_RATIO_192X		\
		},					\
		.divider = {				\
			NRF_I2S_MCK_32MDIV42,		\
			NRF_I2S_MCK_32MDIV42,		\
			NRF_I2S_MCK_32MDIV15		\
		},					\
	},						\
	{						\
		.frequency = 16000,			\
		.ratio = {				\
			NRF_I2S_RATIO_32X,		\
			NRF_I2S_RATIO_32X,		\
			NRF_I2S_RATIO_96X		\
	},						\
		.divider = {				\
			NRF_I2S_MCK_32MDIV63,		\
			NRF_I2S_MCK_32MDIV63,		\
			NRF_I2S_MCK_32MDIV21		\
		},					\
	},						\
	{						\
		.frequency = 20000,			\
		.ratio = {				\
			NRF_I2S_RATIO_96X,		\
			NRF_I2S_RATIO_96X,		\
			NRF_I2S_RATIO_96X		\
		},					\
		.divider = {				\
			NRF_I2S_MCK_32MDIV16,		\
			NRF_I2S_MCK_32MDIV16,		\
			NRF_I2S_MCK_32MDIV16		\
		},					\
	},						\
	{						\
		.frequency = 22050,			\
		.ratio = {				\
			NRF_I2S_RATIO_96X,		\
			NRF_I2S_RATIO_96X,		\
			NRF_I2S_RATIO_96X		\
		},					\
		.divider = {				\
			NRF_I2S_MCK_32MDIV15,		\
			NRF_I2S_MCK_32MDIV15,		\
			NRF_I2S_MCK_32MDIV15		\
		},					\
	},						\
	{						\
		.frequency = 24000,			\
		.ratio = {				\
			NRF_I2S_RATIO_32X,		\
			NRF_I2S_RATIO_32X,		\
			NRF_I2S_RATIO_96X		\
		},					\
		.divider = {				\
			NRF_I2S_MCK_32MDIV42,		\
			NRF_I2S_MCK_32MDIV42,		\
			NRF_I2S_MCK_32MDIV15		\
		},					\
	},						\
	{						\
		.frequency = 30000,			\
		.ratio = {				\
			NRF_I2S_RATIO_96X,		\
			NRF_I2S_RATIO_96X,		\
			NRF_I2S_RATIO_96X		\
		},					\
		.divider = {				\
			NRF_I2S_MCK_32MDIV11,		\
			NRF_I2S_MCK_32MDIV11,		\
			NRF_I2S_MCK_32MDIV11		\
		},					\
	},						\
	{						\
		.frequency = 32000,			\
		.ratio = {				\
			NRF_I2S_RATIO_32X,		\
			NRF_I2S_RATIO_32X,		\
			NRF_I2S_RATIO_48X		\
		},					\
		.divider = {				\
			NRF_I2S_MCK_32MDIV31,		\
			NRF_I2S_MCK_32MDIV31,		\
			NRF_I2S_MCK_32MDIV21		\
		},					\
	},						\
	{						\
		.frequency = 44100,			\
		.ratio = {				\
			NRF_I2S_RATIO_48X,		\
			NRF_I2S_RATIO_32X,		\
			NRF_I2S_RATIO_48X		\
		},					\
		.divider = {				\
			NRF_I2S_MCK_32MDIV15,		\
			NRF_I2S_MCK_32MDIV23,		\
			NRF_I2S_MCK_32MDIV15		\
		},					\
	},						\
	{						\
		.frequency = 48000,			\
		.ratio = {				\
			NRF_I2S_RATIO_64X,		\
			NRF_I2S_RATIO_64X,		\
			NRF_I2S_RATIO_48X		\
		},					\
		.divider = {				\
			NRF_I2S_MCK_32MDIV10,		\
			NRF_I2S_MCK_32MDIV10,		\
			NRF_I2S_MCK_32MDIV15		\
		},					\
	},						\
}


#endif /* DRIVERS_I2S_I2S_NRFX_H_ */
