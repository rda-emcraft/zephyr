/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Example of using I2S shim
 */

#include <zephyr.h>
#include <misc/printk.h>
#include <nrfx.h>
#include <device.h>
#include <i2s.h>
#include <clock_control.h>

#define LOG_LEVEL 4
#include <logging/log.h>
LOG_MODULE_REGISTER(audio_i2s);




#define I2S_DEV "I2S_0"


#define USE_TX
#define USE_RX

/* Number of samples to send over I2S in one buffer */
#define I2S_SAMPLES_NUM			512
/* Number of PCM channels */
#define I2S_CH_NUM			2
/* Sample size 1=8 bit, 2=16 bit, 3=24 bit */
#define I2S_SINGLE_SMPL_SIZE		2
/* I2S clock frequency */
#define I2S_FRAME_CLK_FREQ_HZ		1000

#define I2S_WORD_SIZE_BITS		(I2S_SINGLE_SMPL_SIZE * 8)

#if (I2S_SINGLE_SMPL_SIZE == 1)
typedef u8_t i2s_buf_t;
#elif (I2S_SINGLE_SMPL_SIZE == 2)
typedef u16_t i2s_buf_t;
#elif (I2S_SINGLE_SMPL_SIZE == 3)
typedef u32_t i2s_buf_t;
#else
#error Not supported
#endif
#define I2S_BLOCK_SIZE_BYTES		\
	(I2S_SAMPLES_NUM * I2S_CH_NUM * (1 << (I2S_SINGLE_SMPL_SIZE - 1)))

#ifdef USE_TX
K_MEM_SLAB_DEFINE(i2sBufferTx, I2S_BLOCK_SIZE_BYTES,
		  CONFIG_NRFX_I2S_TX_BLOCK_COUNT, 4);

struct i2s_config i2s_cfg_tx = {
	.word_size = I2S_WORD_SIZE_BITS,
	.channels = I2S_CH_NUM,
	.format = I2S_FMT_DATA_FORMAT_I2S,
	.options = 0,
	.frame_clk_freq = I2S_FRAME_CLK_FREQ_HZ,
	.mem_slab = &i2sBufferTx,
	.block_size = I2S_BLOCK_SIZE_BYTES,
	.timeout = K_NO_WAIT
};

/*Macro for preparing test buffer to be sent*/
void PREPARE_BUFFER(void *buf, u32_t siz)
{
		static i2s_buf_t current_val;
		//printk("\nB %x", current_val);
		i2s_buf_t *d = buf;
		for (u32_t i = 0; i < siz; i++) {
			d[i] = (i2s_buf_t)(current_val++);
		}
}
#endif /*USE_TX*/

#ifdef USE_RX
K_MEM_SLAB_DEFINE(i2sBufferRx, I2S_BLOCK_SIZE_BYTES,
		  CONFIG_NRFX_I2S_RX_BLOCK_COUNT, 4);

struct i2s_config i2s_cfg_rx = {
	.word_size = I2S_WORD_SIZE_BITS,
	.channels = I2S_CH_NUM,
	.format = I2S_FMT_DATA_FORMAT_I2S,
	.options = 0,
	.frame_clk_freq = I2S_FRAME_CLK_FREQ_HZ,
	.mem_slab = &i2sBufferRx,
	.block_size = I2S_BLOCK_SIZE_BYTES,
	.timeout = 1000 /* TODO: This has to be changed if this shall be blocking */
};
#endif /*USE_RX*/

static struct device *dev;
static struct device *clk;


void call_me_if_ready(struct device *dev, void *user_data)
{
	bool *rd = (bool*)user_data;
	*rd = true;
}




void main(void)
{
	volatile bool ready = false;
	struct clock_control_async_data clk_data = {
		.cb = call_me_if_ready,
		.user_data = (void*)&ready,
	};
	clk = device_get_binding("CLOCK_16M");
	clock_control_async_on(clk, NULL, &clk_data);
	while (!ready) {

	}
	u32_t clk_offs = 0x40000000;
	printk("\nHFCLKSTAT(LSb should be set if ext clk active): 0x%x", *((u32_t*)(clk_offs + 0x40c)));
	while(1) {

	}

}
