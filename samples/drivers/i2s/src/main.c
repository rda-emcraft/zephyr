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



#define I2S_DEV "I2S_0"


#define USE_TX
//#define USE_RX

#define NB_OF_SAMPLES			256
#define NB_OF_CHANNELS			2
#define SINGLE_SAMPLE_SIZE_BYTES	3
#define FRAME_CLOCK_FREQUENCY_HZ	10000
#define WORD_SIZE_BITS			(SINGLE_SAMPLE_SIZE_BYTES * 8)
#define TRANSFER_BLOCK_TIME_US		\
	((NB_OF_SAMPLES * 1000000) / FRAME_CLOCK_FREQUENCY_HZ)
#define TRANSFER_TIMEOUT_MS		\
	((TRANSFER_BLOCK_TIME_US > 1000) ? \
	(1 + TRANSFER_BLOCK_TIME_US / 1000) : (1))
#if (SINGLE_SAMPLE_SIZE_BYTES == 1)
typedef u8_t i2s_buf_t;
#elif (SINGLE_SAMPLE_SIZE_BYTES == 2)
typedef u16_t i2s_buf_t;
#elif (SINGLE_SAMPLE_SIZE_BYTES == 3)
typedef u32_t i2s_buf_t;
#else
#error Not supported
#endif
#define BLOCK_SIZE_BYTES		\
	(NB_OF_SAMPLES * NB_OF_CHANNELS * (1 << (SINGLE_SAMPLE_SIZE_BYTES - 1)))


#ifdef USE_TX
K_MEM_SLAB_DEFINE(i2sBufferTx, BLOCK_SIZE_BYTES,
		  CONFIG_NRFX_I2S_TX_BLOCK_COUNT, 4);

struct i2s_config i2sConfigTx = {
	.word_size = WORD_SIZE_BITS,
	.channels = NB_OF_CHANNELS,
	.format = I2S_FMT_DATA_FORMAT_I2S,
	.options = 0,
	.frame_clk_freq = FRAME_CLOCK_FREQUENCY_HZ,
	.mem_slab = &i2sBufferTx,
	.block_size = BLOCK_SIZE_BYTES,
	.timeout = TRANSFER_TIMEOUT_MS
};

/*Macro for preparing test buffer to be sent*/
#define PREPARE_BUFFER(buf, siz)	{			\
		static i2s_buf_t current_val;			\
		i2s_buf_t *d = buf;				\
		for (u32_t i = 0; i < siz; i++) {		\
			d[i] = (i2s_buf_t)(current_val++);	\
		}						\
	}
#endif /*USE_TX*/

#ifdef USE_RX
K_MEM_SLAB_DEFINE(i2sBufferRx, BLOCK_SIZE_BYTES,
		  CONFIG_NRFX_I2S_RX_BLOCK_COUNT, 4);

struct i2s_config i2sConfigRx = {
	.word_size = WORD_SIZE_BITS,
	.channels = NB_OF_CHANNELS,
	.format = I2S_FMT_DATA_FORMAT_I2S,
	.options = 0,
	.frame_clk_freq = FRAME_CLOCK_FREQUENCY_HZ,
	.mem_slab = &i2sBufferRx,
	.block_size = BLOCK_SIZE_BYTES,
	.timeout = TRANSFER_TIMEOUT_MS
};
#endif /*USE_RX*/

void main(void)
{
	struct device *dev;
	int ret = -1;

	printk("[I2S]Starting example.\n\n[I2S]Configuration:\n");
	dev = device_get_binding(I2S_DEV);
	if (dev == NULL) {
		printk("[I2S]Driver not found\n");
		return;
	}
#ifdef USE_TX
	/*TX transfer in progress*/
	bool tx_running = false;
	/*result of k_mem_slab_alloc() for TX buffer*/
	int alloc_res;
	/*result of i2s_write() for TX direction*/
	int write_res;
	/*tx_buffer handler*/
	void *my_tx_buf = NULL;
#endif
#ifdef USE_RX
	/*RX transfer in progress*/
	bool rx_running = false;
	/*result of i2s_read() for RX direction*/
	int read_res;
	/*rx_buffer handler*/
	void *my_rx_buf = NULL;
	/*data size received by i2s_read()*/
	size_t rcv_size = 0;
#endif
	for (u32_t i = 0; ; ++i) {
		if (i == 0) {


#ifdef USE_TX
			i2sConfigTx.frame_clk_freq = FRAME_CLOCK_FREQUENCY_HZ;
			ret = i2s_configure(dev, I2S_DIR_TX, &i2sConfigTx);
			if (ret != 0) {
				printk("[I2S]TX:Configuration failed\n");
				return;
			}
			printk("[I2S]TX:ENABLED\n");
			printk("[I2S]TX Timeout:%u[ms]\n", i2sConfigTx.timeout);
			printk("[I2S]TX Word size:%u[bits]\n", i2sConfigTx.word_size);
#else
			printk("[I2S]TX:DISABLED\n");
#endif

#ifdef USE_RX
			i2sConfigRx.frame_clk_freq = FRAME_CLOCK_FREQUENCY_HZ;
			ret = i2s_configure(dev, I2S_DIR_RX, &i2sConfigRx);
			if (ret != 0) {
				printk("[I2S]RX:Configuration failed\n");
				return;
			}
			printk("[I2S]RX:ENABLED\n");
			printk("[I2S]RX Timeout:%u[ms]\n", i2sConfigRx.timeout);
			printk("[I2S]RX Word size:%u[bits]\n", i2sConfigRx.word_size);
#else
			printk("[I2S]RX:DISABLED\n");
#endif
		}
#ifdef USE_TX
		if (i == 10) {
			printk("trg[%u] ", i);
			if (i2s_trigger(dev,
					I2S_DIR_TX, I2S_TRIGGER_START) != 0) {
				/*error occured -
				 * refer to I2S API zephyr description
				 */
				printk("[I2S]i2s_trigger(TX) returned error\n");
				return;
			}
			printk("trg[end] ", i);
			tx_running = true;
			printk("[I2S]Starting TX\n");
		}
		/*
		 * reserve buffer for data to be transmitted.
		 * When transmission is stopped there is no reason to wait until
		 * allocation timeout
		 */
		alloc_res = k_mem_slab_alloc(i2sConfigTx.mem_slab, &my_tx_buf,
			    (tx_running) ? (TRANSFER_TIMEOUT_MS) : (K_NO_WAIT));
		if (alloc_res == 0) {
			printk("%u ", i);
			/*if valid operation fill the buffer by test samples*/
			PREPARE_BUFFER(my_tx_buf,
				       (NB_OF_SAMPLES * NB_OF_CHANNELS));
			/*and send it, or add to send queue if no transmission*/
			write_res = i2s_write(dev, my_tx_buf, BLOCK_SIZE_BYTES);
			if (write_res < 0) {
				/*error occured -
				 * refer to I2S API zephyr description
				 */
				printk("[I2S]i2s_write() returned error\n");
				return;
			}
		} else {
			/*
			 * can't allocate block from memory slab
			 * refer to Memory Slabs Zephyr description
			 */
		}
#endif /*USE_TX*/

#ifdef USE_RX
		if (i == 10) {
			if (i2s_trigger(dev,
					I2S_DIR_RX, I2S_TRIGGER_START) != 0) {
				/*error occured -
				 * refer to I2S API zephyr description
				 */
				printk("[I2S]i2s_trigger(RX) returned error\n");
				return;
			}
			printk("[I2S]Starting RX\n");
			rx_running = true;
		}
		if (rx_running) {
			/*
			 * read data from driver. The i2s_read() will
			 * gives handler to received data. For more details
			 * refer to I2S API zephyr description
			 */
			read_res = i2s_read(dev, &my_rx_buf, &rcv_size);
			if (read_res == 0) {
				/*we can use received data (e.g. print it)*/
				i2s_buf_t *rcv_data = (i2s_buf_t *)my_rx_buf;

				/*
				 * Beware of high FRAME_CLOCK_FREQUENCY_HZ
				 * values while printing received data -
				 * it may cause TX underrun errors
				 */
				/*printk("%u, %p: %X %X %X %X %X %X %X %X\n",
						 rcv_size, rcv_data,
						 rcv_data[0], rcv_data[1],
						 rcv_data[2], rcv_data[3],
						 rcv_data[4], rcv_data[5],
						 rcv_data[6], rcv_data[7]
							       );*/
				/*after use free allocated buffer*/
				k_mem_slab_free(i2sConfigRx.mem_slab,
						 &my_rx_buf);
			} else {
				/*
				 * error occured or no data to read
				 * refer to I2S API zephyr description
				 */
			}
		}
		if (i == 100) {
			i2sConfigTx.frame_clk_freq = 0;
			i2sConfigRx.frame_clk_freq = 0;
						while((i2s_read(dev, &my_rx_buf, &rcv_size) == 0)) {
				k_mem_slab_free(i2sConfigRx.mem_slab, &my_rx_buf);
			}
			while (i2s_trigger(dev,
					I2S_DIR_TX, I2S_TRIGGER_DRAIN) != 0) {


			}
			while (i2s_trigger(dev,
					I2S_DIR_RX, I2S_TRIGGER_DRAIN) != 0) {

				if (i2s_read(dev, &my_rx_buf, &rcv_size) == 0) {
					k_mem_slab_free(i2sConfigRx.mem_slab,
							 &my_rx_buf);
				}
			}
			while (i2s_configure(dev, I2S_DIR_TX, &i2sConfigTx) == -EIO) {

			}
			while  (i2s_configure(dev, I2S_DIR_RX, &i2sConfigRx) == -EIO) {
				if (i2s_read(dev, &my_rx_buf, &rcv_size) == 0) {
					k_mem_slab_free(i2sConfigRx.mem_slab,
							 &my_rx_buf);
				}
			}

			printk("\r\nstopped\r\n");
			k_sleep(5000);
			printk("zzzzzzzzzzzzzzzzzzzzzzzzzzzzz");
			i = 0xFFFFFFFF;
		}
#endif
	}
	while (1) {

	}
}
