/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>
#include <nrfx.h>
#include <device.h>
#include <i2s.h>



#define I2S_DEV "I2S_0"


#define USE_TX
#define USE_RX
#define USE_GPIO_DBG

#ifdef USE_GPIO_DBG
#include <drivers/gpio.h>
#endif

//mono / stereo
#define NB_OF_SAMPLES			256
#define NB_OF_CHANNELS			2
#define SINGLE_SAMPLE_SIZE_BYTES	3
#define FRAME_CLOCK_FREQUENCY_HZ	44100

//#define TRANSFER_BLOCK_TIME_US		((BLOCK_SIZE_BYTES * 8 * 1000000) / FRAME_CLOCK_FREQUENCY_HZ)
#define TRANSFER_BLOCK_TIME_MS		(BLOCK_SIZE_BYTES * 1000 / (FRAME_CLOCK_FREQUENCY_HZ * NB_OF_CHANNELS * SINGLE_SAMPLE_SIZE_BYTES))
#if (SINGLE_SAMPLE_SIZE_BYTES > 3)
#error it wont work!
#else
#define BLOCK_SIZE_BYTES 			(NB_OF_SAMPLES * NB_OF_CHANNELS * (1 << (SINGLE_SAMPLE_SIZE_BYTES - 1)))
#endif

#ifdef USE_TX
K_MEM_SLAB_DEFINE(i2sBufferTx, BLOCK_SIZE_BYTES , CONFIG_NRFX_I2S_TX_BLOCK_COUNT, 4);

struct i2s_config i2sConfigTx = {
	.word_size = SINGLE_SAMPLE_SIZE_BYTES * 8,
	.channels = NB_OF_CHANNELS,
	.format = I2S_FMT_DATA_FORMAT_I2S,
	.options = 0,
	.frame_clk_freq = FRAME_CLOCK_FREQUENCY_HZ,
	.mem_slab = &i2sBufferTx,
	.block_size = BLOCK_SIZE_BYTES,
	.timeout = TRANSFER_BLOCK_TIME_MS
};
#endif //USE_TX

#ifdef USE_RX
K_MEM_SLAB_DEFINE(i2sBufferRx, BLOCK_SIZE_BYTES, CONFIG_NRFX_I2S_RX_BLOCK_COUNT, 4);

struct i2s_config i2sConfigRx = {
	.word_size = SINGLE_SAMPLE_SIZE_BYTES * 8,
	.channels = NB_OF_CHANNELS,
	.format = I2S_FMT_DATA_FORMAT_I2S,
	.options = 0,
	.frame_clk_freq = FRAME_CLOCK_FREQUENCY_HZ,
	.mem_slab = &i2sBufferRx,
	.block_size = BLOCK_SIZE_BYTES,
	.timeout = TRANSFER_BLOCK_TIME_MS
};
#endif //USE_RX

#define NOW	1

#define STOP_HERE(expression)	{printk("\r\nSTOPPING\r\n");while(1){expression};}
#define PREPARE_BUFFER(buf, siz, pattern)	{\
		static u16_t current_val;\
		u16_t *d = buf;\
		for (u32_t i = 0; i < siz; i++) { \
			d[i] = current_val++; \
		}}


#define SLEEP(cond, ms)	{if (cond) k_sleep(ms);}
#define START_TX(cond)		{if (cond) {if (i2s_trigger(dev, I2S_DIR_TX, I2S_TRIGGER_START)	) {printk("\r\nError while trigger for %u\r\n", cond); tx_started = false;} else {tx_started = true;}} }
#define STOP_TX(cond)		{if (cond) {if (i2s_trigger(dev, I2S_DIR_TX, I2S_TRIGGER_STOP)	) {printk("\r\nError while trigger for %u\r\n", cond); } else {tx_started = false;trg_pin_drv(1, 0);} } }
#define DRAIN_TX(cond)		{if (cond) {if (i2s_trigger(dev, I2S_DIR_TX, I2S_TRIGGER_DRAIN)	) {printk("\r\nError while trigger for %u\r\n", cond); } } }
#define DROP_TX(cond)		{if (cond) {if (i2s_trigger(dev, I2S_DIR_TX, I2S_TRIGGER_DROP)	) {printk("\r\nError while trigger for %u\r\n", cond); } } }
#define PREPARE_TX(cond)	{if (cond) {if (i2s_trigger(dev, I2S_DIR_TX, I2S_TRIGGER_PREPARE)){printk("\r\nError while trigger for %u\r\n", cond); }} }
#define START_RX(cond)		{if (cond) {if (i2s_trigger(dev, I2S_DIR_RX, I2S_TRIGGER_START)	) {printk("\r\nError while trigger for %u\r\n", cond); rx_started = false;} else {rx_started = true;} }}
#define STOP_RX(cond)		{if (cond) {if (i2s_trigger(dev, I2S_DIR_RX, I2S_TRIGGER_STOP)	) {printk("\r\nError while trigger for %u\r\n", cond); } else {rx_started = false;}} }
#define DRAIN_RX(cond)		{if (cond) {if (i2s_trigger(dev, I2S_DIR_RX, I2S_TRIGGER_DRAIN)	) {printk("\r\nError while trigger for %u\r\n", cond); } } }
#define DROP_RX(cond)		{if (cond) {if (i2s_trigger(dev, I2S_DIR_RX, I2S_TRIGGER_DROP)	) {printk("\r\nError while trigger for %u\r\n", cond); } } }
#define EXIT_LOOP(cond)		{if (cond) break;}


#ifdef USE_GPIO_DBG
struct trg_pin {
	struct device *dev[4];
	u32_t pin[4];
	char port[4][8];
} trg_pin;

struct trg_pin trg_pin = {
	.pin = {4, 3, 2, 26},
	.port = {"GPIO_0", "GPIO_0", "GPIO_0", "GPIO_0"},
};

struct trg_pin  trg_button = {
		.pin = {DT_ALIAS_SW0_GPIOS_PIN, DT_ALIAS_SW1_GPIOS_PIN, DT_ALIAS_SW2_GPIOS_PIN, DT_ALIAS_SW3_GPIOS_PIN},
		.port = {DT_ALIAS_SW0_GPIOS_CONTROLLER, DT_ALIAS_SW1_GPIOS_CONTROLLER, DT_ALIAS_SW2_GPIOS_CONTROLLER, DT_ALIAS_SW3_GPIOS_CONTROLLER},
};

bool trg_pins_init()
{
	int ret;
	for (u8_t i = 0; i < sizeof(trg_pin.pin) / sizeof(trg_pin.pin[0]); i++) {
	        trg_pin.dev[i] = device_get_binding(trg_pin.port[i]);
	        if (!trg_pin.dev[i]) {
	                printk("Cannot find %s!\n", trg_pin.port[i]);
	                return false;
	        }
	        /* Setup GPIO output */
	         ret = gpio_pin_configure(trg_pin.dev[i], trg_pin.pin[i], GPIO_DIR_OUT);
	         if (ret) {
	                 printk("Error configuring pin %d!\n", trg_pin.pin[i]);
	         }
	         ret = gpio_pin_write(trg_pin.dev[i], trg_pin.pin[i], 1);
		 if (ret) {
			 printk("Error set pin %d!\n", trg_pin.pin[i]);
		 }
	}
	return true;
}

bool trg_buttons_init()
{
	int ret;
	for (u8_t i = 0; i < sizeof(trg_button.pin) / sizeof(trg_button.pin[0]); i++) {
	        trg_button.dev[i] = device_get_binding(trg_button.port[i]);
	        if (!trg_button.dev[i]) {
	                printk("Cannot find %s!\n", trg_button.port[i]);
	                return false;
	        }
	        /* Setup GPIO output */
	         ret = gpio_pin_configure(trg_button.dev[i], trg_button.pin[i], GPIO_DIR_IN |  GPIO_PUD_PULL_UP );
	         if (ret) {
	                 printk("Error configuring button %d!\n", trg_button.pin[i]);
	         }

	}
	return true;
}

bool trg_button_pushed(u8_t i)
{
	bool ret;
	static u32_t last_val[sizeof(trg_button.pin)/sizeof(trg_button.pin[0])];
	u32_t val;
	gpio_pin_read(trg_button.dev[i], trg_button.pin[i], &val);
	ret = (val == 0 && val != last_val[i]) ? (true) : (false);
	last_val[i] = val;
	return ret;
}

void trg_pin_drv(u8_t i, u8_t val)
{
	if (gpio_pin_write(trg_pin.dev[i], trg_pin.pin[i], val)) {
		printk("\r\n[%s] Error driving pin no %u\r\n", __func__, i);
	}
}




#endif//#ifdef USE_GPIO_DBG
static u32_t tx_alloc_cnt;
static u32_t rx_alloc_cnt;

void main(void)
{
	struct device *dev;
	int ret = 0;

	dev = device_get_binding(I2S_DEV);
	if (dev == NULL) {
		while (1) { }
	}
#ifdef USE_GPIO_DBG
	if (!trg_pins_init()) {
		return;
	}
	if (!trg_buttons_init()) {
		return;
	}
#endif //#ifdef USE_GPIO_DBG
#ifdef USE_TX
	bool tx_started = false;
	void *my_tx_buf;
	ret = i2s_configure(dev, I2S_DIR_TX, &i2sConfigTx);
#endif
#ifdef USE_RX
	bool rx_started = false;
	void *my_rx_buf;
	size_t rcv_size = 0;
	ret = i2s_configure(dev, I2S_DIR_RX, &i2sConfigRx);
#endif
	if (ret != 0) {
		printk("\r\nerror configuring device, error code : %d", ret);
		while(1) {

		}
	}

	//testConfig = i2s_config_get(dev, I2S_DIR_TX);

	/*-------------------tests---------------------*/
	u16_t sample_indicator = 0;
	for (u32_t i = 0; ; sample_indicator += 0x1000, ++i) {
#ifdef USE_TX
		 my_tx_buf = NULL;
		 ret = k_mem_slab_alloc(i2sConfigTx.mem_slab, &my_tx_buf, (tx_started) ? (TRANSFER_BLOCK_TIME_MS) : (K_NO_WAIT));
		 if (ret == 0) {
#ifdef USE_GPIO_DBG
			 trg_pin_drv(0, 0);
#endif //USE_GPIO_DBG
			 tx_alloc_cnt ++;
			 //printk("\r\nTX LOG_MEM[%s]MEM ALLOC (%p %u)\r\n", __func__, my_tx_buf, tx_alloc_cnt);
			 PREPARE_BUFFER(my_tx_buf, 512, sample_indicator);
			 ret = i2s_write(dev, my_tx_buf, BLOCK_SIZE_BYTES);
			 if (ret != 0) {
				printk("w");
			 }
			 else {
#ifdef USE_GPIO_DBG
				trg_pin_drv(0, 1);
#endif //USE_GPIO_DBG
				//printk("\r\nwriting data (%u): %p %04X - %04X\r\n", alloc_cnt, d, d[0], d[511]);
			 }
		 }
		 else {
			 printk("a");
		 }

#endif


		 if (trg_button_pushed(0)) {
			 printk("\r\n----------------------------------------button start TX--------------------------------------------------------\r\n");
			 START_TX(1);
		 }
		 if (trg_button_pushed(1)) {
			 printk("\r\n----------------------------------------button start RX--------------------------------------------------------\r\n");
			 START_RX(1);
		 }
		 if (trg_button_pushed(2)) {
			 printk("\r\n----------------------------------------button stop TX--------------------------------------------------------\r\n");
			 STOP_TX(1);
		 }
		 if (trg_button_pushed(3)) {
			 printk("\r\n----------------------------------------button stop RX--------------------------------------------------------\r\n");
			 STOP_RX(1);
		 }
#ifdef USE_RX
		 if (rx_started) {

			 ret = i2s_read(dev, &my_rx_buf, &rcv_size);

			 if (ret != 0) {
				printk("r");
			 }
			 else {
#ifdef USE_GPIO_DBG
				 trg_pin_drv(1, 0);
#endif //USE_GPIO_DBG
				 //printk("RX LOG_MEM[%s]MEM FREE (%p)\r\n", __func__, my_rx_buf);
				 k_mem_slab_free(i2sConfigRx.mem_slab, &my_rx_buf);
				 rx_alloc_cnt --;
				 //printk("\r\nTX LOG_MEM1[%s(%u)]MEM free (%p)\r\n", __func__, rx_alloc_cnt, my_rx_buf);
#ifdef USE_GPIO_DBG
				 //trg_pin_drv(1, 1);
#endif //USE_GPIO_DBG
			 }

		 }
#endif
	}
	 k_sleep(2000);
	 printk("\r\n%s Finished\n", CONFIG_BOARD);
	 while (1) { }
}
