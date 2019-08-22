/*
 * nrfx_i2s.c
 *
 *  Created on: Nov 7, 2018
 *      Author: barteks
 */


#include <string.h>
#include <stdlib.h>
#include <dma.h>
#include <i2s.h>
#include <soc.h>
#include <nrfx.h>
#include <nrfx_i2s.h>
#include <stdint.h>
/*test and logging*/
#define USE_PRINTK_FOR_LOG_MESSAGES
#ifndef USE_PRINTK_FOR_LOG_MESSAGES
#define LOG_DOMAIN "nrfx_i2s"
#define LOG_LEVEL _LOG_LEVEL_DBG
#include <logging/log.h>
LOG_MODULE_REGISTER(nrfx_i2s);
#else
void f(u8_t *x, ...) {}
#define LOG_ERR	/*printk("\r\n--");*/printk
#define LOG_INF1 f//printk
#define LOG_MEM	f//printk
#endif

#define NRFX_I2S_REPORT_ERROR(err_code)

/** @brief I2S driver clock configuration structure. */
typedef struct i2s_clk_settings_t {
	u32_t frequency;                ///< Configured frequency [Hz].
	nrf_i2s_ratio_t ratio[3];       /**< Content of CONFIG.RATIO register
	                                 * for given frequency. Every element of
	                                 * ratio[3] array corresponds to
	                                 * one of 3 possible sample width values
	                                 *(accordingly 8, 16 and 24 bit). */
	nrf_i2s_mck_t divider[3];       /**< Content of CONFIG.MCKFREQ register
	                                 * for given frequency. Every element of
	                                 * divider[3] array corresponds to one
	                                 * of 3 possible sample width values
	                                 *(accordingly 8, 16 and 24 bit). */
} i2s_clk_settings_t;

/**@brief I2S driver clock configfuration table. */
#define NRFX_I2S_AVAILABLE_CLOCK_SETTINGS								   \
	{												   \
		{											   \
			.frequency = 1000,								   \
			.ratio = { NRF_I2S_RATIO_256X, NRF_I2S_RATIO_256X, NRF_I2S_RATIO_384X },	   \
			.divider = { NRF_I2S_MCK_32MDIV125, NRF_I2S_MCK_32MDIV125, NRF_I2S_MCK_32MDIV63 }, \
		},											   \
		{											   \
			.frequency = 2000,								   \
			.ratio = { NRF_I2S_RATIO_128X, NRF_I2S_RATIO_128X, NRF_I2S_RATIO_384X },	   \
			.divider = { NRF_I2S_MCK_32MDIV125, NRF_I2S_MCK_32MDIV125, NRF_I2S_MCK_32MDIV42 }, \
		},											   \
		{											   \
			.frequency = 4000,								   \
			.ratio = { NRF_I2S_RATIO_64X, NRF_I2S_RATIO_64X, NRF_I2S_RATIO_192X },		   \
			.divider = { NRF_I2S_MCK_32MDIV125, NRF_I2S_MCK_32MDIV125, NRF_I2S_MCK_32MDIV42 }, \
		},											   \
		{											   \
			.frequency = 8000,								   \
			.ratio = { NRF_I2S_RATIO_32X, NRF_I2S_RATIO_32X, NRF_I2S_RATIO_96X },		   \
			.divider = { NRF_I2S_MCK_32MDIV125, NRF_I2S_MCK_32MDIV125, NRF_I2S_MCK_32MDIV42 }, \
		},											   \
		{											   \
			.frequency = 10000,								   \
			.ratio = { NRF_I2S_RATIO_96X, NRF_I2S_RATIO_96X, NRF_I2S_RATIO_192X },		   \
			.divider = { NRF_I2S_MCK_32MDIV32, NRF_I2S_MCK_32MDIV32, NRF_I2S_MCK_32MDIV16 },   \
		},											   \
		{											   \
			.frequency = 11025,								   \
			.ratio = { NRF_I2S_RATIO_96X, NRF_I2S_RATIO_96X, NRF_I2S_RATIO_192X },		   \
			.divider = { NRF_I2S_MCK_32MDIV30, NRF_I2S_MCK_32MDIV30, NRF_I2S_MCK_32MDIV15 },   \
		},											   \
		{											   \
			.frequency = 12000,								   \
			.ratio = { NRF_I2S_RATIO_64X, NRF_I2S_RATIO_64X, NRF_I2S_RATIO_192X },		   \
			.divider = { NRF_I2S_MCK_32MDIV42, NRF_I2S_MCK_32MDIV42, NRF_I2S_MCK_32MDIV15 },   \
		},											   \
		{											   \
			.frequency = 16000,								   \
			.ratio = { NRF_I2S_RATIO_32X, NRF_I2S_RATIO_32X, NRF_I2S_RATIO_96X },		   \
			.divider = { NRF_I2S_MCK_32MDIV63, NRF_I2S_MCK_32MDIV63, NRF_I2S_MCK_32MDIV21 },   \
		},											   \
		{											   \
			.frequency = 20000,								   \
			.ratio = { NRF_I2S_RATIO_96X, NRF_I2S_RATIO_96X, NRF_I2S_RATIO_96X },		   \
			.divider = { NRF_I2S_MCK_32MDIV16, NRF_I2S_MCK_32MDIV16, NRF_I2S_MCK_32MDIV16 },   \
		},											   \
		{											   \
			.frequency = 22050,								   \
			.ratio = { NRF_I2S_RATIO_64X, NRF_I2S_RATIO_64X, NRF_I2S_RATIO_96X },		   \
			.divider = { NRF_I2S_MCK_32MDIV23, NRF_I2S_MCK_32MDIV23, NRF_I2S_MCK_32MDIV15 },   \
		},											   \
		{											   \
			.frequency = 24000,								   \
			.ratio = { NRF_I2S_RATIO_32X, NRF_I2S_RATIO_32X, NRF_I2S_RATIO_96X },		   \
			.divider = { NRF_I2S_MCK_32MDIV42, NRF_I2S_MCK_32MDIV42, NRF_I2S_MCK_32MDIV15 },   \
		},											   \
		{											   \
			.frequency = 30000,								   \
			.ratio = { NRF_I2S_RATIO_96X, NRF_I2S_RATIO_96X, NRF_I2S_RATIO_96X },		   \
			.divider = { NRF_I2S_MCK_32MDIV11, NRF_I2S_MCK_32MDIV11, NRF_I2S_MCK_32MDIV11 },   \
		},											   \
		{											   \
			.frequency = 32000,								   \
			.ratio = { NRF_I2S_RATIO_32X, NRF_I2S_RATIO_32X, NRF_I2S_RATIO_48X },		   \
			.divider = { NRF_I2S_MCK_32MDIV31, NRF_I2S_MCK_32MDIV31, NRF_I2S_MCK_32MDIV21 },   \
		},											   \
		{											   \
			.frequency = 44100,								   \
			.ratio = { NRF_I2S_RATIO_32X, NRF_I2S_RATIO_32X, NRF_I2S_RATIO_48X },		   \
			.divider = { NRF_I2S_MCK_32MDIV23, NRF_I2S_MCK_32MDIV23, NRF_I2S_MCK_32MDIV15 },   \
		},											   \
	}



/** @struct i2s_config
 * @brief Interface configuration options.
 *
 * Memory slab pointed to by the mem_slab field has to be defined and
 * initialized by the user. For I2S driver to function correctly number of
 * memory blocks in a slab has to be at least 2 per queue. Size of the memory
 * block should be multiple of frame_size where frame_size = (channels *
 * word_size_bytes). As an example 16 bit word will occupy 2 bytes, 24 or 32
 * bit word will occupy 4 bytes.
 *
 * Please check Zephyr Kernel Primer for more information on memory slabs.
 *
 * @remark When I2S data format is selected parameter channels is ignored,
 * number of words in a frame is always 2.
 *
 * @param word_size Number of bits representing one data word.
 * @param channels Number of words per frame.
 * @param format Data stream format as defined by I2S_FMT_* constants.
 * @param options Configuration options as defined by I2S_OPT_* constants.
 * @param frame_clk_freq Frame clock (WS) frequency, this is sampling rate.
 * @param mem_slab memory slab to store RX/TX data.
 * @param block_size Size of one RX/TX memory block (buffer) in bytes.
 * @param timeout Read/Write timeout. Number of milliseconds to wait in case TX
 *        queue is full, RX queue is empty or one of the special values
 *        K_NO_WAIT, K_FOREVER.
 */
#define RING_BUF_INC(idx, limit) {idx = (++idx < limit) ? idx : 0;}
#define STATIC


#define DEV_CFG(dev) \
	(const struct zephyr_i2s_cfg *const)((dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct zephyr_i2s_data *const)(dev)->driver_data)


struct zephyr_i2s_cfg {
	u8_t sck_pin;
	u8_t lrck_pin;
	u8_t mck_pin;
	u8_t sdout_pin;
	u8_t sdin_pin;
	void (*instance_init)(struct device *dev);
};

struct queue_item {
	void *data;
	u32_t size;
};

struct queue {
	struct queue_item *queue_items;
	u8_t read_idx;
	u8_t write_idx;
	u8_t len;
};

struct nrfx_i2s_stream {
	nrfx_i2s_buffers_t buffers;
	size_t size;
	struct channel_str *channel_tx;
	struct channel_str *channel_rx;
};

enum i2s_if_state {
	I2S_IF_NOT_READY = 0,
	I2S_IF_READY,
	I2S_IF_STARTING,
	I2S_IF_RUNNING,
	I2S_IF_STOPPING,
	I2S_IF_STOP,
	I2S_IF_RESTARTING,
	I2S_IF_ERROR
};

struct nrfx_i2s_interface {
	enum i2s_if_state state;
	struct nrfx_i2s_stream stream;
};
static struct nrfx_i2s_interface interface;

struct channel_str {
	struct k_sem sem;
	struct k_mem_slab *mem_slab;
	s32_t timeout;
	enum i2s_state current_state;
	struct i2s_config api_config_copy;
	struct queue mem_block_queue;
	int (*start)(struct channel_str *);
	int (*stop)(struct channel_str *);
	int (*drop)(struct channel_str *);
	void (*data_handler)(struct channel_str *config,
			     nrfx_i2s_buffers_t const *p_released, u32_t status,
			     nrfx_i2s_buffers_t *p_new_buffers);
	bool (*is_empty)(struct channel_str *config);
	int (*get_data)(struct channel_str *config, u32_t **buf, size_t *block_size);
};

struct zephyr_i2s_data {
	nrfx_i2s_config_t nrfx_driver_config;
	size_t block_size;
	struct channel_str channel_tx;
	struct channel_str channel_rx;
};

static int i2s_channel_get(enum i2s_dir dir,
			    struct zephyr_i2s_data *const dev_data,
			    struct channel_str **channel);
static int nrfx_i2s_tx_start(struct channel_str *direction_tx);
static int nrfx_i2s_tx_stop(struct channel_str *direction_tx);

static void rx_callback(struct channel_str *direction_tx,
			nrfx_i2s_buffers_t const *p_released, u32_t status,
			nrfx_i2s_buffers_t *p_new_buffers);

static int i2s_nrfx_change_channel_state(struct channel_str *channel,
				 enum i2s_state new_state);

static int nrfx_i2s_rx_stop(struct channel_str *direction_rx);























//-------------------------ready------------------------------------------------
static void i2s_nrfx_interface_error_service();
static int i2s_nrfx_interface_set_state(enum i2s_if_state new_state);
static enum i2s_if_state i2s_nrfx_interface_get_state();
static int i2s_nrfx_interface_start();
static int i2s_nrfx_interface_restart();

static void i2s_nrfx_interface_error_service()
{
	LOG_INF1("[%s]", __func__);
	i2s_nrfx_interface_set_state(I2S_IF_ERROR);
	nrfx_i2s_stop();
}

static int i2s_nrfx_interface_set_state(enum i2s_if_state new_state)
{
	bool change_impossible = false;
	switch (new_state) {
	case I2S_IF_STOPPING:
		if (interface.state != I2S_IF_RUNNING)
			change_impossible = true;
		break;
	case I2S_IF_RESTARTING:
		if (interface.state != I2S_IF_RUNNING)
			change_impossible = true;
		break;
	case I2S_IF_RUNNING:
	case I2S_IF_READY:
	case I2S_IF_STARTING:
	case I2S_IF_STOP:
	case I2S_IF_ERROR:
		break;
	default:
		return -EINVAL;
	}
	if (change_impossible) {
		i2s_nrfx_interface_error_service();
		return -EIO;
	}
	interface.state = new_state;
	LOG_INF1("\r\n[%s] - %u\r\n", __func__, new_state);
	return 0;
}

static inline enum i2s_if_state i2s_nrfx_interface_get_state()
{
	return interface.state;
}

static int i2s_nrfx_interface_restart()
{
	int ret;
	ret = i2s_nrfx_interface_set_state(I2S_IF_RESTARTING);
	if (ret != 0) {
		i2s_nrfx_interface_error_service();
		return ret;
	}
	return 0;
}

static int i2s_nrfx_interface_stop()
{
	int ret;
	ret = i2s_nrfx_interface_set_state(I2S_IF_STOPPING);
	if (ret != 0) {
		i2s_nrfx_interface_error_service();
		return ret;
	}
	return 0;
}

static int i2s_nrfx_interface_start()
{
	int ret;
	ret = i2s_nrfx_interface_set_state(I2S_IF_RUNNING);
	if (ret != 0) {
		i2s_nrfx_interface_error_service();
		return ret;
	}
	LOG_INF1("\r\n[%s]Buffer tx:%p Buffer rx:%p\r\n", __func__, interface.stream.buffers.p_tx_buffer, interface.stream.buffers.p_rx_buffer);
	nrfx_err_t status = nrfx_i2s_start(&interface.stream.buffers, interface.stream.size, 0);
	if (status != NRFX_SUCCESS) {
		ret = -EIO;
	}
	return ret;
}

static inline int nrfx_i2s_set_error_state(struct channel_str *channel, int err_code) {
	i2s_nrfx_change_channel_state(channel, I2S_STATE_ERROR);
	return err_code;
}

void nrfx_i2s_stream_handler(nrfx_i2s_buffers_t const *p_released,
		u32_t status)
{
	LOG_INF1("[%s enter(%p, %u)] -> ", __func__, p_released, status);
	struct channel_str *rx_str = interface.stream.channel_rx;
	struct channel_str *tx_str = interface.stream.channel_tx;
	nrfx_i2s_buffers_t p_new_buffers;

	p_new_buffers.p_rx_buffer = NULL;
	p_new_buffers.p_tx_buffer = NULL;

	if ((status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED)
	    == NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) {
		if (rx_str != NULL && (rx_str->current_state == I2S_STATE_RUNNING || rx_str->current_state == I2S_STATE_STOPPING)) {
			rx_str->data_handler(rx_str, p_released, status, &p_new_buffers);
		}
		if (tx_str != NULL && (tx_str->current_state == I2S_STATE_RUNNING || tx_str->current_state == I2S_STATE_STOPPING)) {
			tx_str->data_handler(tx_str, p_released, status, &p_new_buffers);
		}
		if (i2s_nrfx_interface_get_state() == I2S_IF_RESTARTING ||
		    i2s_nrfx_interface_get_state() == I2S_IF_STOPPING) {
			LOG_INF1("\r\n[%s]I2S_IF_RESTARTING or  I2S_IF_STOPPING\r\n", __func__);
			nrfx_i2s_stop();
			return;
		}
		else if (nrfx_i2s_next_buffers_set(&p_new_buffers) != NRFX_SUCCESS) {
			LOG_ERR("Error while setting new buffers");
			nrfx_i2s_stop();
			NRFX_I2S_REPORT_ERROR(INTERNAL);
			return;

		}
		if ((p_new_buffers.p_rx_buffer == NULL)
		    && (p_new_buffers.p_tx_buffer == NULL)) {
			LOG_ERR("\r\nboth p_new_buffers are NULL\r\n", __func__);
			nrfx_i2s_stop();
			//NRFX_I2S_REPORT_ERROR(INTERNAL);
			return;
		}
		interface.stream.buffers = p_new_buffers;
	}
	else { //stopping
		if (i2s_nrfx_interface_get_state() == I2S_IF_RESTARTING) {
			LOG_INF1("\r\n[%s] restarting", __func__);
			if (i2s_nrfx_interface_set_state(I2S_IF_STARTING) != 0) {
				LOG_ERR("[%s] Error while changing interface state", __func__);
			}
			return;
		}
		else if (i2s_nrfx_interface_get_state() == I2S_IF_STOPPING) {
			LOG_INF1("\r\n[%s] stopping", __func__);
			if (i2s_nrfx_interface_set_state(I2S_IF_READY)) {
				LOG_ERR("[%s] Error while changing interface state", __func__);
			}
			return;
		}
		else {
			LOG_INF1("\r\n[%s]UNKNOWN (%u)\r\n", __func__, i2s_nrfx_interface_get_state());
		}
	}
}

static inline nrf_i2s_mck_t i2s_get_divider(i2s_clk_settings_t const *clk_settings,
					    u8_t word_size)
{
	u32_t sub_idx = (word_size >> 3) - 1;

	return clk_settings->divider[sub_idx];
}

static inline nrf_i2s_ratio_t i2s_get_ratio(i2s_clk_settings_t const *clk_settings,
					    u8_t word_size)
{
	u32_t sub_idx = (word_size >> 3) - 1;

	return clk_settings->ratio[sub_idx];
}


static int i2s_nrfx_change_channel_state(struct channel_str *channel,
				 enum i2s_state new_state)
{
	bool change_impossible = false;
	enum i2s_state old_state = channel->current_state;
	LOG_INF1("***Changing_state %u->%u***", old_state, new_state);
	switch (new_state) {
	case I2S_STATE_NOT_READY:
		if (1) /*(old_state != I2S_STATE_NOT_READY && old_state != I2S_STATE_ERROR)*/ {
			change_impossible = true;
		}
		break;
	case I2S_STATE_READY:
		if (old_state == I2S_STATE_READY) {
			change_impossible = true;
		}
		break;
	case I2S_STATE_RUNNING:
		if (old_state == I2S_STATE_RUNNING) {
			change_impossible = true;
		}
		break;
	case I2S_STATE_STOPPING:
		if (old_state != I2S_STATE_RUNNING /*&& old_state != I2S_STATE_STOPPING*/) {
			change_impossible = true;
		}
		break;
	case I2S_STATE_ERROR:
		if (old_state == I2S_STATE_NOT_READY) {
			change_impossible = true;
		}
		break;
	default:
		LOG_ERR("Invalid state");
		return -EINVAL;
	}

	if (change_impossible) {
		LOG_ERR("Can't switch from state %u to %u", old_state, new_state);
		return -EIO;
	}
	channel->current_state = new_state;
	return 0;
}

static void i2s_nrfx_match_clock_settings(nrfx_i2s_config_t *config,
			       struct i2s_config const *i2s_cfg)
{
	const struct i2s_clk_settings_t i2s_clock_settings[] = NRFX_I2S_AVAILABLE_CLOCK_SETTINGS;
	u32_t des_s_r = (s32_t)i2s_cfg->frame_clk_freq;
	u8_t nb_of_settings_array_elems =
		sizeof(i2s_clock_settings) / sizeof(i2s_clock_settings[0]);
	i2s_clk_settings_t const *chosen_settings =
		&i2s_clock_settings[nb_of_settings_array_elems - 1];


	for (u8_t i = 1; i < nb_of_settings_array_elems; ++i) {
		if (des_s_r < i2s_clock_settings[i].frequency) {
			u32_t diff_h = i2s_clock_settings[i].frequency - des_s_r;
			u32_t diff_l = abs((s32_t)des_s_r - (s32_t)i2s_clock_settings[i - 1].frequency);
			chosen_settings = (diff_h < diff_l) ?
					  (&i2s_clock_settings[i]) :
					  (&i2s_clock_settings[i - 1]);
			break;
		}
	}
	config->mck_setup = i2s_get_divider(chosen_settings,
					    i2s_cfg->word_size);
	config->ratio = i2s_get_ratio(chosen_settings,
				      i2s_cfg->word_size);
}

static int i2s_periph_config(struct device *dev,
			     struct i2s_config const *i2s_cfg)
{
	const struct zephyr_i2s_cfg *const dev_const_cfg = DEV_CFG(dev);
	struct zephyr_i2s_data *const dev_data = DEV_DATA(dev);

	if (i2s_cfg == NULL) {
		LOG_ERR("[%s]Invalid i2s_cfg handler", __func__);
		return -EINVAL;
	}

	if (dev_data == NULL) {
		LOG_ERR("[%s]Invalid dev_data handler", __func__);
		return -EINVAL;
	}

	if (i2s_cfg->mem_slab == NULL) {
		LOG_ERR("[%s]No valid memory slab defined", __func__);
		return -EINVAL;
	}

	nrfx_i2s_config_t drv_cfg;
	memset(&drv_cfg, 0, sizeof(nrfx_i2s_config_t));

	drv_cfg.sck_pin = dev_const_cfg->sck_pin;
	drv_cfg.lrck_pin = dev_const_cfg->lrck_pin;
	drv_cfg.mck_pin = dev_const_cfg->mck_pin;
	drv_cfg.sdout_pin = dev_const_cfg->sdout_pin;
	drv_cfg.sdin_pin = dev_const_cfg->sdin_pin;

	if (dev_data) {
		switch (i2s_cfg->word_size) {
		case 8:
			drv_cfg.sample_width = NRF_I2S_SWIDTH_8BIT;
			break;
		case 16:
			drv_cfg.sample_width = NRF_I2S_SWIDTH_16BIT;
			break;
		case 24:
			drv_cfg.sample_width = NRF_I2S_SWIDTH_24BIT;
			break;
		default:
			LOG_ERR("[%s]Invalid word size", __func__);
			return -EINVAL;
		}
	}

	if (i2s_cfg->frame_clk_freq == 0) {
		LOG_ERR("[%s]Invalid LRCK value", __func__);
		return -EINVAL;
	}

	switch (i2s_cfg->format /*& I2S_FMT_DATA_FORMAT_MASK*/) {
	case I2S_FMT_DATA_FORMAT_I2S:
		drv_cfg.alignment = NRF_I2S_ALIGN_LEFT;
		drv_cfg.format = NRF_I2S_FORMAT_I2S;
		break;

	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		drv_cfg.alignment = NRF_I2S_ALIGN_LEFT;
		drv_cfg.format = NRF_I2S_FORMAT_ALIGNED;
		break;

	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		drv_cfg.alignment = NRF_I2S_ALIGN_RIGHT;
		drv_cfg.format = NRF_I2S_FORMAT_ALIGNED;
		break;

	default:
		LOG_ERR("[%s]Unsupported I2S data format", __func__);
		return -EINVAL;
	}

	switch (i2s_cfg->options) {
	case I2S_OPT_BIT_CLK_CONT:
		break;
	default:
		LOG_ERR("[%s]Unsupported I2S data format", __func__);
		return -EINVAL;
	}

	if ((i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE) ||
	    (i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE)) {
		drv_cfg.mode = NRF_I2S_MODE_SLAVE;
		LOG_ERR("Slave mode is not supported at the moment");
		return -ENOTSUP;
	} else {
		drv_cfg.mode = NRF_I2S_MODE_MASTER;
	}

	switch (i2s_cfg->channels) {
	case 2:
		drv_cfg.channels = NRF_I2S_CHANNELS_STEREO;
		break;
	case 1:
		LOG_ERR("[%s]At the moment mono mode is not supported", __func__);
		return -ENOTSUP;
	default:
		LOG_ERR("[%s]Invalid number of channels", __func__);
		return -EINVAL;
	}

	/* block size is common for tx and rx so raise an error if not the same */
	if ((dev_data->block_size != 0 &&
	     dev_data->block_size != i2s_cfg->block_size) ||
	     i2s_cfg->block_size == 0) {
		LOG_ERR("[%s]Invalid block size", __func__);
		return -EINVAL;
	}
	dev_data->block_size = i2s_cfg->block_size;

	i2s_nrfx_match_clock_settings(&drv_cfg, i2s_cfg);
	memcpy(&dev_data->nrfx_driver_config, &drv_cfg,
	       sizeof(nrfx_i2s_config_t));

	return 0;
}

int i2s_nrfx_api_configure(struct device *dev, enum i2s_dir dir,
			   struct i2s_config *i2s_cfg)
{
	LOG_INF1("\r\n[%s] (%p %u %p)\r\n", __func__, dev, dir, i2s_cfg);
	struct zephyr_i2s_data *const dev_data = DEV_DATA(dev);
	struct channel_str *channel;
	int ret;

	if (dev == NULL || i2s_cfg == NULL) {
		LOG_ERR("Invalid dev or i2s_cfg object");
		return -EINVAL;
	}

	ret = i2s_channel_get(dir, dev_data, &channel);
	if (ret != 0) {
		LOG_ERR("Can't get channel");
		return nrfx_i2s_set_error_state(channel, ret);
	}

	ret = i2s_periph_config(dev, i2s_cfg);
	/* disable channels in case of invalid configuration*/
	if (ret != 0) {
		//channel->drop(channel);
		//channel->stop(channel);
		//dev_data->sample_rate = 0;
		LOG_ERR("Can't configure peripheral");
		return nrfx_i2s_set_error_state(channel, ret);
	}


//tmp
	if (interface.stream.size != 0 && interface.stream.size != dev_data->block_size / 4) {
		LOG_ERR("[%s]Invalid size (given: %u, expected %u)", __func__, dev_data->block_size / 4, interface.stream.size);
		return -EINVAL;
	}
	if (dir == I2S_DIR_RX) {
		interface.stream.buffers.p_rx_buffer = NULL;
		interface.stream.channel_rx = channel;

	}
	if (dir == I2S_DIR_TX) {
		interface.stream.buffers.p_tx_buffer = NULL;
		interface.stream.channel_tx = channel;
	}
	interface.stream.size = dev_data->block_size / 4;
	channel->mem_slab = i2s_cfg->mem_slab;
	channel->timeout = i2s_cfg->timeout;
	ret = i2s_nrfx_change_channel_state(channel, I2S_STATE_READY);
	if (ret != 0) {
		return nrfx_i2s_set_error_state(channel, ret);
	}
	nrfx_err_t status;
	if (i2s_nrfx_interface_get_state() == I2S_IF_NOT_READY) {
		status = nrfx_i2s_init(&dev_data->nrfx_driver_config, nrfx_i2s_stream_handler);
		if (status != NRFX_SUCCESS) {
			if (status == NRFX_ERROR_INVALID_STATE) {
				LOG_ERR("Invalid state");
				return nrfx_i2s_set_error_state(channel, -EINVAL);
			}
			else if (status == NRFX_ERROR_INVALID_PARAM) {
				LOG_ERR("Invalid param");
				return nrfx_i2s_set_error_state(channel, -EINVAL);
			}
			else {
				LOG_ERR("Unknown error, status = %d", status);
				return nrfx_i2s_set_error_state(channel, -ENOTSUP);
			}
		}
		ret = i2s_nrfx_interface_set_state(I2S_IF_READY);
		if (ret != 0) {
			LOG_ERR("Can't change state");
			return nrfx_i2s_set_error_state(channel, -ENOTSUP);
		}
	}
	else if(i2s_nrfx_interface_get_state() != I2S_IF_READY) {
		LOG_ERR("Invalid state during initialization");
		return nrfx_i2s_set_error_state(channel, -ENOTSUP);
	}
	return ret;
}


//------------------------------------------------------------------------------
















































/*
 *
 *  QUEUE
 *
 */
void queue_init(struct queue *queue, u8_t len, struct queue_item *queue_items)
{
	queue->read_idx = 0;
	queue->write_idx = 0;
	queue->len = len;
	queue->queue_items = queue_items;
}


static int queue_add(struct queue *queue, void *data, u32_t size)
{
	u8_t wr_idx = queue->write_idx;

	RING_BUF_INC(wr_idx, queue->len);

	if (wr_idx == queue->read_idx) {
		/* cannot overwrite unread data */
		LOG_ERR("\r\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![%s] can't add to queue\r\n", __func__);
		NRFX_I2S_REPORT_ERROR(QUEUE_FULL);
		return -ENOMEM;
	}

	queue->queue_items[queue->write_idx].data = data;
	queue->queue_items[queue->write_idx].size = size;
	LOG_MEM("\r\nLOG_MEM[%s]ADD %p\r\n", __func__, queue->queue_items[queue->write_idx].data);
	queue->write_idx = wr_idx;
	return 0;
}


static inline bool queue_is_empty(struct queue *queue)
{
	return (queue->read_idx == queue->write_idx) ? true : false;
}


STATIC int queue_fetch(struct queue *queue, void **data, u32_t *size)
{
	if (queue_is_empty(queue)) {
		return -ENOMEM;
	}
	*data = queue->queue_items[queue->read_idx].data;
	*size = queue->queue_items[queue->read_idx].size;

	LOG_MEM("\r\nLOG_MEM[%s]FETCH %p\r\n", __func__, queue->queue_items[queue->read_idx].data);

	RING_BUF_INC(queue->read_idx, queue->len);
	return 0;
}



struct i2s_config *nrfx_i2s_config_get(struct device *dev, enum i2s_dir dir)
{
	struct channel_str *dir_config;
	int ret;

	ret = i2s_channel_get(dir, DEV_DATA(dev), &dir_config);
	if (ret != 0) {
		return NULL;
	}

	return &dir_config->api_config_copy;
}


int nrfx_i2s_read(struct device *dev, void **mem_block, size_t *size)
{
	struct channel_str *channel = interface.stream.channel_rx;//dev_data->channel_rx;
	int ret;

	if (channel->current_state == I2S_STATE_NOT_READY || channel->current_state == I2S_STATE_ERROR) {
		LOG_ERR("invalid state");
		return -EIO;
	}
	/*if (queue_is_empty(&channel->mem_block_queue) == true) {
		return -EIO;
	}*/
	ret = k_sem_take(&channel->sem, channel->timeout * 10);
	if (ret < 0) {
		LOG_ERR("Can't take semaphore (code=%d)", ret);
		return ret;
	}
	LOG_INF1("\r\n[%s]TAKE (%u / %u)\r\n", __func__, interface.stream.channel_rx->sem.count, interface.stream.channel_rx->sem.limit);
	ret = queue_fetch(&channel->mem_block_queue, mem_block, size);
	if (ret < 0) {
		return -EIO;
	}

	return 0;
}

int nrfx_i2s_write(struct device *dev, void *mem_block, size_t size)
{
	//struct zephyr_i2s_data *const dev_data = DEV_DATA(dev);
	struct channel_str *channel = interface.stream.channel_tx;//dev_data->channel_tx;
	int ret;

	if (channel->current_state != I2S_STATE_READY && channel->current_state != I2S_STATE_RUNNING) {
		return -EIO;
	}

	ret = k_sem_take(&channel->sem, channel->timeout * 2);
	if (ret < 0) {
		NRFX_I2S_REPORT_ERROR(SEM_UNAVAILABLE);
		return ret;
	}
	LOG_MEM("\r\n[%s]TAKE (%u / %u)\r\n", __func__, interface.stream.channel_tx->sem.count, interface.stream.channel_tx->sem.limit);
	ret = queue_add(&channel->mem_block_queue, mem_block, size / 4);
	return ret;
}


STATIC int nrfx_i2s_trigger(struct device *dev, enum i2s_dir dir,
			    enum i2s_trigger_cmd cmd)
{
	struct zephyr_i2s_data *const dev_data = DEV_DATA(dev);
	struct channel_str *channel;
	int ret;

	ret = i2s_channel_get(dir, dev_data, &channel);
	if (ret != 0) {
		return ret;
	}

	LOG_INF1("\r\n[%s](%u)\r\n", __func__, cmd);

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (channel->current_state != I2S_STATE_READY) {
			LOG_ERR("[%s]Invalid channel state (%u) for I2S_TRIGGER_START command", __func__, channel->current_state);
			return -EIO;
		}
		ret = channel->start(channel);//nrfx_i2s_tx_start();
		if (ret < 0) {
			LOG_ERR("%lld: Invalid cmd found(%u). Can't change state from (%u)", k_uptime_get(), cmd, channel->current_state);
			return ret;
		}
		break;

	case I2S_TRIGGER_STOP:

		if (channel->current_state != I2S_STATE_RUNNING) {
			LOG_ERR("%lld: Invalid cmd found(%u). Can't change state from (%u)", k_uptime_get(), cmd, channel->current_state);
			return -EIO;
		}
		return channel->stop(channel);//nrfx_i2s_tx_stop
		break;

	case I2S_TRIGGER_DRAIN:
		if (channel->current_state != I2S_STATE_RUNNING) {
			LOG_ERR("%lld: Invalid cmd found(%u). Can't change state from (%u)", k_uptime_get(), cmd, channel->current_state);
			return -EIO;
		}
		//ret = channel->stop(channel);
		if (ret < 0) {
			return ret;
		}
		channel->drop(channel);
		break;

	case I2S_TRIGGER_DROP:
		if (channel->current_state == I2S_STATE_NOT_READY) {
			LOG_ERR("%lld: Invalid cmd found(%u). Can't change state from (%u)", k_uptime_get(), cmd, channel->current_state);
			return -EIO;
		}
		//ret = channel->stop(channel);
		if (ret < 0) {
			return ret;
		}
		channel->drop(channel);
		break;

	case I2S_TRIGGER_PREPARE:
		if (channel->current_state != I2S_STATE_ERROR) {
			LOG_ERR("%lld: Invalid cmd found(%u). Can't change state from (%u)", k_uptime_get(), cmd, channel->current_state);
			return -EIO;
		}
		return channel->drop(channel);
		break;

	default:
		LOG_ERR("Unsupported trigger command");
		return -EINVAL;
	}
	return 0;
}



/*
 *
 *   API HANDLERS
 *
 */


static bool tx_check_empty(struct channel_str *direction_tx)
{
	return queue_is_empty(&direction_tx->mem_block_queue);
}


static int tx_get_data(struct channel_str *direction_tx, u32_t **buf,
		size_t *block_size)
{
	int ret;

	ret = queue_fetch(&direction_tx->mem_block_queue, (void **)buf, block_size);
	if (ret < 0) {
		return ret;
	}
	k_sem_give(&direction_tx->sem);
	LOG_MEM("[%s]GIVE (%u / %u) -> ", __func__, interface.stream.channel_tx->sem.count, interface.stream.channel_tx->sem.limit);
	return 0;
}


static void tx_callback(struct channel_str *direction_tx,
			nrfx_i2s_buffers_t const *p_released, u32_t status,
			nrfx_i2s_buffers_t *p_new_buffers)
{
	LOG_INF1("[%s(%u)] -> ", __func__, (u32_t)k_uptime_get());
	//trg_pin_drv(2,0);
	size_t mem_block_size;
	u32_t *mem_block;
	int ret;
	if (p_released->p_tx_buffer != NULL) {
		k_mem_slab_free(direction_tx->mem_slab, (void **)&p_released->p_tx_buffer);
	}
	if (status == 0) {
		LOG_INF1("[%s] STATUS IS ZERO -> ", __func__);
		return;
	}
	if (direction_tx->current_state == I2S_STATE_STOPPING) {
		ret = i2s_nrfx_change_channel_state(direction_tx, I2S_STATE_READY);
		if (ret != 0) {
			nrfx_i2s_stop();
			NRFX_I2S_REPORT_ERROR(GET_DATA);
			return;
		}
		while (direction_tx->get_data(direction_tx, &mem_block, &mem_block_size) == 0) {
			k_mem_slab_free(direction_tx->mem_slab, (void**)&mem_block);
		}
		for (unsigned int sem_cnt = direction_tx->sem.count; sem_cnt < direction_tx->sem.limit; sem_cnt ++) {
			//LOG_INF1("[%s]GIVE (%u / %u)\r\n", __func__, direction_tx->sem.count, direction_tx->sem.limit);
			k_sem_give(&direction_tx->sem);
		}
		k_mem_slab_free(direction_tx->mem_slab, (void**)&interface.stream.buffers.p_tx_buffer);
		interface.stream.buffers.p_tx_buffer = NULL;
		return;
	}
	else if (i2s_nrfx_interface_get_state() == I2S_IF_RESTARTING) {
		return;
	}
	ret = direction_tx->get_data(direction_tx, &mem_block, &mem_block_size);
	if (ret != 0) {
		LOG_ERR("\r\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![%s] Queue fetching error\r\n", __func__);
		nrfx_i2s_tx_stop(direction_tx);
		NRFX_I2S_REPORT_ERROR(GET_DATA);
		return;
	}

	/* continue transmission */
	p_new_buffers->p_tx_buffer = mem_block;
	u16_t* d = (u16_t*)p_new_buffers->p_tx_buffer;
	LOG_INF1("[%s](p_released = %p, p_new_buffers = %p) -> ", __func__, p_released->p_tx_buffer, p_new_buffers->p_tx_buffer);
	LOG_INF1("[%04X][%04X][%04X]", d[0], d[1], d[2]);
	if (direction_tx->is_empty(direction_tx) == true) {
		LOG_ERR("\r\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![%s] Underrun[data : %04X %04X\r\n", __func__, p_released->p_tx_buffer[0], p_released->p_tx_buffer[1]);
		nrfx_i2s_tx_stop(direction_tx);
		NRFX_I2S_REPORT_ERROR(UNDERRUN);
		return;
	}
	return;
}

static void rx_callback(struct channel_str *direction_rx,
			nrfx_i2s_buffers_t const *p_released, u32_t status,
			nrfx_i2s_buffers_t *p_new_buffers)
{
	LOG_INF1("[%s(%u)] -> ", __func__, (u32_t)k_uptime_get());
	//trg_pin_drv(3, 0);
	int ret;
	if (p_released->p_rx_buffer != NULL) {
		ret = queue_add(&direction_rx->mem_block_queue, (void *)p_released->p_rx_buffer, interface.stream.size);
		if (ret < 0) {
			return;
		}
		k_sem_give(&direction_rx->sem);
		LOG_INF1("[%s]GIVE (%u / %u) -> ", __func__, interface.stream.channel_rx->sem.count, interface.stream.channel_rx->sem.limit);
	}
	if (status == 0) {
		LOG_INF1("[%s] STATUS IS ZERO -> ", __func__);
		return;
	}
	if (direction_rx->current_state == I2S_STATE_STOPPING) {

		ret = i2s_nrfx_change_channel_state(direction_rx, I2S_STATE_READY);
		if (ret != 0) {
			nrfx_i2s_stop();
			NRFX_I2S_REPORT_ERROR(GET_DATA);
		}

		if (interface.stream.buffers.p_rx_buffer != NULL) {
			k_mem_slab_free(direction_rx->mem_slab, (void**)&interface.stream.buffers.p_rx_buffer);
		}
		else {
			LOG_INF1("--------------------------------------------------------------------\r\n---------------------------------------------------------\r\n---------------------------------------------------------\r\n");
		}
		interface.stream.buffers.p_rx_buffer = NULL;
		return;
	}
	else if (i2s_nrfx_interface_get_state() == I2S_IF_RESTARTING) {
		return;
	}
	//k_sem_reset(&direction_rx->sem); - gdzies to musi byc przy stopie
	ret = k_mem_slab_alloc(direction_rx->mem_slab, (void **)&p_new_buffers->p_rx_buffer, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("Cannot allocate memory (code: %d)", ret);
		return;
	}


}

static bool rx_check_empty(struct channel_str *direction_rx)
{
	return queue_is_empty(&direction_rx->mem_block_queue);
}


static int rx_get_data(struct channel_str *direction_rx, u32_t **buf,
		size_t *block_size)
{

	return 0;
}

static int nrfx_i2s_tx_start(struct channel_str *direction_tx)
{
	LOG_INF1("\r\n[%s(%u)]\r\n", __func__, (u32_t)k_uptime_get());
	int ret;
	unsigned int key;

	ret = i2s_nrfx_change_channel_state(direction_tx, I2S_STATE_RUNNING);
	if (ret != 0) {
		return ret;
	}

	key = irq_lock();
	if (i2s_nrfx_interface_get_state() == I2S_IF_RUNNING) {
		LOG_INF1("\r\n[%s]I2S_IF_RUNNING\r\n", __func__);
		ret = direction_tx->get_data(direction_tx, (u32_t**)&interface.stream.buffers.p_tx_buffer, &interface.stream.size);
		ret = i2s_nrfx_interface_restart();
		irq_unlock(key);
		if (ret != 0) {
			LOG_ERR("[%s] Can't restart interface", __func__);
			return ret;
		}
	}
	else if (i2s_nrfx_interface_get_state() == I2S_IF_READY) {
		LOG_INF1("\r\n[%s]I2S_IF_READY\r\n", __func__);
		ret = direction_tx->get_data(direction_tx, (u32_t**)&interface.stream.buffers.p_tx_buffer, &interface.stream.size);
		if (ret != 0) {
			irq_unlock(key);
			LOG_ERR("[%s]Can't get data", __func__);
			return ret;
		}
		ret = i2s_nrfx_interface_start();
		LOG_INF1("\r\n[%s]STARTING\r\n", __func__);
		irq_unlock(key);
		if (ret != 0) {
			LOG_ERR("[%s] Can't start interface", __func__);
			return ret;
		}
	}
	else {
		irq_unlock(key);
		LOG_ERR("[%s]Invalid interface state (%u)", __func__, i2s_nrfx_interface_get_state());
		return -EIO;
	}
	return 0;
}

static int nrfx_i2s_rx_start(struct channel_str *direction_rx)
{
	LOG_INF1("\r\n[%s(%u)]\r\n", __func__, (u32_t)k_uptime_get());
	int ret;
	unsigned int key;

	ret = i2s_nrfx_change_channel_state(direction_rx, I2S_STATE_RUNNING);
	if (ret != 0) {
		return ret;
	}

	key = irq_lock();
	if (i2s_nrfx_interface_get_state() == I2S_IF_RUNNING) {
		LOG_INF1("\r\n[%s]I2S_IF_RUNNING\r\n", __func__);
		ret = k_mem_slab_alloc(direction_rx->mem_slab,
						       (void**)&interface.stream.buffers.p_rx_buffer, K_NO_WAIT);
		ret = i2s_nrfx_interface_restart();
		irq_unlock(key);
		if (ret != 0) {
			LOG_ERR("[%s] Can't restart interface", __func__);
			return ret;
		}
	}
	else if (i2s_nrfx_interface_get_state() == I2S_IF_READY) {
		LOG_INF1("\r\n[%s]I2S_IF_READY\r\n", __func__);
		ret = k_mem_slab_alloc(direction_rx->mem_slab,
				       (void**)&interface.stream.buffers.p_rx_buffer, K_NO_WAIT);
		if (ret != 0) {
			irq_unlock(key);
			LOG_ERR("[%s]Can't get data", __func__);
			return ret;
		}
		ret = i2s_nrfx_interface_start();
		irq_unlock(key);
		if (ret != 0) {
			LOG_ERR("[%s] Can't start interface", __func__);
			return ret;
		}
	}
	else {
		irq_unlock(key);
		LOG_ERR("[%s]Invalid interface state (%u)", __func__, i2s_nrfx_interface_get_state());
		return -EIO;
	}
	return 0;

}

static int nrfx_i2s_tx_stop(struct channel_str *direction_tx)
{
	LOG_INF1("\r\n[%s(%u)]\r\n", __func__, (u32_t)k_uptime_get());
	int ret;
	ret = i2s_nrfx_change_channel_state(direction_tx, I2S_STATE_STOPPING);
	if (ret != 0) {
		return ret;
	}
	if (interface.stream.channel_rx->current_state == I2S_STATE_RUNNING) {
		ret = i2s_nrfx_interface_restart();
		if (ret != 0) {
			return ret;
		}
	}
	else {
		ret = i2s_nrfx_interface_stop();
		if (ret != 0) {
			return ret;
		}
	}
	return 0;
}

static int nrfx_i2s_rx_stop(struct channel_str *direction_rx)
{
	LOG_INF1("\r\n[%s(%u)]\r\n", __func__, (u32_t)k_uptime_get());
	int ret;
	ret = i2s_nrfx_change_channel_state(direction_rx, I2S_STATE_STOPPING);
	if (ret != 0) {
		return ret;
	}
	if (interface.stream.channel_tx->current_state == I2S_STATE_RUNNING) {
		ret = i2s_nrfx_interface_restart();
		if (ret != 0) {
			return ret;
		}
	}
	else {
		ret = i2s_nrfx_interface_stop();
		if (ret != 0) {
			return ret;
		}
	}
	return 0;
}

static int i2s_channel_get(enum i2s_dir dir,
			    struct zephyr_i2s_data *const dev_data,
			    struct channel_str **channel)
{
	switch (dir) {
	case I2S_DIR_RX:
		*channel = &dev_data->channel_rx;
		break;
	case I2S_DIR_TX:
		*channel = &dev_data->channel_tx;
		break;
	default:
		LOG_ERR("Either RX or TX direction must be selected");
		return -EINVAL;
	}
	return 0;
}


static void _i2s_isr(void *arg)
{
	/* pass the interrupt to nrfx */
	nrfx_i2s_irq_handler();
	if (i2s_nrfx_interface_get_state() == I2S_IF_STARTING) {
		LOG_INF1("\r\n[%s] starting", __func__);
		if (i2s_nrfx_interface_set_state(I2S_IF_RUNNING) != 0) {
			LOG_ERR("[%s] Error while changing interface state", __func__);
			return;
		}
		nrfx_err_t status = nrfx_i2s_start(&interface.stream.buffers, interface.stream.size, 0);
		if (status != NRFX_SUCCESS) {
			//i2s_nrfx_interface_error_service();
			LOG_ERR("Error %08x while starting peripheral", status);
		}
		return;
	}
	//trg_pin_drv(2,1);
	//trg_pin_drv(3,1);
}

static int nrfx_i2s_initialize(struct device *dev)
{
	const struct zephyr_i2s_cfg *const dev_const_cfg = DEV_CFG(dev);
	struct zephyr_i2s_data *const dev_data = DEV_DATA(dev);
	k_sem_init(&dev_data->channel_rx.sem, 0,
		   CONFIG_NRFX_I2S_RX_BLOCK_COUNT);
	k_sem_init(&dev_data->channel_tx.sem, CONFIG_NRFX_I2S_TX_BLOCK_COUNT,
		   CONFIG_NRFX_I2S_TX_BLOCK_COUNT);
	i2s_nrfx_interface_set_state(I2S_IF_NOT_READY);
	dev_const_cfg->instance_init(dev);
	return 0;
}


#ifdef CONFIG_NRFX_I2S
struct queue_item rx_1_ring_buf[CONFIG_NRFX_I2S_RX_BLOCK_COUNT + 1];
struct queue_item tx_1_ring_buf[CONFIG_NRFX_I2S_TX_BLOCK_COUNT + 1];
STATIC void setup_instance_0(struct device *dev)
{
	struct zephyr_i2s_data *const dev_data = DEV_DATA(dev);

	queue_init(&dev_data->channel_tx.mem_block_queue,
		   CONFIG_NRFX_I2S_TX_BLOCK_COUNT + 1, &tx_1_ring_buf[0]);
	queue_init(&dev_data->channel_rx.mem_block_queue,
		   CONFIG_NRFX_I2S_RX_BLOCK_COUNT + 1, &rx_1_ring_buf[0]);

	IRQ_CONNECT(DT_NORDIC_NRF_I2S_DT_I2S_0_IRQ, DT_NORDIC_NRF_I2S_DT_I2S_0_IRQ_PRIORITY, _i2s_isr, 0, 0);
	irq_enable(DT_NORDIC_NRF_I2S_DT_I2S_0_IRQ);
}

STATIC const struct zephyr_i2s_cfg zephyr_i2s_cfg_0 = {
	.sck_pin = DT_NORDIC_NRF_I2S_DT_I2S_0_SCK_PIN,
	.lrck_pin =  DT_NORDIC_NRF_I2S_DT_I2S_0_LRCK_PIN,
	.mck_pin = DT_NORDIC_NRF_I2S_DT_I2S_0_MCK_PIN,
	.sdout_pin = DT_NORDIC_NRF_I2S_DT_I2S_0_SDOUT_PIN,
	.sdin_pin = DT_NORDIC_NRF_I2S_DT_I2S_0_SDIN_PIN,
	.instance_init = setup_instance_0,
};

struct zephyr_i2s_data zephyr_i2s_data_0 =
{
	.channel_tx =
	{
		.start = nrfx_i2s_tx_start,
		.stop = nrfx_i2s_tx_stop,
		.data_handler = tx_callback,
		.is_empty = tx_check_empty,
		.get_data = tx_get_data,
	},
	.channel_rx =
	{
		.start = nrfx_i2s_rx_start,
		.stop = nrfx_i2s_rx_stop,
		.data_handler = rx_callback,
		.is_empty = rx_check_empty,
		.get_data = rx_get_data,
	},
};

STATIC const struct i2s_driver_api i2s_nrf_driver_api = {
	.configure = i2s_nrfx_api_configure,
	.read = nrfx_i2s_read,
	.write = nrfx_i2s_write,
	.trigger = nrfx_i2s_trigger,
	.config_get = nrfx_i2s_config_get,
};



DEVICE_AND_API_INIT(i2s_0, DT_NORDIC_NRF_I2S_DT_I2S_0_LABEL, &nrfx_i2s_initialize,
		    &zephyr_i2s_data_0, &zephyr_i2s_cfg_0, POST_KERNEL,
		    CONFIG_I2S_INIT_PRIORITY, &i2s_nrf_driver_api);


#endif /* CONFIG_I2S_1 */
