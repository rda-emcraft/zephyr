
#include <string.h>
#include <stdlib.h>
#include <dma.h>
#include <i2s.h>
#include <soc.h>
#include <nrfx.h>
#include <nrfx_i2s.h>
#include <stdint.h>
#include "i2s_nrfx.h"
/*test and logging*/
#define USE_PRINTK_FOR_LOG_MESSAGES
#ifndef USE_PRINTK_FOR_LOG_MESSAGES
#define LOG_DOMAIN "i2s_nrfx"
#define LOG_LEVEL _LOG_LEVEL_DBG
#include <logging/log.h>
LOG_MODULE_REGISTER(i2s_nrfx);
#else
void f(u8_t *x, ...) {}
#define LOG_ERR	/*printk("\r\n--");*/printk
#define LOG_INF1 printk
#define LOG_MEM	printk
#endif

#define NRFX_I2S_REPORT_ERROR(err_code)





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
#define DEV_CFG(dev) \
	(const struct i2s_nrfx_config *const)((dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct i2s_nrfx_data *const)(dev)->driver_data)

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

struct i2s_nrfx_config {
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

struct channel_str {
	struct k_sem sem;
	struct k_mem_slab *mem_slab;
	s32_t timeout;
	enum i2s_state current_state;
	struct i2s_config api_config_copy;
	struct queue mem_block_queue;
	int (*start)(void);
	int (*stop)(void);
#warning this is nullptr
	int (*drop)(struct channel_str *);
	int (*drain)(struct channel_str *);
	void (*data_handler)(nrfx_i2s_buffers_t const *p_released, u32_t status,
			     nrfx_i2s_buffers_t *p_new_buffers);
	int (*get_data)(struct channel_str *config, u32_t **buf,
			size_t *block_size);
};

struct i2s_nrfx_data {
	nrfx_i2s_config_t nrfx_driver_config;
	size_t block_size;
	struct channel_str channel_tx;
	struct channel_str channel_rx;
};

struct i2s_nrfx_interface {
	enum i2s_if_state state;
	size_t size;
	nrfx_i2s_buffers_t buffers;
	struct channel_str *channel_tx;
	struct channel_str *channel_rx;
	void (*error_service)(void);
	int (*set_state)(enum i2s_if_state new_state);
	enum i2s_if_state (*get_state)(void);
	int (*start)(void);
	int (*stop)(void);
	int (*restart)(void);
	void (*handler)(nrfx_i2s_buffers_t const *, u32_t);
};

//static struct i2s_nrfx_interface interface;


static int i2s_nrfx_channel_get(enum i2s_dir dir,
			    struct i2s_nrfx_data *const dev_data,
			    struct channel_str **channel);

static int channel_change_state(struct channel_str *channel,
				 enum i2s_state new_state);

static inline struct i2s_nrfx_interface * get_interface(void);


static void interface_error_service(void)
{
	LOG_INF1("[%s]", __func__);
	struct i2s_nrfx_interface *i2s = get_interface();
	i2s->set_state(I2S_IF_ERROR);
	nrfx_i2s_stop();
}

static int interface_set_state(enum i2s_if_state new_state)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	bool change_forbidden = false;

	switch (new_state) {
	case I2S_IF_STOPPING:
		if (i2s->state != I2S_IF_RUNNING) {
			change_forbidden = true;
		}
		break;
	case I2S_IF_RESTARTING:
		if (i2s->state != I2S_IF_RUNNING) {
			change_forbidden = true;
		}
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

	if (change_forbidden) {
		i2s->error_service();
		return -EIO;
	}

	i2s->state = new_state;
	LOG_INF1("\r\n[%s] - %u\r\n", __func__, new_state);
	return 0;
}

static inline enum i2s_if_state interface_get_state(void)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	return i2s->state;
}

static int interface_restart(void)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	int ret;

	ret = i2s->set_state(I2S_IF_RESTARTING);

	if (ret != 0) {
		i2s->error_service();
		return ret;
	}

	return 0;
}

static int interface_stop(void)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	int ret;

	ret = i2s->set_state(I2S_IF_STOPPING);

	if (ret != 0) {
		i2s->error_service();
		return ret;
	}

	return 0;
}

static int interface_start(void)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	int ret;

	ret = i2s->set_state(I2S_IF_RUNNING);

	if (ret != 0) {
		i2s->error_service();
		return ret;
	}

	LOG_INF1("\r\n[%s]Buffer tx:%p Buffer rx:%p\r\n", __func__, i2s->buffers.p_tx_buffer, i2s->buffers.p_rx_buffer);
	nrfx_err_t status = nrfx_i2s_start(&i2s->buffers,
					   i2s->size, 0);

	if (status != NRFX_SUCCESS) {
		ret = -EIO;
	}

	return ret;
}

static void interface_handler(nrfx_i2s_buffers_t const *p_released,
				    u32_t status)
{
	LOG_INF1("[%s enter(%p, %u)] -> ", __func__, p_released, status);
	struct i2s_nrfx_interface *i2s = get_interface();
	struct channel_str *rx_str = i2s->channel_rx;
	struct channel_str *tx_str = i2s->channel_tx;

	nrfx_i2s_buffers_t p_new_buffers;
	p_new_buffers.p_rx_buffer = NULL;
	p_new_buffers.p_tx_buffer = NULL;

	if ((status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED)
	    == NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) {
		if (rx_str != NULL && (rx_str->current_state == I2S_STATE_RUNNING || rx_str->current_state == I2S_STATE_STOPPING)) {
			rx_str->data_handler(p_released, status, &p_new_buffers);
		}

		if (tx_str != NULL && (tx_str->current_state == I2S_STATE_RUNNING || tx_str->current_state == I2S_STATE_STOPPING)) {
			tx_str->data_handler(p_released, status, &p_new_buffers);
		}

		if (i2s->get_state() == I2S_IF_RESTARTING ||
		    i2s->get_state() == I2S_IF_STOPPING) {
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

		i2s->buffers = p_new_buffers;
	}
	else {
		if (i2s->get_state() == I2S_IF_RESTARTING) {
			LOG_INF1("\r\n[%s] restarting", __func__);
			if (i2s->set_state(I2S_IF_STARTING) != 0) {
				LOG_ERR("[%s] Error while changing interface state", __func__);
			}
			return;
		}
		else if (i2s->get_state() == I2S_IF_STOPPING) {
			LOG_INF1("\r\n[%s] stopping", __func__);
			if (i2s->set_state(I2S_IF_READY)) {
				LOG_ERR("[%s] Error while changing interface state", __func__);
			}
			return;
		}
		else {
			LOG_INF1("\r\n[%s]UNKNOWN (%u)\r\n", __func__, i2s->get_state());
		}
	}
}

static inline int channel_set_error_state(struct channel_str *channel,
					   int err_code)
{
	channel_change_state(channel, I2S_STATE_ERROR);

	return err_code;
}

static inline nrf_i2s_mck_t cfg_get_divider(i2s_clk_settings_t const *clk_set,
					    u8_t word_size)
{
	u32_t sub_idx = (word_size >> 3) - 1;

	return clk_set->divider[sub_idx];
}

static inline nrf_i2s_ratio_t cfg_get_ratio(i2s_clk_settings_t const *clk_set,
					    u8_t word_size)
{
	u32_t sub_idx = (word_size >> 3) - 1;

	return clk_set->ratio[sub_idx];
}

static void cfg_match_clock_settings(nrfx_i2s_config_t *config,
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
	config->mck_setup = cfg_get_divider(chosen_settings,
					    i2s_cfg->word_size);
	config->ratio = cfg_get_ratio(chosen_settings,
				      i2s_cfg->word_size);
}

static int cfg_periph_config(struct device *dev,
			     struct i2s_config const *i2s_cfg)
{
	const struct i2s_nrfx_config *const dev_const_cfg = DEV_CFG(dev);
	struct i2s_nrfx_data *const dev_data = DEV_DATA(dev);

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

	cfg_match_clock_settings(&drv_cfg, i2s_cfg);
	memcpy(&dev_data->nrfx_driver_config, &drv_cfg,
	       sizeof(nrfx_i2s_config_t));

	return 0;
}



//------------------------------------------------------------------------------
















































/*
 *
 *  Queue management
 *
 */
static void queue_init(struct queue *queue, u8_t len,
		       struct queue_item *queue_items)
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


static int queue_fetch(struct queue *queue, void **data, u32_t *size)
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


/*
 *
 * API functions
 *
 */
static int i2s_nrfx_api_configure(struct device *dev, enum i2s_dir dir,
			   struct i2s_config *i2s_cfg)
{
	LOG_INF1("\r\n[%s] (%p %u %p)\r\n", __func__, dev, dir, i2s_cfg);
	struct i2s_nrfx_interface *i2s = get_interface();
	struct i2s_nrfx_data *const dev_data = DEV_DATA(dev);
	struct channel_str *channel;
	int ret;

	if (dev == NULL || i2s_cfg == NULL) {
		LOG_ERR("Invalid dev or i2s_cfg object");
		return -EINVAL;
	}

	ret = i2s_nrfx_channel_get(dir, dev_data, &channel);
	if (ret != 0) {
		LOG_ERR("Can't get channel");
		return channel_set_error_state(channel, ret);
	}

	ret = cfg_periph_config(dev, i2s_cfg);
	/* disable channels in case of invalid configuration*/
	if (ret != 0) {
		//channel->drop(channel);
		//channel->stop(channel);
		//dev_data->sample_rate = 0;
		LOG_ERR("Can't configure peripheral");
		return channel_set_error_state(channel, ret);
	}


//tmp
	if (i2s->size != 0 && i2s->size != dev_data->block_size / 4) {
		LOG_ERR("[%s]Invalid size (given: %u, expected %u)", __func__, dev_data->block_size / 4, i2s->size);
		return -EINVAL;
	}
	if (dir == I2S_DIR_RX) {
		i2s->buffers.p_rx_buffer = NULL;
		i2s->channel_rx = channel;

	}
	if (dir == I2S_DIR_TX) {
		i2s->buffers.p_tx_buffer = NULL;
		i2s->channel_tx = channel;
	}
	i2s->size = dev_data->block_size / 4;
	channel->mem_slab = i2s_cfg->mem_slab;
	channel->timeout = i2s_cfg->timeout;
	ret = channel_change_state(channel, I2S_STATE_READY);
	if (ret != 0) {
		return channel_set_error_state(channel, ret);
	}
	nrfx_err_t status;
	if (i2s->get_state() == I2S_IF_NOT_READY) {
		status = nrfx_i2s_init(&dev_data->nrfx_driver_config,
				       i2s->handler);
		if (status != NRFX_SUCCESS) {
			if (status == NRFX_ERROR_INVALID_STATE) {
				LOG_ERR("Invalid state");
				return channel_set_error_state(channel,
							       -EINVAL);
			}
			else if (status == NRFX_ERROR_INVALID_PARAM) {
				LOG_ERR("Invalid param");
				return channel_set_error_state(channel,
						   	       -EINVAL);
			}
			else {
				LOG_ERR("Unknown error, status = %d", status);
				return channel_set_error_state(channel,
							       -ENOTSUP);
			}
		}
		ret = i2s->set_state(I2S_IF_READY);
		if (ret != 0) {
			LOG_ERR("Can't change state");
			return channel_set_error_state(channel, -ENOTSUP);
		}
	}
	else if(i2s->get_state() != I2S_IF_READY) {
		LOG_ERR("Invalid state during initialization");
		return channel_set_error_state(channel, -ENOTSUP);
	}
	return ret;
}


static struct i2s_config *i2s_nrfx_config_get(struct device *dev,
					enum i2s_dir dir)
{
	struct channel_str *dir_config;
	int ret;

	ret = i2s_nrfx_channel_get(dir, DEV_DATA(dev), &dir_config);
	if (ret != 0) {
		return NULL;
	}

	return &dir_config->api_config_copy;
}


static int i2s_nrfx_read(struct device *dev, void **mem_block, size_t *size)
{
	struct channel_str *ch_rx = get_interface()->channel_rx;
	int ret;

	if (ch_rx->current_state == I2S_STATE_NOT_READY || ch_rx->current_state == I2S_STATE_ERROR) {
		LOG_ERR("invalid state");
		return -EIO;
	}
	/*if (queue_is_empty(&ch_rx->mem_block_queue) == true) {
		return -EIO;
	}*/
	ret = k_sem_take(&ch_rx->sem, ch_rx->timeout * 10);
	if (ret < 0) {
		LOG_ERR("Can't take semaphore (code=%d)", ret);
		return ret;
	}
	LOG_INF1("\r\n[%s]TAKE (%u / %u)\r\n", __func__, ch_rx->sem.count, ch_rx->sem.limit);
	ret = queue_fetch(&ch_rx->mem_block_queue, mem_block, size);
	if (ret < 0) {
		return -EIO;
	}

	return 0;
}

static int i2s_nrfx_write(struct device *dev, void *mem_block, size_t size)
{
	struct channel_str *ch_tx = get_interface()->channel_tx;
	int ret;

	if (ch_tx->current_state != I2S_STATE_READY && ch_tx->current_state != I2S_STATE_RUNNING) {
		return -EIO;
	}

	ret = k_sem_take(&ch_tx->sem, ch_tx->timeout * 2);
	if (ret < 0) {
		NRFX_I2S_REPORT_ERROR(SEM_UNAVAILABLE);
		return ret;
	}
	LOG_MEM("\r\n[%s]TAKE (%u / %u)\r\n", __func__, ch_tx->sem.count, ch_tx->sem.limit);
	ret = queue_add(&ch_tx->mem_block_queue, mem_block, size / 4);
	return ret;
}


static int i2s_nrfx_trigger(struct device *dev, enum i2s_dir dir,
			    enum i2s_trigger_cmd cmd)
{
	struct i2s_nrfx_data *const dev_data = DEV_DATA(dev);
	const struct i2s_nrfx_config *const dev_const_cfg = DEV_CFG(dev);
	struct channel_str *channel;
	int ret;

	ret = i2s_nrfx_channel_get(dir, dev_data, &channel);
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
		ret = channel->start();
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
		return channel->stop();
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
static int channel_change_state(struct channel_str *channel,
				 enum i2s_state new_state)
{
	bool change_forbidden = false;
	enum i2s_state old_state = channel->current_state;
	LOG_INF1("[%s] %u->%u\r\n", __func__, old_state, new_state);
	switch (new_state) {
	case I2S_STATE_NOT_READY:
		if (1) /*(old_state != I2S_STATE_NOT_READY && old_state != I2S_STATE_ERROR)*/ {
			change_forbidden = true;
		}
		break;
	case I2S_STATE_READY:
		if (old_state == I2S_STATE_READY) {
			change_forbidden = true;
		}
		break;
	case I2S_STATE_RUNNING:
		if (old_state == I2S_STATE_RUNNING) {
			change_forbidden = true;
		}
		break;
	case I2S_STATE_STOPPING:
		if (old_state != I2S_STATE_RUNNING /*&& old_state != I2S_STATE_STOPPING*/) {
			change_forbidden = true;
		}
		break;
	case I2S_STATE_ERROR:
		if (old_state == I2S_STATE_NOT_READY) {
			change_forbidden = true;
		}
		break;
	default:
		LOG_ERR("Invalid state");
		return -EINVAL;
	}

	if (change_forbidden) {
		LOG_ERR("Can't switch from state %u to %u", old_state, new_state);
		return -EIO;
	}
	channel->current_state = new_state;
	return 0;
}

static int channel_tx_start()
{
	LOG_INF1("\r\n[%s(%u)]\r\n", __func__, (u32_t)k_uptime_get());
	struct i2s_nrfx_interface *i2s = get_interface();
	int ret;
	unsigned int key;
	struct channel_str * const ch_tx = i2s->channel_tx;

	ret = channel_change_state(ch_tx, I2S_STATE_RUNNING);
	if (ret != 0) {
		return ret;
	}

	key = irq_lock();
	if (i2s->get_state() == I2S_IF_RUNNING) {
		LOG_INF1("\r\n[%s]I2S_IF_RUNNING\r\n", __func__);
		ret = i2s->restart();
		irq_unlock(key);
		if (ret != 0) {
			LOG_ERR("[%s] Can't restart interface", __func__);
			return ret;
		}
	}
	else if (i2s->get_state() == I2S_IF_READY) {
		LOG_INF1("\r\n[%s]I2S_IF_READY\r\n", __func__);
		ret = ch_tx->get_data(ch_tx, (u32_t**)&i2s->buffers.p_tx_buffer,
				      &i2s->size);
		if (ret != 0) {
			irq_unlock(key);
			LOG_ERR("[%s]Can't get data", __func__);
			return ret;
		}
		LOG_INF1("\r\n[%s]STARTING1\r\n", __func__);
		ret = i2s->start();
		LOG_INF1("\r\n[%s]STARTING\r\n", __func__);
		irq_unlock(key);
		if (ret != 0) {
			LOG_ERR("[%s] Can't start interface", __func__);
			return ret;
		}
	}
	else {
		irq_unlock(key);
		LOG_ERR("[%s]Invalid interface state (%u)", __func__, i2s->get_state());
		return -EIO;
	}
	return 0;
}

static int channel_rx_start()
{
	LOG_INF1("\r\n[%s(%u)]\r\n", __func__, (u32_t)k_uptime_get());
	struct i2s_nrfx_interface *i2s = get_interface();
	int ret;
	unsigned int key;
	struct channel_str * const ch_rx = i2s->channel_rx;
#pragma message "czy tu zmiana stanu kanalu? czy pozniej?"
	ret = channel_change_state(ch_rx, I2S_STATE_RUNNING);
	if (ret != 0) {
		return ret;
	}

	key = irq_lock();
	if (i2s->get_state() == I2S_IF_RUNNING) {
		LOG_INF1("\r\n[%s]I2S_IF_RUNNING\r\n", __func__);
		ret = i2s->restart();
		irq_unlock(key);
		if (ret != 0) {
			LOG_ERR("[%s] Can't restart interface", __func__);
			return ret;
		}
	}
	else if (i2s->get_state() == I2S_IF_READY) {
		LOG_INF1("\r\n[%s]I2S_IF_READY\r\n", __func__);
		ret = k_mem_slab_alloc(ch_rx->mem_slab,
				       (void**)&i2s->buffers.p_rx_buffer,
				       K_NO_WAIT);
		if (ret != 0) {
			irq_unlock(key);
			LOG_ERR("[%s]Can't get data", __func__);
			return ret;
		}
		ret = i2s->start();
		irq_unlock(key);
		if (ret != 0) {
			LOG_ERR("[%s] Can't start interface", __func__);
			return ret;
		}
	}
	else {
		irq_unlock(key);
		LOG_ERR("[%s]Invalid interface state (%u)", __func__, i2s->get_state());
		return -EIO;
	}
	return 0;

}

static int channel_tx_stop(void)
{
	LOG_INF1("\r\n[%s(%u)]\r\n", __func__, (u32_t)k_uptime_get());
	struct i2s_nrfx_interface *i2s = get_interface();
	int ret;
	struct channel_str * const ch_rx = i2s->channel_rx;
	struct channel_str * const ch_tx = i2s->channel_tx;

	ret = channel_change_state(ch_tx, I2S_STATE_STOPPING);
	if (ret != 0) {
		return ret;
	}
	if (ch_rx->current_state == I2S_STATE_RUNNING) {
		ret = i2s->restart();
		if (ret != 0) {
			return ret;
		}
	}
	else {
		ret = i2s->stop();
		if (ret != 0) {
			return ret;
		}
	}
	return 0;
}

static int channel_rx_stop(void)
{
	LOG_INF1("\r\n[%s(%u)]\r\n", __func__, (u32_t)k_uptime_get());
	struct i2s_nrfx_interface *i2s = get_interface();
	int ret;
	struct channel_str * const ch_rx = i2s->channel_rx;
	struct channel_str * const ch_tx = i2s->channel_tx;

	ret = channel_change_state(ch_rx, I2S_STATE_STOPPING);

	if (ret != 0) {
		return ret;
	}
	if (ch_tx->current_state == I2S_STATE_RUNNING) {
		ret = i2s->restart();
		if (ret != 0) {
			return ret;
		}
	}
	else {
		ret = i2s->stop();
		if (ret != 0) {
			return ret;
		}
	}
	return 0;
}

static bool channel_check_empty(struct channel_str *channel)
{
	return queue_is_empty(&channel->mem_block_queue);
}


static int channel_tx_get_data(struct channel_str *config, u32_t **buf,
			       size_t *block_size)
{
	struct channel_str *ch_tx = get_interface()->channel_tx;
	int ret;

	ret = queue_fetch(&ch_tx->mem_block_queue, (void **)buf, block_size);
	if (ret < 0) {
		return ret;
	}
	k_sem_give(&ch_tx->sem);
	LOG_MEM("[%s]GIVE (%u / %u) -> ", __func__, ch_tx->sem.count, ch_tx->sem.limit);
	return 0;
}


static void channel_tx_callback(nrfx_i2s_buffers_t const *p_released,
				u32_t status, nrfx_i2s_buffers_t *p_new_buffers)
{
	LOG_INF1("[%s(%u)] -> ", __func__, (u32_t)k_uptime_get());
	struct i2s_nrfx_interface *i2s = get_interface();
	struct channel_str *ch_tx = i2s->channel_tx;
	size_t mem_block_size;
	u32_t *mem_block;
	int ret;
	if (p_released->p_tx_buffer != NULL) {
		k_mem_slab_free(ch_tx->mem_slab,
				(void **)&p_released->p_tx_buffer);
	}
	if (status == 0) {
		LOG_INF1("[%s] STATUS IS ZERO -> ", __func__);
		return;
	}
	if (ch_tx->current_state == I2S_STATE_STOPPING) {
		ret = channel_change_state(ch_tx, I2S_STATE_READY);
		if (ret != 0) {
			nrfx_i2s_stop();
			NRFX_I2S_REPORT_ERROR(GET_DATA);
			return;
		}
		while (ch_tx->get_data(ch_tx, &mem_block, &mem_block_size) == 0) {
			k_mem_slab_free(ch_tx->mem_slab, (void**)&mem_block);
		}
		for (unsigned int sem_cnt = ch_tx->sem.count;
		     sem_cnt < ch_tx->sem.limit; sem_cnt ++) {
			//LOG_INF1("[%s]GIVE (%u / %u)\r\n", __func__, ch_tx->sem.count, ch_tx->sem.limit);
			k_sem_give(&ch_tx->sem);
		}
		k_mem_slab_free(ch_tx->mem_slab, (void**)&i2s->buffers.p_tx_buffer);
		i2s->buffers.p_tx_buffer = NULL;
		return;
	}
	else if (i2s->get_state() == I2S_IF_RESTARTING) {
		return;
	}
	ret = ch_tx->get_data(ch_tx, &mem_block, &mem_block_size);
	if (ret != 0) {
		LOG_ERR("\r\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![%s] Queue fetching error\r\n", __func__);
		ch_tx->stop();
		NRFX_I2S_REPORT_ERROR(GET_DATA);
		return;
	}

	/* continue transmission */
	p_new_buffers->p_tx_buffer = mem_block;
	u16_t* d = (u16_t*)p_new_buffers->p_tx_buffer;
	LOG_INF1("[%s](p_released = %p, p_new_buffers = %p) -> ", __func__, p_released->p_tx_buffer, p_new_buffers->p_tx_buffer);
	LOG_INF1("[%04X][%04X][%04X]", d[0], d[1], d[2]);
	if (channel_check_empty(ch_tx) == true) {
		LOG_ERR("\r\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![%s] Underrun[data : %04X %04X\r\n", __func__, p_released->p_tx_buffer[0], p_released->p_tx_buffer[1]);
		ch_tx->stop();
		NRFX_I2S_REPORT_ERROR(UNDERRUN);
		return;
	}
	return;
}

static void channel_rx_callback(nrfx_i2s_buffers_t const *p_released,
				u32_t status, nrfx_i2s_buffers_t *p_new_buffers)
{
	LOG_INF1("[%s(%u)] -> ", __func__, (u32_t)k_uptime_get());
	struct i2s_nrfx_interface *i2s = get_interface();
	struct channel_str *ch_rx = i2s->channel_rx;
	int ret;
	if (p_released->p_rx_buffer != NULL) {
		ret = queue_add(&ch_rx->mem_block_queue,
				(void *)p_released->p_rx_buffer, i2s->size);
		if (ret < 0) {
			return;
		}
		k_sem_give(&ch_rx->sem);
		LOG_INF1("[%s]GIVE (%u / %u) -> ", __func__, i2s->channel_rx->sem.count, i2s->channel_rx->sem.limit);
	}
	if (status == 0) {
		LOG_INF1("[%s] STATUS IS ZERO -> ", __func__);
		return;
	}
	if (ch_rx->current_state == I2S_STATE_STOPPING) {

		ret = channel_change_state(ch_rx, I2S_STATE_READY);
		if (ret != 0) {
			nrfx_i2s_stop();
			NRFX_I2S_REPORT_ERROR(GET_DATA);
		}

		if (i2s->buffers.p_rx_buffer != NULL) {
			k_mem_slab_free(ch_rx->mem_slab,
					(void**)&i2s->buffers.p_rx_buffer);
		}
		else {
			LOG_INF1("--------------------------------------------------------------------\r\n---------------------------------------------------------\r\n---------------------------------------------------------\r\n");
		}
		i2s->buffers.p_rx_buffer = NULL;
		return;
	}
	else if (i2s->get_state() == I2S_IF_RESTARTING) {
		return;
	}
	//k_sem_reset(&direction_rx->sem); - gdzies to musi byc przy stopie
	ret = k_mem_slab_alloc(ch_rx->mem_slab,
			       (void **)&p_new_buffers->p_rx_buffer, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("Cannot allocate memory (code: %d)", ret);
		return;
	}


}


static int channel_rx_get_data(struct channel_str *direction_rx, u32_t **buf,
		size_t *block_size)
{

	return 0;
}

static int i2s_nrfx_channel_get(enum i2s_dir dir,
			    struct i2s_nrfx_data *const dev_data,
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

static int i2s_nrfx_initialize(struct device *dev)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	const struct i2s_nrfx_config *const dev_const_cfg = DEV_CFG(dev);
	struct i2s_nrfx_data *const dev_data = DEV_DATA(dev);
	k_sem_init(&dev_data->channel_rx.sem, 0,
		   CONFIG_NRFX_I2S_RX_BLOCK_COUNT);
	k_sem_init(&dev_data->channel_tx.sem, CONFIG_NRFX_I2S_TX_BLOCK_COUNT,
		   CONFIG_NRFX_I2S_TX_BLOCK_COUNT);
	i2s->set_state(I2S_IF_NOT_READY);
	dev_const_cfg->instance_init(dev);
	return 0;
}

static void isr(void *arg)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	u32_t base = 0x40025000;
	u32_t *ev_txupd = base + 0x114;
	u32_t *ev_rxupd = base + 0x104;
	u32_t *ev_stopped = base + 0x108;
	u32_t *inten = base + 0x300;
	u32_t *intenset = base + 0x304;
	u32_t *intenclr = base + 0x308;
	LOG_INF1("t:%u r:%u s:%u", *ev_txupd, *ev_rxupd, *ev_stopped);
	/* pass the interrupt to nrfx */
	nrfx_i2s_irq_handler();
	if (i2s->get_state() == I2S_IF_STARTING) {
		LOG_INF1("\r\n[%s] starting %p %p", __func__, i2s->buffers.p_tx_buffer, i2s->buffers.p_rx_buffer);
		if (i2s->set_state(I2S_IF_RUNNING) != 0) {
			LOG_ERR("[%s] Error while changing interface state", __func__);
			return;
		}
		nrfx_err_t status = nrfx_i2s_start(&i2s->buffers, i2s->size, 0);
		if (status != NRFX_SUCCESS) {
			//i2s->error_service();
			LOG_ERR("Error %08x while starting peripheral", status);
		}
		LOG_INF1("\r\n%04X %04X %04X\r\n", *inten, *intenset, *intenclr);
		return;
	}
}


#ifdef CONFIG_NRFX_I2S
struct queue_item rx_1_ring_buf[CONFIG_NRFX_I2S_RX_BLOCK_COUNT + 1];
struct queue_item tx_1_ring_buf[CONFIG_NRFX_I2S_TX_BLOCK_COUNT + 1];
static void setup_instance_0(struct device *dev)
{
	struct i2s_nrfx_data *const dev_data = DEV_DATA(dev);

	queue_init(&dev_data->channel_tx.mem_block_queue,
		   CONFIG_NRFX_I2S_TX_BLOCK_COUNT + 1, &tx_1_ring_buf[0]);
	queue_init(&dev_data->channel_rx.mem_block_queue,
		   CONFIG_NRFX_I2S_RX_BLOCK_COUNT + 1, &rx_1_ring_buf[0]);

	IRQ_CONNECT(DT_NORDIC_NRF_I2S_DT_I2S_0_IRQ,
		    DT_NORDIC_NRF_I2S_DT_I2S_0_IRQ_PRIORITY, isr, 0, 0);
	irq_enable(DT_NORDIC_NRF_I2S_DT_I2S_0_IRQ);
}

static const struct i2s_nrfx_config zephyr_i2s_cfg_0 = {
	.sck_pin = DT_NORDIC_NRF_I2S_DT_I2S_0_SCK_PIN,
	.lrck_pin =  DT_NORDIC_NRF_I2S_DT_I2S_0_LRCK_PIN,
	.mck_pin = DT_NORDIC_NRF_I2S_DT_I2S_0_MCK_PIN,
	.sdout_pin = DT_NORDIC_NRF_I2S_DT_I2S_0_SDOUT_PIN,
	.sdin_pin = DT_NORDIC_NRF_I2S_DT_I2S_0_SDIN_PIN,
	.instance_init = setup_instance_0,
};

struct i2s_nrfx_data zephyr_i2s_data_0 =
{
	.channel_tx =
	{
		.start = channel_tx_start,
		.stop = channel_tx_stop,
		.data_handler = channel_tx_callback,
		.get_data = channel_tx_get_data,
	},
	.channel_rx =
	{
		.start = channel_rx_start,
		.stop = channel_rx_stop,
		.data_handler = channel_rx_callback,
		.get_data = channel_rx_get_data,
	},
};

static struct i2s_nrfx_interface interface =
{
	.state = I2S_IF_NOT_READY,
	.size = 0,
	.buffers = NULL,
	.channel_tx = NULL,
	.channel_rx = NULL,
	.error_service = interface_error_service,
	.set_state = interface_set_state,
	.get_state = interface_get_state,
	.start = interface_start,
	.stop = interface_stop,
	.restart = interface_restart,
	.handler = interface_handler,
};

static inline struct i2s_nrfx_interface * get_interface(void)
{
	return &interface;
}

static const struct i2s_driver_api i2s_nrf_driver_api = {
	.configure = i2s_nrfx_api_configure,
	.read = i2s_nrfx_read,
	.write = i2s_nrfx_write,
	.trigger = i2s_nrfx_trigger,
	.config_get = i2s_nrfx_config_get,
};



DEVICE_AND_API_INIT(i2s_0, DT_NORDIC_NRF_I2S_DT_I2S_0_LABEL, &i2s_nrfx_initialize,
		    &zephyr_i2s_data_0, &zephyr_i2s_cfg_0, POST_KERNEL,
		    CONFIG_I2S_INIT_PRIORITY, &i2s_nrf_driver_api);


#endif /* CONFIG_I2S_1 */
