/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Nordic Semiconductor nRF I2S
 */


#include <string.h>
#include <stdlib.h>
#include <dma.h>
#include <i2s.h>
#include <soc.h>
#include <nrfx.h>
#include <nrfx_i2s.h>
#include "i2s_nrfx.h"
#define LOG_DOMAIN "i2s_nrfx"
#include <logging/log.h>
#define LOG_LEVEL _LOG_LEVEL_DBG

LOG_MODULE_REGISTER(i2s_nrfx);

#define LOG_ERROR	LOG_ERR("\r\n[%s] : %u\r\n", __func__, __LINE__)
#define RING_BUF_INC(idx, limit) {idx = (++idx < limit) ? idx : 0; }
#define DEV_CFG(dev) \
	(const struct i2s_nrfx_config *const)((dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct i2s_nrfx_data *const)(dev)->driver_data)

enum i2s_if_state {
	I2S_IF_NOT_READY = 0,
	I2S_IF_READY,
	I2S_IF_RESTARTING,
	I2S_IF_RUNNING,
	I2S_IF_STOPPING,
	I2S_IF_NEEDS_RESTART,
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
	struct queue mem_block_queue;
	enum i2s_trigger_cmd last_trigger_cmd;
	int (*start)(void);
	int (*stop)(void);
	int (*drop)(void);
	int (*drain)(void);
	void (*data_handler)(nrfx_i2s_buffers_t const *p_released, u32_t status,
			     nrfx_i2s_buffers_t *p_new_buffers);
	int (*get_data)(u32_t **buf, size_t *block_size);
	void (*mem_clear)(void const *first_block);
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
	void (*handler)(nrfx_i2s_buffers_t const *p_released, u32_t status);
};

static int i2s_nrfx_channel_get(enum i2s_dir dir,
			    struct i2s_nrfx_data *const dev_data,
			    struct channel_str **channel);

static int channel_change_state(struct channel_str *channel,
				 enum i2s_state new_state);

static inline struct i2s_nrfx_interface *get_interface(void);

static inline int channel_set_error_state(struct channel_str *channel,
					   int err_code);

static void channel_error_service(struct channel_str *channel);

static inline bool next_buffers_needed(u32_t status)
{
	return (status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED)
			== NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED;
}

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
		return -ENOMEM;
	}

	queue->queue_items[queue->write_idx].data = data;
	queue->queue_items[queue->write_idx].size = size;
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

	RING_BUF_INC(queue->read_idx, queue->len);
	return 0;
}

/*
 * Interface service functions
 */

static void interface_error_service(void)
{
	struct i2s_nrfx_interface *i2s = get_interface();

	i2s->set_state(I2S_IF_ERROR);
	LOG_ERROR;
	nrfx_i2s_stop();
}

static int interface_set_state(enum i2s_if_state new_state)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	bool change_forbidden = false;

	switch (new_state) {
	case I2S_IF_STOPPING:
		if (i2s->state != I2S_IF_RUNNING &&
		    i2s->state != I2S_IF_NEEDS_RESTART) {
			change_forbidden = true;
		}
		break;
	case I2S_IF_NEEDS_RESTART:
		if (i2s->state != I2S_IF_RUNNING) {
			change_forbidden = true;
		}
		break;
	case I2S_IF_RUNNING:
		if (i2s->state != I2S_IF_RESTARTING &&
		    i2s->state != I2S_IF_READY) {
			change_forbidden = true;
		}
		break;
	case I2S_IF_READY:
		if (i2s->state != I2S_IF_STOPPING &&
		    i2s->state != I2S_IF_NOT_READY &&
		    i2s->state != I2S_IF_ERROR) {
			change_forbidden = true;
		}
		break;
	case I2S_IF_RESTARTING:
		if (i2s->state != I2S_IF_NEEDS_RESTART) {
			change_forbidden = true;
		}
		break;
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

	return i2s->set_state(I2S_IF_NEEDS_RESTART);
}

static int interface_stop(void)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	int ret;

	ret = i2s->set_state(I2S_IF_STOPPING);
	if (ret < 0) {
		i2s->error_service();
		return ret;
	}
	return 0;
}

static int interface_stop_restart(enum i2s_state other_channel_state)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	int ret;

	if (other_channel_state == I2S_STATE_RUNNING) {
		ret = i2s->restart();
		if (ret != 0) {
			return ret;
		}
	} else {
		ret = i2s->stop();
		if (ret != 0) {
			i2s->error_service();
			return ret;
		}
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
	nrfx_err_t status = nrfx_i2s_start(&i2s->buffers,
					   i2s->size / 4, 0);

	if (status != NRFX_SUCCESS) {
		i2s->error_service();
		ret = -EIO;
	}

	return ret;
}

static void interface_handler(nrfx_i2s_buffers_t const *p_released,
				    u32_t status)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	struct channel_str *rx_str = i2s->channel_rx;
	struct channel_str *tx_str = i2s->channel_tx;
	nrfx_i2s_buffers_t p_new_buffers;

	p_new_buffers.p_rx_buffer = NULL;
	p_new_buffers.p_tx_buffer = NULL;
	if (rx_str != NULL && rx_str->current_state != I2S_STATE_READY) {
		rx_str->data_handler(p_released, status, &p_new_buffers);
	}
	if (tx_str != NULL && tx_str->current_state != I2S_STATE_READY) {
		tx_str->data_handler(p_released, status, &p_new_buffers);
	}
	if (next_buffers_needed(status)) {
		if (i2s->get_state() == I2S_IF_NEEDS_RESTART ||
		    i2s->get_state() == I2S_IF_STOPPING) {
			nrfx_i2s_stop();
			return;
		} else if (nrfx_i2s_next_buffers_set(&p_new_buffers) !=
			   NRFX_SUCCESS) {
			LOG_ERROR;
			nrfx_i2s_stop();
			i2s->error_service();
			return;

		}
		if ((p_new_buffers.p_rx_buffer == NULL)
		    && (p_new_buffers.p_tx_buffer == NULL)) {
			nrfx_i2s_stop();
			return;
		}
		i2s->buffers = p_new_buffers;
	} else {
		if (i2s->get_state() == I2S_IF_NEEDS_RESTART) {
			if (i2s->set_state(I2S_IF_RESTARTING) != 0) {
				LOG_ERROR;
				i2s->error_service();
			}
		} else if (i2s->get_state() == I2S_IF_STOPPING) {
			if (i2s->set_state(I2S_IF_READY)) {
				LOG_ERROR;
				i2s->error_service();
			}
		} else if (rx_str->current_state != I2S_STATE_RUNNING &&
			 tx_str->current_state != I2S_STATE_RUNNING) {
			if (i2s->get_state() == I2S_IF_RUNNING) {
				i2s->stop();
			}
		}
	}
}

/*
 * configuration functions
 */

static void cfg_reinit(struct device *dev)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	struct channel_str *ch_tx = i2s->channel_tx;
	struct channel_str *ch_rx = i2s->channel_rx;

	nrfx_i2s_stop();
	i2s->state = I2S_IF_READY;
	ch_tx->current_state = I2S_STATE_READY;
	ch_rx->current_state = I2S_STATE_READY;
}

static inline nrf_i2s_mck_t cfg_get_divider(
		struct i2s_clk_settings_t const *clk_set, u8_t word_size)
{
	u32_t sub_idx = (word_size >> 3) - 1;

	return clk_set->divider[sub_idx];
}

static inline nrf_i2s_ratio_t cfg_get_ratio(
		struct i2s_clk_settings_t const *clk_set, u8_t word_size)
{
	u32_t sub_idx = (word_size >> 3) - 1;

	return clk_set->ratio[sub_idx];
}

static void cfg_match_clock_settings(nrfx_i2s_config_t *config,
			       struct i2s_config const *i2s_cfg)
{
	const struct i2s_clk_settings_t i2s_clock_settings[] =
			NRFX_I2S_AVAILABLE_CLOCK_SETTINGS;
	u32_t des_s_r = (s32_t)i2s_cfg->frame_clk_freq;
	u8_t nb_of_settings_array_elems = ARRAY_SIZE(i2s_clock_settings);
	struct i2s_clk_settings_t const *chosen_settings =
		&i2s_clock_settings[nb_of_settings_array_elems - 1];

	for (u8_t i = 1; i < nb_of_settings_array_elems; ++i) {
		if (des_s_r < i2s_clock_settings[i].frequency) {
			u32_t diff_h =
				i2s_clock_settings[i].frequency - des_s_r;
			u32_t diff_l =
				abs((s32_t)des_s_r -
				(s32_t)i2s_clock_settings[i - 1].frequency);
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
	nrfx_i2s_config_t drv_cfg;

	if (i2s_cfg == NULL) {
		LOG_ERROR;
		interface_error_service();
		return -EINVAL;
	}

	if (dev_data == NULL) {
		LOG_ERROR;
		interface_error_service();
		return -EINVAL;
	}

	if (i2s_cfg->mem_slab == NULL) {
		LOG_ERROR;
		interface_error_service();
		return -EINVAL;
	}
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
			LOG_ERROR;
			interface_error_service();
			return -EINVAL;
		}
	}
	if (i2s_cfg->frame_clk_freq == 0) {
		LOG_ERROR;
		interface_error_service();
		return -EINVAL;
	}
	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
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
		LOG_ERROR;
		interface_error_service();
		return -EINVAL;
	}
	if ((i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE) ||
	    (i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE)) {
		drv_cfg.mode = NRF_I2S_MODE_SLAVE;
		LOG_ERROR;
		interface_error_service();
		return -ENOTSUP;
	}
	drv_cfg.mode = NRF_I2S_MODE_MASTER;
	switch (i2s_cfg->channels) {
	case 2:
		drv_cfg.channels = NRF_I2S_CHANNELS_STEREO;
		break;
	case 1:
		LOG_ERROR;
		interface_error_service();
		return -ENOTSUP;
	default:
		LOG_ERROR;
		interface_error_service();
		return -EINVAL;
	}
	if ((dev_data->block_size != 0 &&
	     dev_data->block_size != i2s_cfg->block_size) ||
	     i2s_cfg->block_size == 0) {
		LOG_ERROR;
		interface_error_service();
		return -EINVAL;
	}
	dev_data->block_size = i2s_cfg->block_size;
	cfg_match_clock_settings(&drv_cfg, i2s_cfg);
	memcpy(&dev_data->nrfx_driver_config, &drv_cfg,
	       sizeof(nrfx_i2s_config_t));
	return 0;
}

/*
 *
 * API functions
 *
 */

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

static int i2s_nrfx_api_configure(struct device *dev, enum i2s_dir dir,
			   struct i2s_config *i2s_cfg)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	struct i2s_nrfx_data *const dev_data = DEV_DATA(dev);
	struct channel_str *channel;
	int ret;
	nrfx_err_t status;

	if (dev == NULL || i2s_cfg == NULL) {
		LOG_ERROR;
		i2s->error_service();
		return -EINVAL;
	}
	ret = i2s_nrfx_channel_get(dir, dev_data, &channel);
	if (ret != 0) {
		LOG_ERROR;
		i2s->error_service();
		return -EINVAL;
	}
	ret = cfg_periph_config(dev, i2s_cfg);
	/* disable channels in case of invalid configuration*/
	if (ret != 0) {
		LOG_ERROR;
		return channel_set_error_state(channel, ret);
	}
	if (i2s->size != 0 && i2s->size != dev_data->block_size) {
		LOG_ERROR;
		return channel_set_error_state(channel, -EINVAL);
	}
	if (dir == I2S_DIR_RX) {
		i2s->buffers.p_rx_buffer = NULL;
		i2s->channel_rx = channel;

	}
	if (dir == I2S_DIR_TX) {
		i2s->buffers.p_tx_buffer = NULL;
		i2s->channel_tx = channel;
	}
	i2s->size = dev_data->block_size;
	channel->mem_slab = i2s_cfg->mem_slab;
	channel->timeout = i2s_cfg->timeout;
	ret = channel_change_state(channel, I2S_STATE_READY);
	if (ret != 0) {
		LOG_ERROR;
		return channel_set_error_state(channel, ret);
	}

	if (i2s->get_state() == I2S_IF_NOT_READY) {
		status = nrfx_i2s_init(&dev_data->nrfx_driver_config,
				       i2s->handler);
		if (status != NRFX_SUCCESS) {
			if (status == NRFX_ERROR_INVALID_STATE) {
				LOG_ERROR;
				return channel_set_error_state(channel,
							       -EINVAL);
			} else if (status == NRFX_ERROR_INVALID_PARAM) {
				LOG_ERROR;
				return channel_set_error_state(channel,
							       -EINVAL);
			} else {
				LOG_ERROR;
				return channel_set_error_state(channel,
							       -ENOTSUP);
			}
		}
		ret = i2s->set_state(I2S_IF_READY);
		if (ret < 0) {
			LOG_ERROR;
			return channel_set_error_state(channel, -ENOTSUP);
		}
	} else if (i2s->get_state() != I2S_IF_READY) {
		LOG_ERROR;
		return channel_set_error_state(channel, -ENOTSUP);
	}
	return ret;
}

static int i2s_nrfx_read(struct device *dev, void **mem_block, size_t *size)
{
	struct channel_str *ch_rx = get_interface()->channel_rx;
	int ret;

	if (ch_rx->current_state == I2S_STATE_NOT_READY ||
	    ch_rx->current_state == I2S_STATE_ERROR) {
		return -EIO;
	}
	ret = k_sem_take(&ch_rx->sem, ch_rx->timeout);
	if (ret < 0) {
		return ret;
	}
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

	if (ch_tx->current_state != I2S_STATE_READY &&
	    ch_tx->current_state != I2S_STATE_RUNNING) {
		return -EIO;
	}

	ret = k_sem_take(&ch_tx->sem, ch_tx->timeout);
	if (ret < 0) {
		return ret;
	}
	ret = queue_add(&ch_tx->mem_block_queue, mem_block, size);
	if (ret < 0) {
		return ret;
	}
	return 0;
}

static int i2s_nrfx_trigger(struct device *dev, enum i2s_dir dir,
			    enum i2s_trigger_cmd cmd)
{
	struct i2s_nrfx_data *dev_data = DEV_DATA(dev);
	struct channel_str *channel;
	int ret;

	ret = i2s_nrfx_channel_get(dir, dev_data, &channel);
	if ((ret != 0) || (channel == NULL)) {
		return ret;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (channel->current_state != I2S_STATE_READY) {
			LOG_ERROR;
			return -EIO;
		}
		ret = channel->start();
		break;

	case I2S_TRIGGER_STOP:

		if (channel->current_state != I2S_STATE_RUNNING) {
			LOG_ERROR;
			return -EIO;
		}
		ret = channel->stop();
		break;

	case I2S_TRIGGER_DRAIN:
		if (channel->current_state != I2S_STATE_RUNNING) {
			LOG_ERROR;
			return -EIO;
		}
		ret = channel->drain();
		break;

	case I2S_TRIGGER_DROP:
		if (channel->current_state == I2S_STATE_NOT_READY) {
			LOG_ERROR;
			return -EIO;
		}
		ret = channel->drop();
		break;

	case I2S_TRIGGER_PREPARE:
	{
		if (channel->current_state != I2S_STATE_ERROR) {
			LOG_ERROR;
			return -EIO;
		}
		cfg_reinit(dev);
		channel->drop();
		break;
	}
	default:
		LOG_ERROR;
		return channel_set_error_state(channel, -EINVAL);
	}
	if (ret < 0) {
		LOG_ERROR;
		return channel_set_error_state(channel, ret);
	}
	channel->last_trigger_cmd = cmd;
	return 0;
}

/*
 * channel management functions
 */

static inline int channel_set_error_state(struct channel_str *channel,
					   int err_code)
{
	channel_change_state(channel, I2S_STATE_ERROR);
	return err_code;
}

static void channel_error_service(struct channel_str *channel)
{
	channel_change_state(channel, I2S_STATE_ERROR);
}

static int channel_change_state(struct channel_str *channel,
				 enum i2s_state new_state)
{
	bool change_forbidden = false;
	enum i2s_state old_state = channel->current_state;

	switch (new_state) {
	case I2S_STATE_NOT_READY:
			change_forbidden = true;
		break;
	case I2S_STATE_READY:
		if (old_state == I2S_STATE_READY) {
			change_forbidden = true;
		}
		break;
	case I2S_STATE_RUNNING:
		if (old_state != I2S_STATE_READY) {
			change_forbidden = true;
		}
		break;
	case I2S_STATE_STOPPING:
		if (old_state != I2S_STATE_RUNNING) {
			change_forbidden = true;
		}
		break;
	case I2S_STATE_ERROR:
		break;
	default:

		LOG_ERROR;
		return channel_set_error_state(channel, -EINVAL);
	}

	if (change_forbidden) {
		LOG_ERROR;
		return channel_set_error_state(channel, -EIO);
	}
	channel->current_state = new_state;
	return 0;
}

static int channel_tx_start(void)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	int ret;
	unsigned int key;
	struct channel_str * const ch_tx = i2s->channel_tx;
	size_t mem_block_size;

	ret = channel_change_state(ch_tx, I2S_STATE_RUNNING);

	if (ret < 0) {
		return ret;
	}
	key = irq_lock();
	if (i2s->get_state() == I2S_IF_RUNNING) {
		ret = ch_tx->get_data((u32_t **)&i2s->buffers.p_tx_buffer,
				      &mem_block_size);
		if (ret < 0) {
			LOG_ERROR;
			return channel_set_error_state(ch_tx, ret);
		}
		ret = i2s->restart();
		irq_unlock(key);
		if (ret != 0) {
			LOG_ERROR;
			return channel_set_error_state(ch_tx, ret);
		}
	} else if (i2s->get_state() == I2S_IF_READY) {
		ret = ch_tx->get_data((u32_t **)&i2s->buffers.p_tx_buffer,
				      &mem_block_size);
		if (ret < 0) {
			irq_unlock(key);
			LOG_ERROR;
			return channel_set_error_state(ch_tx, ret);
		}
		ret = i2s->start();
		irq_unlock(key);
		if (ret < 0) {
			LOG_ERROR;
			return channel_set_error_state(ch_tx, ret);
		}
	} else {
		irq_unlock(key);
		LOG_ERROR;
		return channel_set_error_state(ch_tx, -EIO);
	}
	return 0;
}

static int channel_rx_start(void)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	int ret;
	unsigned int key;
	struct channel_str * const ch_rx = i2s->channel_rx;

	ret = channel_change_state(ch_rx, I2S_STATE_RUNNING);
	if (ret != 0) {
		LOG_ERROR;
		return channel_set_error_state(ch_rx, ret);
	}
	key = irq_lock();
	if (i2s->get_state() == I2S_IF_RUNNING) {
		ret = k_mem_slab_alloc(ch_rx->mem_slab,
				       (void **)&i2s->buffers.p_rx_buffer,
				       K_NO_WAIT);
		if (ret != 0) {
			irq_unlock(key);
			LOG_ERROR;
			return channel_set_error_state(ch_rx, ret);
		}
		ret = i2s->restart();
		irq_unlock(key);
		if (ret != 0) {
			LOG_ERROR;
			return channel_set_error_state(ch_rx, ret);
		}
	} else if (i2s->get_state() == I2S_IF_READY) {
		ret = k_mem_slab_alloc(ch_rx->mem_slab,
				       (void **)&i2s->buffers.p_rx_buffer,
				       K_NO_WAIT);
		if (ret != 0) {
			irq_unlock(key);
			LOG_ERROR;
			return channel_set_error_state(ch_rx, ret);
		}
		ret = i2s->start();
		irq_unlock(key);
		if (ret != 0) {
			LOG_ERROR;
			return channel_set_error_state(ch_rx, ret);
		}
	} else {
		irq_unlock(key);
		LOG_ERROR;
		return channel_set_error_state(ch_rx, -EIO);
	}
	return 0;

}
static void channel_tx_mem_clear(void const *first_block)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	struct channel_str * const ch_tx = i2s->channel_tx;
	void *mem_block = (void *)first_block;
	size_t mem_block_size;

	if (first_block == NULL) {
		if (ch_tx->get_data((u32_t **)&mem_block,
		    &mem_block_size) != 0) {
			return;
		}
	}
	do {
		k_mem_slab_free(ch_tx->mem_slab,
			(void **)&mem_block);
	} while (ch_tx->get_data((u32_t **)&mem_block, &mem_block_size) == 0);
	while (ch_tx->sem.count < ch_tx->sem.limit) {
		k_sem_give(&ch_tx->sem);
	}
}

static int channel_tx_stop(void)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	int ret;
	struct channel_str * const ch_rx = i2s->channel_rx;
	struct channel_str * const ch_tx = i2s->channel_tx;

	ret = channel_change_state(ch_tx, I2S_STATE_STOPPING);
	if (ret < 0) {
		return ret;
	}
	return interface_stop_restart(ch_rx->current_state);
}

static int channel_tx_drain(void)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	struct channel_str * const ch_tx = i2s->channel_tx;

	return channel_change_state(ch_tx, I2S_STATE_STOPPING);
}

static int channel_tx_drop(void)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	int ret;
	struct channel_str * const ch_rx = i2s->channel_rx;
	struct channel_str * const ch_tx = i2s->channel_tx;

	if (ch_tx->current_state == I2S_STATE_RUNNING) {
		ret = channel_change_state(ch_tx, I2S_STATE_STOPPING);
		if (ret < 0) {
			return ret;
		}
		ret = interface_stop_restart(ch_rx->current_state);
		if (ret < 0) {
			i2s->error_service();
			return ret;
		}
	} else {
		ch_tx->mem_clear(NULL);
	}
	return 0;
}

static int channel_rx_stop(void)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	int ret;
	struct channel_str * const ch_rx = i2s->channel_rx;
	struct channel_str * const ch_tx = i2s->channel_tx;

	ret = channel_change_state(ch_rx, I2S_STATE_STOPPING);
	if (ret < 0) {
		return ret;
	}
	return interface_stop_restart(ch_tx->current_state);

}

static int channel_rx_drop(void)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	int ret;
	struct channel_str * const ch_rx = i2s->channel_rx;
	struct channel_str * const ch_tx = i2s->channel_tx;

	if (ch_tx->current_state == I2S_STATE_RUNNING) {
		ret = channel_change_state(ch_rx, I2S_STATE_STOPPING);
		if (ret < 0) {
			return ret;
		}
		ret =  interface_stop_restart(ch_tx->current_state);
		if (ret < 0) {
			i2s->error_service();
			return ret;
		}
	} else {
		ch_rx->mem_clear(NULL);
	}
	return 0;
}

static bool channel_check_empty(struct channel_str *channel)
{
	return queue_is_empty(&channel->mem_block_queue);
}

static int channel_tx_get_data(u32_t **buf, size_t *block_size)
{
	struct channel_str *ch_tx = get_interface()->channel_tx;
	int ret;

	ret = queue_fetch(&ch_tx->mem_block_queue, (void **)buf, block_size);
	if (ret < 0) {
		return ret;
	}
	k_sem_give(&ch_tx->sem);
	return 0;
}

static void channel_tx_callback(nrfx_i2s_buffers_t const *p_released,
				u32_t status, nrfx_i2s_buffers_t *p_new_buffers)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	struct channel_str *ch_tx = get_interface()->channel_tx;
	size_t mem_block_size;
	u32_t *mem_block = NULL;
	int get_data_ret = 1;

	if (ch_tx->current_state == I2S_STATE_RUNNING &&
	    i2s->get_state() == I2S_IF_NEEDS_RESTART) {
		if (p_released->p_tx_buffer != NULL) {
			if (next_buffers_needed(status)) {
				k_mem_slab_free(ch_tx->mem_slab,
				      (void **)&p_released->p_tx_buffer);
			}
		}
		get_data_ret = ch_tx->get_data(&mem_block,
					       &mem_block_size);
		if (get_data_ret == 0) {
			k_mem_slab_free(ch_tx->mem_slab,
					(void **)&mem_block);
		}
		return;
	}
	if (p_released->p_tx_buffer != NULL) {
		k_mem_slab_free(ch_tx->mem_slab,
				(void **)&p_released->p_tx_buffer);
	}
	if (next_buffers_needed(status)) {
		get_data_ret = ch_tx->get_data(&mem_block,
					       &mem_block_size);
	}
	if (ch_tx->current_state == I2S_STATE_STOPPING) {
		enum i2s_trigger_cmd ch_cmd = ch_tx->last_trigger_cmd;

		if (next_buffers_needed(status)) {
			switch (ch_cmd) {
			case I2S_TRIGGER_DROP:
				if (get_data_ret == 0) {
					ch_tx->mem_clear(mem_block);
				}
				break;
			case I2S_TRIGGER_STOP:
				if (get_data_ret == 0) {
					k_mem_slab_free(ch_tx->mem_slab,
							(void **)&mem_block);
				}
				break;
			case I2S_TRIGGER_DRAIN:
				break;
			default:
				channel_error_service(ch_tx);
				return;
			}
		} else {
			int ret = channel_change_state(ch_tx, I2S_STATE_READY);

			if (ret != 0) {
				channel_error_service(ch_tx);
				return;
			}
			if (ch_cmd == I2S_TRIGGER_DRAIN) {
				return;
			}
		}
	} else if (ch_tx->current_state == I2S_STATE_ERROR) {
		return;
	} else if (get_data_ret < 0) {
		LOG_ERROR;
		interface_error_service();
		channel_error_service(ch_tx);
		return;
	} else if (channel_check_empty(ch_tx) == true) {
		/*underrun error occurred*/
		if (get_data_ret == 0) {
			k_mem_slab_free(ch_tx->mem_slab,
					(void **)&mem_block);
		}
		LOG_ERROR;
		interface_error_service();
		channel_error_service(ch_tx);
		return;
	}
	/* continue transmission */
	p_new_buffers->p_tx_buffer = mem_block;
}

static void channel_rx_mem_clear(void const *first_block)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	struct channel_str * const ch_rx = i2s->channel_rx;
	void *mem_block;
	size_t mem_block_size;

	while (queue_fetch(&ch_rx->mem_block_queue,
			(void **)&mem_block,
			&mem_block_size) != 0) {
		if (k_sem_take(&ch_rx->sem, ch_rx->timeout * 10) < 0) {
			return;
		}
		k_mem_slab_free(ch_rx->mem_slab,
			(void **)&mem_block);
	}
}

static void channel_rx_callback(nrfx_i2s_buffers_t const *p_released,
				u32_t status, nrfx_i2s_buffers_t *p_new_buffers)
{
	struct i2s_nrfx_interface *i2s = get_interface();
	struct channel_str *ch_rx = i2s->channel_rx;
	int ret;

	if (p_released->p_rx_buffer != NULL && next_buffers_needed(status)) {
		ret = ch_rx->get_data((u32_t **)&p_released->p_rx_buffer,
				      &i2s->size);
		if (ret < 0) {
			return;
		}
	}
	if (ch_rx->current_state == I2S_STATE_STOPPING) {
		enum i2s_trigger_cmd ch_cmd = ch_rx->last_trigger_cmd;

		if (next_buffers_needed(status)) {
			switch (ch_cmd) {
			case I2S_TRIGGER_DROP:
				ch_rx->mem_clear(NULL);
				break;
			case I2S_TRIGGER_DRAIN:
			case I2S_TRIGGER_STOP:
				break;
			default:
				channel_error_service(ch_rx);
				return;
			}
		} else {
			if (p_released->p_rx_buffer) {
				ret = ch_rx->get_data(
					(u32_t **)&p_released->p_rx_buffer,
					&i2s->size);
				if (ret < 0) {
					return;
				}
			}
			int ret = channel_change_state(ch_rx,
					I2S_STATE_READY);
			if (ret < 0) {
				channel_error_service(ch_rx);
				return;
			}
		}
		i2s->buffers.p_rx_buffer = NULL;
		return;
	} else if (ch_rx->current_state == I2S_STATE_RUNNING &&
		 i2s->get_state() == I2S_IF_NEEDS_RESTART) {
		return;
	} else if (ch_rx->current_state == I2S_STATE_ERROR) {
		return;
	}
	if (next_buffers_needed(status)) {
		ret = k_mem_slab_alloc(ch_rx->mem_slab,
				       (void **)&p_new_buffers->p_rx_buffer,
				       K_NO_WAIT);
		if (ret < 0) {
			/*overrun error occurred*/
			LOG_ERROR;
			interface_error_service();
			channel_error_service(ch_rx);
			return;
		}
	}
}

static int channel_rx_get_data(u32_t **buf, size_t *block_size)
{
	struct channel_str *ch_rx = get_interface()->channel_rx;
	int ret = queue_add(&ch_rx->mem_block_queue, *buf, *block_size);

	if (ret < 0) {
		return ret;
	}
	k_sem_give(&ch_rx->sem);
	return 0;
}

static void isr(void *arg)
{
	struct i2s_nrfx_interface *i2s = get_interface();

	nrfx_i2s_irq_handler();
	if (i2s->get_state() == I2S_IF_RESTARTING) {
		int ret = i2s->start();

		if (ret < 0) {
			i2s->error_service();
		}
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
	IRQ_CONNECT(DT_INST_0_NORDIC_NRF_I2S_IRQ_0,
		    DT_INST_0_NORDIC_NRF_I2S_IRQ_0_PRIORITY, isr, 0, 0);
	irq_enable(DT_INST_0_NORDIC_NRF_I2S_IRQ_0);
}

static const struct i2s_nrfx_config channel_cfg_0 = {
	.sck_pin = DT_NORDIC_NRF_I2S_I2S_0_SCK_PIN,
	.lrck_pin =  DT_NORDIC_NRF_I2S_I2S_0_LRCK_PIN,
	.mck_pin = DT_NORDIC_NRF_I2S_I2S_0_MCK_PIN,
	.sdout_pin = DT_NORDIC_NRF_I2S_I2S_0_SDOUT_PIN,
	.sdin_pin = DT_NORDIC_NRF_I2S_I2S_0_SDIN_PIN,
	.instance_init = setup_instance_0,
};

struct i2s_nrfx_data channels_data_0 = {
	.channel_tx = {
		.current_state = I2S_STATE_NOT_READY,
		.last_trigger_cmd = I2S_TRIGGER_PREPARE,
		.start = channel_tx_start,
		.stop = channel_tx_stop,
		.drain = channel_tx_drain,
		.drop = channel_tx_drop,
		.data_handler = channel_tx_callback,
		.get_data = channel_tx_get_data,
		.mem_clear = channel_tx_mem_clear,
	},
	.channel_rx = {
		.current_state = I2S_STATE_NOT_READY,
		.last_trigger_cmd = I2S_TRIGGER_PREPARE,
		.start = channel_rx_start,
		.stop = channel_rx_stop,
		.drain = channel_rx_stop,
		.drop = channel_rx_drop,
		.data_handler = channel_rx_callback,
		.get_data = channel_rx_get_data,
		.mem_clear = channel_rx_mem_clear,
	},
};

static struct i2s_nrfx_interface interface = {
	.state = I2S_IF_NOT_READY,
	.size = 0,
	.error_service = interface_error_service,
	.set_state = interface_set_state,
	.get_state = interface_get_state,
	.start = interface_start,
	.stop = interface_stop,
	.restart = interface_restart,
	.handler = interface_handler,
};

static inline struct i2s_nrfx_interface *get_interface(void)
{
	return &interface;
}

static const struct i2s_driver_api i2s_nrf_driver_api = {
	.configure = i2s_nrfx_api_configure,
	.read = i2s_nrfx_read,
	.write = i2s_nrfx_write,
	.trigger = i2s_nrfx_trigger,
};

DEVICE_AND_API_INIT(i2s_0, DT_NORDIC_NRF_I2S_I2S_0_LABEL,
		    &i2s_nrfx_initialize, &channels_data_0, &channel_cfg_0,
		    POST_KERNEL, CONFIG_I2S_INIT_PRIORITY, &i2s_nrf_driver_api);

#endif /* CONFIG_NRFX_I2S */
