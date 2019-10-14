/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Nordic Semiconductor nRF I2S
 */


#include <stdlib.h>
#include <i2s.h>
#include <nrfx_i2s.h>
#include "i2s_nrfx.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(i2s_nrfx, CONFIG_I2S_LOG_LEVEL);

#define LOG_ERROR(msg) LOG_ERR("\r\n[%s:%u]: %s\r\n", __func__, __LINE__, msg)

#define DEV_CFG(dev) \
	(const struct i2s_nrfx_config *const)((dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct i2s_nrfx_data *const)(dev)->driver_data)

struct i2s_nrfx_interface;

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

struct dir_mng {
	int (*start)(struct i2s_nrfx_interface *i2s);
	int (*stop)(struct i2s_nrfx_interface *i2s);
	int (*drop)(struct i2s_nrfx_interface *i2s);
	int (*drain)(struct i2s_nrfx_interface *i2s);
	void (*data_handler)(struct i2s_nrfx_interface *i2s,
			     nrfx_i2s_buffers_t const *p_released,
			     u32_t status, nrfx_i2s_buffers_t *p_new_buffers);
	int (*get_data)(struct i2s_nrfx_interface *i2s, u32_t **buf,
			size_t *block_size);
	void (*mem_clear)(struct i2s_nrfx_interface *i2s,
			  void const *first_block);
};

struct channel_str {
	struct k_sem sem;
	struct k_mem_slab *mem_slab;
	s32_t timeout;
	enum i2s_state current_state;
	struct queue mem_block_queue;
	enum i2s_trigger_cmd last_trigger_cmd;
	struct dir_mng const *mng;
};

struct i2s_nrfx_interface {
	enum i2s_if_state state;
	size_t size;
	nrfx_i2s_buffers_t buffers;
	struct channel_str channel_tx;
	struct channel_str channel_rx;
};

struct i2s_nrfx_data {
	nrfx_i2s_config_t nrfx_driver_config;
	struct i2s_nrfx_interface interface;
};

static int i2s_nrfx_channel_get(enum i2s_dir dir,
			    struct i2s_nrfx_data *const dev_data,
			    struct channel_str **channel);

static int channel_change_state(struct channel_str *channel,
				 enum i2s_state new_state);

static inline struct i2s_nrfx_interface *get_interface(
				struct device *dev __attribute__((unused)));

static inline int channel_set_error_state(struct channel_str *channel,
					   int err_code);

static int interface_set_state(struct i2s_nrfx_interface *i2s,
				enum i2s_if_state new_state);

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
static inline void queue_idx_inc(u8_t *idx, u8_t limit)
{
	*idx = (++(*idx) < limit) ? (*idx) : 0;
}

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

	queue_idx_inc(&wr_idx, queue->len);
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

	queue_idx_inc(&queue->read_idx, queue->len);
	return 0;
}

/*
 * Interface service functions
 */

static void interface_error_service(struct i2s_nrfx_interface *i2s,
					const char * const err_msg)
{
	interface_set_state(i2s, I2S_IF_ERROR);
	LOG_ERROR(err_msg);
	nrfx_i2s_stop();
}

static int interface_set_state(struct i2s_nrfx_interface *i2s,
				enum i2s_if_state new_state)
{
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
		LOG_ERROR("Invalid interface state");
		return -EINVAL;
	}
	if (change_forbidden) {
		interface_error_service(i2s,
				"Failed to change interface state");
		return -EIO;
	}
	i2s->state = new_state;
	return 0;
}

static inline enum i2s_if_state interface_get_state(
					struct i2s_nrfx_interface *i2s)
{
	return i2s->state;
}

static int interface_restart(struct i2s_nrfx_interface *i2s)
{
	return interface_set_state(i2s, I2S_IF_NEEDS_RESTART);
}

static int interface_stop(struct i2s_nrfx_interface *i2s)
{
	int ret;

	ret = interface_set_state(i2s, I2S_IF_STOPPING);
	if (ret < 0) {
		interface_error_service(i2s, "Failed to stop interface");
		return ret;
	}
	return 0;
}

static int interface_stop_restart(struct i2s_nrfx_interface *i2s,
				enum i2s_state other_channel_state)
{
	int ret;

	if (other_channel_state == I2S_STATE_RUNNING) {
		ret = interface_restart(i2s);
		if (ret != 0) {
			return ret;
		}
	} else {
		ret = interface_stop(i2s);
		if (ret != 0) {
			interface_error_service(i2s,
				"Failed to restart interface");
			return ret;
		}
	}
	return 0;
}

static int interface_start(struct i2s_nrfx_interface *i2s)
{
	int ret;

	ret = interface_set_state(i2s, I2S_IF_RUNNING);
	if (ret != 0) {
		return ret;
	}

	/*nrfx_i2s_start() procedure needs buffer size in 32-bit word units*/
	nrfx_err_t status = nrfx_i2s_start(&i2s->buffers,
					   i2s->size / sizeof(u32_t), 0);

	if (status != NRFX_SUCCESS) {
		interface_error_service(i2s, "Failed to start interface");
		ret = -EIO;
	}

	return ret;
}

static void interface_handler(nrfx_i2s_buffers_t const *p_released,
				    u32_t status)
{
	struct i2s_nrfx_interface *i2s = get_interface(NULL);
	struct channel_str *rx_str = &i2s->channel_rx;
	struct channel_str *tx_str = &i2s->channel_tx;
	nrfx_i2s_buffers_t p_new_buffers;

	p_new_buffers.p_rx_buffer = NULL;
	p_new_buffers.p_tx_buffer = NULL;
	if (rx_str->current_state != I2S_STATE_READY &&
	    rx_str->current_state != I2S_STATE_NOT_READY) {
		rx_str->mng->data_handler(i2s, p_released, status,
					  &p_new_buffers);
	}
	if (tx_str->current_state != I2S_STATE_READY &&
	    tx_str->current_state != I2S_STATE_NOT_READY) {
		tx_str->mng->data_handler(i2s, p_released, status,
					  &p_new_buffers);
	}
	if (next_buffers_needed(status)) {
		if (interface_get_state(i2s) == I2S_IF_NEEDS_RESTART ||
		    interface_get_state(i2s) == I2S_IF_STOPPING) {
			nrfx_i2s_stop();
			return;
		} else if (nrfx_i2s_next_buffers_set(&p_new_buffers) !=
			   NRFX_SUCCESS) {
			nrfx_i2s_stop();
			interface_error_service(i2s, "Internal service error");
			return;

		}
		if ((p_new_buffers.p_rx_buffer == NULL)
		    && (p_new_buffers.p_tx_buffer == NULL)) {
			nrfx_i2s_stop();
			return;
		}
		i2s->buffers = p_new_buffers;
	} else {
		if (interface_get_state(i2s) == I2S_IF_NEEDS_RESTART) {
			if (interface_set_state(i2s, I2S_IF_RESTARTING) != 0) {
				interface_error_service(i2s,
						"Internal service error");
			}
		} else if (interface_get_state(i2s) == I2S_IF_STOPPING) {
			if (interface_set_state(i2s, I2S_IF_READY)) {
				interface_error_service(i2s,
						"Internal service error");
			}
		} else if (rx_str->current_state != I2S_STATE_RUNNING &&
			 tx_str->current_state != I2S_STATE_RUNNING) {
			if (interface_get_state(i2s) == I2S_IF_RUNNING) {
				interface_stop(i2s);
			}
		}
	}
}

/*
 * configuration functions
 */

static void cfg_reinit(struct device *dev)
{
	struct i2s_nrfx_interface *i2s = get_interface(dev);
	struct channel_str *ch_tx = &i2s->channel_tx;
	struct channel_str *ch_rx = &i2s->channel_rx;

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
	struct i2s_nrfx_interface *i2s = get_interface(dev);
	nrfx_i2s_config_t drv_cfg = {
		.sck_pin = dev_const_cfg->sck_pin,
		.lrck_pin = dev_const_cfg->lrck_pin,
		.mck_pin = dev_const_cfg->mck_pin,
		.sdout_pin = dev_const_cfg->sdout_pin,
		.sdin_pin = dev_const_cfg->sdin_pin,
	};

#warning zapytac Andrzeja o podwojne asercje
	assert(i2s_cfg != NULL && dev != NULL && dev_data != NULL);
	if (i2s_cfg->mem_slab == NULL) {
		interface_error_service(i2s, "Config: Invalid memory slab");
		return -EINVAL;
	}
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
			interface_error_service(i2s,
					"Config: Invalid word size");
			return -EINVAL;
		}
	}
	if (i2s_cfg->frame_clk_freq == 0) {
		interface_error_service(i2s,
				"Config: Invalid frame clock frequency value");
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
		interface_error_service(i2s, "Config: Invalid format data");
		return -EINVAL;
	}
	if ((i2s_cfg->options & I2S_OPT_FRAME_CLK_SLAVE) ||
	    (i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE)) {
		drv_cfg.mode = NRF_I2S_MODE_SLAVE;
		interface_error_service(i2s, "Config: Invalid options");
		return -ENOTSUP;
	}
	drv_cfg.mode = NRF_I2S_MODE_MASTER;
	switch (i2s_cfg->channels) {
	case 2:
		drv_cfg.channels = NRF_I2S_CHANNELS_STEREO;
		break;
	case 1:
		interface_error_service(i2s,
				"Config: Mono mode is not supported");
		return -ENOTSUP;
	default:
		interface_error_service(i2s,
				"Config: Invalid number of channels");
		return -EINVAL;
	}
	if ((i2s->size != 0 && i2s->size != i2s_cfg->block_size) ||
					i2s_cfg->block_size == 0) {
		interface_error_service(i2s, "Config: Invalid block size");
		return -EINVAL;
	}
	i2s->size = i2s_cfg->block_size;
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
	assert(dev_data != NULL);
	switch (dir) {
	case I2S_DIR_RX:
		*channel = &dev_data->interface.channel_rx;
		break;
	case I2S_DIR_TX:
		*channel = &dev_data->interface.channel_tx;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int i2s_nrfx_initialize(struct device *dev)
{
	struct i2s_nrfx_interface *i2s = get_interface(dev);
	const struct i2s_nrfx_config *const dev_const_cfg = DEV_CFG(dev);

	k_sem_init(&i2s->channel_rx.sem, 0, CONFIG_NRFX_I2S_RX_BLOCK_COUNT);
	k_sem_init(&i2s->channel_tx.sem, CONFIG_NRFX_I2S_TX_BLOCK_COUNT,
		   CONFIG_NRFX_I2S_TX_BLOCK_COUNT);
	dev_const_cfg->instance_init(dev);
	return 0;
}

static int i2s_nrfx_api_configure(struct device *dev, enum i2s_dir dir,
			   struct i2s_config *i2s_cfg)
{
	struct i2s_nrfx_interface *i2s = get_interface(dev);
	struct i2s_nrfx_data *const dev_data = DEV_DATA(dev);
	struct channel_str *channel;
	int ret;
	nrfx_err_t status;

	assert(i2s_cfg != NULL && dev != NULL && dev_data != NULL);
	ret = i2s_nrfx_channel_get(dir, dev_data, &channel);
	if (ret != 0) {
		interface_error_service(i2s,
			"Config: Invalid channel direction");
		return -EINVAL;
	}

	if (i2s_cfg->frame_clk_freq == 0) {
		if (channel->current_state == I2S_STATE_RUNNING) {
			ret = channel->mng->drop(i2s);
			if (ret < 0) {
				LOG_ERROR("Error trigger while execution");
				return channel_set_error_state(channel, ret);
			}
		} else if (channel->current_state == I2S_STATE_STOPPING) {
			return -EIO;
		}
		channel_change_state(channel, I2S_STATE_NOT_READY);
		return 0;
	}
	ret = cfg_periph_config(dev, i2s_cfg);
	/* disable channels in case of invalid configuration*/
	if (ret != 0) {
		LOG_ERROR("Config: Failed to configure peripheral");
		return channel_set_error_state(channel, ret);
	}
	if (dir == I2S_DIR_RX) {
		i2s->buffers.p_rx_buffer = NULL;
	}
	if (dir == I2S_DIR_TX) {
		i2s->buffers.p_tx_buffer = NULL;
	}
	channel->mem_slab = i2s_cfg->mem_slab;
	channel->timeout = i2s_cfg->timeout;
	ret = channel_change_state(channel, I2S_STATE_READY);
	if (ret != 0) {
		return channel_set_error_state(channel, ret);
	}

	if (interface_get_state(i2s) == I2S_IF_NOT_READY) {
		status = nrfx_i2s_init(&dev_data->nrfx_driver_config,
				       interface_handler);
		if (status != NRFX_SUCCESS) {
			if (status == NRFX_ERROR_INVALID_STATE) {
				LOG_ERROR("Config: NRFX Invalid state");
				return channel_set_error_state(channel,
							       -EINVAL);
			} else if (status == NRFX_ERROR_INVALID_PARAM) {
				LOG_ERROR("Config: NRFX Invalid parameter");
				return channel_set_error_state(channel,
							       -EINVAL);
			} else {
				LOG_ERROR("Config: NRFX General error");
				return channel_set_error_state(channel,
							       -ENOTSUP);
			}
		}
		ret = interface_set_state(i2s, I2S_IF_READY);
		if (ret < 0) {
			return channel_set_error_state(channel, -ENOTSUP);
		}
	} else if (interface_get_state(i2s) != I2S_IF_READY) {
		LOG_ERROR("Config: Interface must be in ready state");
		return channel_set_error_state(channel, -ENOTSUP);
	}
	return ret;
}

static int i2s_nrfx_read(struct device *dev, void **mem_block, size_t *size)
{
	struct channel_str *ch_rx = &get_interface(dev)->channel_rx;
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
	struct channel_str *ch_tx = &get_interface(dev)->channel_tx;
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
	struct i2s_nrfx_interface *i2s = get_interface(dev);
	struct channel_str *channel;
	int ret;

	if ((interface_get_state(i2s) != I2S_IF_READY) &&
	    (interface_get_state(i2s) != I2S_IF_RUNNING)) {
		if (cmd != I2S_TRIGGER_PREPARE) {
			LOG_INF("Wait for stable state");
			return -EIO;
		}
	}

	ret = i2s_nrfx_channel_get(dir, dev_data, &channel);
	if (ret != 0) {
		return ret;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (channel->current_state != I2S_STATE_READY) {
			LOG_ERROR("Failed to execute I2S_TRIGGER_START");
			return -EIO;
		}
		ret = channel->mng->start(i2s);
		break;

	case I2S_TRIGGER_STOP:

		if (channel->current_state != I2S_STATE_RUNNING) {
			LOG_ERROR("Failed to execute I2S_TRIGGER_STOP");
			return -EIO;
		}
		ret = channel->mng->stop(i2s);
		break;

	case I2S_TRIGGER_DRAIN:
		if (channel->current_state != I2S_STATE_RUNNING) {
			LOG_ERROR("Failed to execute I2S_TRIGGER_DRAIN");
			return -EIO;
		}
		ret = channel->mng->drain(i2s);
		break;

	case I2S_TRIGGER_DROP:
		if (channel->current_state == I2S_STATE_NOT_READY) {
			LOG_ERROR("Failed to execute I2S_TRIGGER_DROP");
			return -EIO;
		}
		ret = channel->mng->drop(i2s);
		break;

	case I2S_TRIGGER_PREPARE:
	{
		if (channel->current_state != I2S_STATE_ERROR) {
			LOG_ERROR("Failed to execute I2S_TRIGGER_PREPARE");
			return -EIO;
		}
		cfg_reinit(dev);
		channel->mng->drop(i2s);
		break;
	}
	default:
		LOG_ERROR("Invalid trigger command");
		return channel_set_error_state(channel, -EINVAL);
	}
	if (ret < 0) {
		LOG_ERROR("Error trigger while execution");
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

static int channel_change_state(struct channel_str *channel,
				 enum i2s_state new_state)
{
	bool change_forbidden = false;
	enum i2s_state old_state = channel->current_state;

	switch (new_state) {
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
	case I2S_STATE_NOT_READY:
	case I2S_STATE_ERROR:
		break;
	default:

		LOG_ERROR("Invalid channel state");
		return channel_set_error_state(channel, -EINVAL);
	}

	if (change_forbidden) {
		LOG_ERROR("Failed to change channel state");
		return channel_set_error_state(channel, -EIO);
	}
	printk("%p: %u\n", channel, new_state);
	channel->current_state = new_state;
	return 0;
}

static int channel_tx_start(struct i2s_nrfx_interface *i2s)
{
	int ret;
	unsigned int key;
	struct channel_str * const ch_tx = &i2s->channel_tx;
	size_t mem_block_size;

	ret = channel_change_state(ch_tx, I2S_STATE_RUNNING);
	if (ret < 0) {
		return ret;
	}
	key = irq_lock();
	if (interface_get_state(i2s) != I2S_IF_RUNNING &&
	    interface_get_state(i2s) != I2S_IF_READY) {
		irq_unlock(key);
		LOG_ERROR("TX start: Invalid interface state");
		return channel_set_error_state(ch_tx, -EIO);
	}
	ret = ch_tx->mng->get_data(i2s,
		(u32_t **)&i2s->buffers.p_tx_buffer, &mem_block_size);
	if (ret < 0) {
		LOG_ERROR("TX start: Failed to get data from queue");
		return channel_set_error_state(ch_tx, ret);
	}
	if (interface_get_state(i2s) == I2S_IF_RUNNING) {
		ret = interface_restart(i2s);
	} else if (interface_get_state(i2s) == I2S_IF_READY) {
		ret = interface_start(i2s);
	}
	irq_unlock(key);
	if (ret < 0) {
		LOG_ERROR("TX start: Failed to start/restart interface");
		return channel_set_error_state(ch_tx, ret);
	}
	return 0;
}

static int channel_rx_start(struct i2s_nrfx_interface *i2s)
{
	int ret;
	unsigned int key;
	struct channel_str * const ch_rx = &i2s->channel_rx;

	ret = channel_change_state(ch_rx, I2S_STATE_RUNNING);
	if (ret < 0) {
		return ret;
	}
	key = irq_lock();
	if (interface_get_state(i2s) != I2S_IF_RUNNING &&
	    interface_get_state(i2s) != I2S_IF_READY) {
		irq_unlock(key);
		LOG_ERROR("RX start: Invalid interface state");
		return channel_set_error_state(ch_rx, -EIO);
	}
	ret = k_mem_slab_alloc(ch_rx->mem_slab,
			       (void **)&i2s->buffers.p_rx_buffer,
			       K_NO_WAIT);
	if (ret < 0) {
		irq_unlock(key);
		LOG_ERROR("RX start: Failed allocate memory slab");
		return channel_set_error_state(ch_rx, ret);
	}
	if (interface_get_state(i2s) == I2S_IF_RUNNING) {
		ret = interface_restart(i2s);
	} else if (interface_get_state(i2s) == I2S_IF_READY) {
		ret = interface_start(i2s);
	}
	irq_unlock(key);
	if (ret < 0) {
		LOG_ERROR("RX start: Failed to start/restart interface");
		return channel_set_error_state(ch_rx, ret);
	}
	return 0;

}
static void channel_tx_mem_clear(struct i2s_nrfx_interface *i2s,
				 void const *first_block)
{
	struct channel_str * const ch_tx = &i2s->channel_tx;
	void *mem_block = (void *)first_block;
	size_t mem_block_size;

	if (first_block == NULL) {
		if (ch_tx->mng->get_data(i2s, (u32_t **)&mem_block,
		    &mem_block_size) != 0) {
			return;
		}
	}
	do {
		k_mem_slab_free(ch_tx->mem_slab,
			(void **)&mem_block);
	} while (ch_tx->mng->get_data(i2s,
				(u32_t **)&mem_block, &mem_block_size) == 0);
	while (ch_tx->sem.count < ch_tx->sem.limit) {
		k_sem_give(&ch_tx->sem);
	}
}

static int channel_tx_stop(struct i2s_nrfx_interface *i2s)
{
	int ret;
	struct channel_str * const ch_rx = &i2s->channel_rx;
	struct channel_str * const ch_tx = &i2s->channel_tx;

	ret = channel_change_state(ch_tx, I2S_STATE_STOPPING);
	if (ret < 0) {
		return ret;
	}
	return interface_stop_restart(i2s, ch_rx->current_state);
}

static int channel_tx_drain(struct i2s_nrfx_interface *i2s)
{
	struct channel_str * const ch_tx = &i2s->channel_tx;

	return channel_change_state(ch_tx, I2S_STATE_STOPPING);
}

static int channel_tx_drop(struct i2s_nrfx_interface *i2s)
{
	int ret;
	struct channel_str * const ch_rx = &i2s->channel_rx;
	struct channel_str * const ch_tx = &i2s->channel_tx;

	if (ch_tx->current_state == I2S_STATE_RUNNING) {
		ret = channel_change_state(ch_tx, I2S_STATE_STOPPING);
		if (ret < 0) {
			return ret;
		}
		ret = interface_stop_restart(i2s, ch_rx->current_state);
		if (ret < 0) {
			interface_error_service(i2s,
					"Failed to restart interface");
			return ret;
		}
	} else {
		ch_tx->mng->mem_clear(i2s, NULL);
	}
	return 0;
}

static int channel_rx_stop(struct i2s_nrfx_interface *i2s)
{
	int ret;
	struct channel_str * const ch_rx = &i2s->channel_rx;
	struct channel_str * const ch_tx = &i2s->channel_tx;

	ret = channel_change_state(ch_rx, I2S_STATE_STOPPING);
	if (ret < 0) {
		return ret;
	}
	return interface_stop_restart(i2s, ch_tx->current_state);

}

static int channel_rx_drop(struct i2s_nrfx_interface *i2s)
{
	int ret;
	struct channel_str * const ch_rx = &i2s->channel_rx;
	struct channel_str * const ch_tx = &i2s->channel_tx;

	if (ch_tx->current_state == I2S_STATE_RUNNING) {
		ret = channel_change_state(ch_rx, I2S_STATE_STOPPING);
		if (ret < 0) {
			return ret;
		}
		ret =  interface_stop_restart(i2s, ch_tx->current_state);
		if (ret < 0) {
			interface_error_service(i2s,
					"Failed to restart interface");
			return ret;
		}
	} else {
		ch_rx->mng->mem_clear(i2s, NULL);
	}
	return 0;
}

static bool channel_check_empty(struct channel_str *channel)
{
	return queue_is_empty(&channel->mem_block_queue);
}

static int channel_tx_get_data(struct i2s_nrfx_interface *i2s,
			       u32_t **buf, size_t *block_size)
{
	struct channel_str *ch_tx = &i2s->channel_tx;
	int ret;

	ret = queue_fetch(&ch_tx->mem_block_queue, (void **)buf, block_size);
	if (ret < 0) {
		return ret;
	}
	k_sem_give(&ch_tx->sem);
	return 0;
}

static void channel_tx_callback(struct i2s_nrfx_interface *i2s,
				nrfx_i2s_buffers_t const *p_released,
				u32_t status, nrfx_i2s_buffers_t *p_new_buffers)
{
	struct channel_str *ch_tx = &i2s->channel_tx;
	size_t mem_block_size;
	u32_t *mem_block = NULL;
	int get_data_ret = 1;

	if (ch_tx->current_state == I2S_STATE_RUNNING &&
	    interface_get_state(i2s) == I2S_IF_NEEDS_RESTART) {
		if (p_released->p_tx_buffer != NULL) {
			if (next_buffers_needed(status)) {
				k_mem_slab_free(ch_tx->mem_slab,
				      (void **)&p_released->p_tx_buffer);
			}
		}
		get_data_ret = ch_tx->mng->get_data(i2s, &mem_block,
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
		get_data_ret = ch_tx->mng->get_data(i2s, &mem_block,
					       &mem_block_size);
	}
	if (ch_tx->current_state == I2S_STATE_STOPPING) {
		enum i2s_trigger_cmd ch_cmd = ch_tx->last_trigger_cmd;

		if (next_buffers_needed(status)) {
			switch (ch_cmd) {
			case I2S_TRIGGER_DROP:
				if (get_data_ret == 0) {
					ch_tx->mng->mem_clear(i2s, mem_block);
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
				channel_set_error_state(ch_tx, 0);
				return;
			}
		} else {
			int ret = channel_change_state(ch_tx, I2S_STATE_READY);

			if (ret != 0) {
				channel_set_error_state(ch_tx, 0);
				return;
			}
			if (ch_cmd == I2S_TRIGGER_DRAIN) {
				return;
			}
		}
	} else if (ch_tx->current_state == I2S_STATE_ERROR) {
		return;
	} else if (get_data_ret < 0) {
		interface_error_service(i2s, "TX internal callback error");
		channel_set_error_state(ch_tx, 0);
		return;
	} else if (channel_check_empty(ch_tx) == true) {
		/*underrun error occurred*/
		if (get_data_ret == 0) {
			k_mem_slab_free(ch_tx->mem_slab,
					(void **)&mem_block);
		}
		interface_error_service(i2s, "TX underrun error");
		channel_set_error_state(ch_tx, 0);
		return;
	}
	/* continue transmission */
	p_new_buffers->p_tx_buffer = mem_block;
}

static void channel_rx_mem_clear(struct i2s_nrfx_interface *i2s,
				 void const *first_block)
{
	struct channel_str * const ch_rx = &i2s->channel_rx;
	void *mem_block;
	size_t mem_block_size;

	while (queue_fetch(&ch_rx->mem_block_queue,
			(void **)&mem_block,
			&mem_block_size) != 0) {
		if (k_sem_take(&ch_rx->sem, K_NO_WAIT) < 0) {
			return;
		}
		k_mem_slab_free(ch_rx->mem_slab,
			(void **)&mem_block);
	}
}

static void channel_rx_callback(struct i2s_nrfx_interface *i2s,
				nrfx_i2s_buffers_t const *p_released,
				u32_t status, nrfx_i2s_buffers_t *p_new_buffers)
{
	struct channel_str *ch_rx = &i2s->channel_rx;
	int ret;

	printk("rxc1\n");
	if (p_released->p_rx_buffer != NULL && next_buffers_needed(status)) {
		ret = ch_rx->mng->get_data(i2s,
			     (u32_t **)&p_released->p_rx_buffer, &i2s->size);
		if (ret < 0) {
			return;
		}
	}
	printk("rxc2\n");
	if (ch_rx->current_state == I2S_STATE_STOPPING) {
		enum i2s_trigger_cmd ch_cmd = ch_rx->last_trigger_cmd;

		if (next_buffers_needed(status)) {
			switch (ch_cmd) {
			case I2S_TRIGGER_DROP:
				ch_rx->mng->mem_clear(i2s, NULL);
				break;
			case I2S_TRIGGER_DRAIN:
			case I2S_TRIGGER_STOP:
				break;
			default:
				LOG_ERROR("RX Callback command unknown");
				channel_set_error_state(ch_rx, 0);
				return;
			}
		} else {
			if (p_released->p_rx_buffer) {
				ret = ch_rx->mng->get_data(i2s,
					(u32_t **)&p_released->p_rx_buffer,
					&i2s->size);
				if (ret < 0) {
					return;
				}
			}
			int ret = channel_change_state(ch_rx,
					I2S_STATE_READY);
			if (ret < 0) {
				channel_set_error_state(ch_rx, 0);
				return;
			}
		}
		i2s->buffers.p_rx_buffer = NULL;
		return;
	} else if (ch_rx->current_state == I2S_STATE_RUNNING &&
		   interface_get_state(i2s) == I2S_IF_NEEDS_RESTART) {
		return;
	} else if (ch_rx->current_state == I2S_STATE_ERROR) {
		return;
	}
	printk("rxc3\n");
	if (next_buffers_needed(status)) {
		ret = k_mem_slab_alloc(ch_rx->mem_slab,
				       (void **)&p_new_buffers->p_rx_buffer,
				       K_NO_WAIT);
		if (ret < 0) {
			/*overrun error occurred*/
			interface_error_service(i2s, "RX overrun error");
			channel_set_error_state(ch_rx, 0);
			return;
		}
	}
}

static int channel_rx_get_data(struct i2s_nrfx_interface *i2s,
			       u32_t **buf, size_t *block_size)
{
	struct channel_str *ch_rx = &i2s->channel_rx;
	int ret = queue_add(&ch_rx->mem_block_queue, *buf, *block_size);

	if (ret < 0) {
		return ret;
	}
	k_sem_give(&ch_rx->sem);

	return 0;
}

static void isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	struct i2s_nrfx_interface *i2s = get_interface(dev);

	nrfx_i2s_irq_handler(/*arg*/);
	if (interface_get_state(i2s) == I2S_IF_RESTARTING) {
		int ret = interface_start(i2s);

		if (ret < 0) {
			interface_error_service(i2s, "Internal ISR error");
		}
		return;
	}
}

#define I2S_NRFX_DEVICE(idx)						       \
static struct queue_item q_rx_##idx##_buf[CONFIG_NRFX_I2S_RX_BLOCK_COUNT + 1]; \
static struct queue_item q_tx_##idx##_buf[CONFIG_NRFX_I2S_TX_BLOCK_COUNT + 1]; \
static void i2s_nrfx_irq_##idx##_config(struct device *dev);		       \
									       \
static void setup_instance_##idx(struct device *dev)			       \
{									       \
	struct i2s_nrfx_interface *i2s = get_interface(dev);		       \
									       \
	queue_init(&i2s->channel_tx.mem_block_queue,			       \
		CONFIG_NRFX_I2S_TX_BLOCK_COUNT + 1, &q_tx_##idx##_buf[0]);     \
	queue_init(&i2s->channel_rx.mem_block_queue,			       \
		CONFIG_NRFX_I2S_RX_BLOCK_COUNT + 1, &q_rx_##idx##_buf[0]);     \
	i2s_nrfx_irq_##idx##_config(dev);				       \
}									       \
									       \
static const struct i2s_nrfx_config channel_cfg_##idx = {		       \
	.sck_pin = DT_NORDIC_NRF_I2S_I2S_##idx##_SCK_PIN,		       \
	.lrck_pin = DT_NORDIC_NRF_I2S_I2S_##idx##_LRCK_PIN,		       \
	.mck_pin = DT_NORDIC_NRF_I2S_I2S_##idx##_MCK_PIN,		       \
	.sdout_pin = DT_NORDIC_NRF_I2S_I2S_##idx##_SDOUT_PIN,		       \
	.sdin_pin = DT_NORDIC_NRF_I2S_I2S_##idx##_SDIN_PIN,		       \
	.instance_init = setup_instance_##idx,				       \
};									       \
									       \
static struct i2s_nrfx_data channels_data_##idx = {			       \
	.interface = {							       \
		.state = I2S_IF_NOT_READY,				       \
		.size = 0,						       \
		.channel_tx = {						       \
			.current_state = I2S_STATE_NOT_READY,		       \
			.last_trigger_cmd = I2S_TRIGGER_PREPARE,	       \
			.mng = &ch_mng.tx,				       \
		},							       \
		.channel_rx = {						       \
			.current_state = I2S_STATE_NOT_READY,		       \
			.last_trigger_cmd = I2S_TRIGGER_PREPARE,	       \
			.mng = &ch_mng.rx,				       \
		}							       \
	}								       \
};									       \
									       \
DEVICE_AND_API_INIT(i2s_##idx, DT_NORDIC_NRF_I2S_I2S_##idx##_LABEL,	       \
		    &i2s_nrfx_initialize, &channels_data_##idx,		       \
		    &channel_cfg_##idx, POST_KERNEL,			       \
		    CONFIG_I2S_INIT_PRIORITY, &i2s_nrf_driver_api);	       \
									       \
static void i2s_nrfx_irq_##idx##_config(struct device *dev)		       \
{									       \
	IRQ_CONNECT(DT_NORDIC_NRF_I2S_I2S_##idx##_IRQ_0 ,		       \
		DT_NORDIC_NRF_I2S_I2S_##idx##_IRQ_0_PRIORITY,		       \
		isr, DEVICE_GET(i2s_##idx), 0);				       \
	irq_enable(DT_NORDIC_NRF_I2S_I2S_##idx##_IRQ_0);		       \
}

static const struct i2s_driver_api i2s_nrf_driver_api = {
	.configure = i2s_nrfx_api_configure,
	.read = i2s_nrfx_read,
	.write = i2s_nrfx_write,
	.trigger = i2s_nrfx_trigger,
};

struct channels_management {
	struct dir_mng rx;
	struct dir_mng tx;
};

static const struct channels_management ch_mng = {
	.rx = {
		.start = channel_rx_start,
		.stop = channel_rx_stop,
		.drain = channel_rx_stop,
		.drop = channel_rx_drop,
		.data_handler = channel_rx_callback,
		.get_data = channel_rx_get_data,
		.mem_clear = channel_rx_mem_clear,
	},
	.tx = {
		.start = channel_tx_start,
		.stop = channel_tx_stop,
		.drain = channel_tx_drain,
		.drop = channel_tx_drop,
		.data_handler = channel_tx_callback,
		.get_data = channel_tx_get_data,
		.mem_clear = channel_tx_mem_clear,
	},
};

I2S_NRFX_DEVICE(0);

static inline struct i2s_nrfx_interface *get_interface(
				struct device *dev __attribute__((unused)))
{
	struct i2s_nrfx_data *const dev_data = DEV_DATA(DEVICE_GET(i2s_0));

	return &dev_data->interface;
}
