/**
 * @file
 * @brief Comparator public API header file.
 */

/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef ZEPHYR_INCLUDE_DRIVERS_COMP_H_
#define ZEPHYR_INCLUDE_DRIVERS_COMP_H_

#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef u8_t comp_event_t;

#define COMP_EVENT_EDGE_RISING			BIT(0)
#define COMP_EVENT_EDGE_FALLING			BIT(1)
#define COMP_EVENT_READY			BIT(2)

/** @brief Comparator operation modes. */
enum comp_op_mode {
	COMP_OP_DIFFERENTIAL,
	COMP_OP_SINGLE_ENDED
};

/** @brief Hysteresis value. User can use only one hysteresis unit. */
struct comp_hysteresis_value {
	/** Value expressed in milivolts. */
	i16_t absolute_value_mv;
	/** Value expressed in permil related to reference voltage. */
	u16_t relative_value_permil;
};

/** @brief Hysteresis high and low values. */
struct comp_hysteresis {
	comp_hysteresis_value down;
	comp_hysteresis_value up;
};

/** @brief Comparator power modes. */
enum comp_power_mode {
	/**	High performance / full power. */
	COMP_PWR_HIGH_SPEED,
	/**	Medium performance / medium power. */
	COMP_PWR_NORMAL,
	/**	Low performance / low power. */
	COMP_PWR_LOW_POWER,
	/**	Very low performance / ultra low power. */
	COMP_PWR_ULTRA_LOW_POWER,
};

/** @brief Comparator output polarity. */
enum comp_polarity {
	/** Output is non-inverted. */
	COMP_POLARITY_NORMAL,
	/** Output is inverted. */
	COMP_POLARITY_INVERTED,
};

/** @brief Comparator trigger commands. */
enum comp_trigger {
	/** Start comparator. */
	COMP_TRIGGER_START,
	/** Stop comparator sampling. */
	COMP_TRIGGER_STOP,
};

/** @brief Comparator reference sources. */
enum comp_reference {
	COMP_REF_VDD_1_16, /**< VDD * 1/16. */
	COMP_REF_VDD_2_16, /**< VDD * 2/16. */
	COMP_REF_VDD_3_16, /**< VDD * 3/16. */
	COMP_REF_VDD_4_16, /**< VDD * 4/16. */
	COMP_REF_VDD_5_16, /**< VDD * 5/16. */
	COMP_REF_VDD_6_16, /**< VDD * 6/16. */
	COMP_REF_VDD_7_16, /**< VDD * 7/16. */
	COMP_REF_VDD_8_16, /**< VDD * 8/16. */
	COMP_REF_VDD_9_16, /**< VDD * 9/16. */
	COMP_REF_VDD_10_16, /**< VDD * 10/16. */
	COMP_REF_VDD_11_16, /**< VDD * 11/16. */
	COMP_REF_VDD_12_16, /**< VDD * 12/16. */
	COMP_REF_VDD_13_16, /**< VDD * 13/16. */
	COMP_REF_VDD_14_16, /**< VDD * 14/16. */
	COMP_REF_VDD_15_16, /**< VDD * 15/16. */
	COMP_REF_VDD,  /**< VDD. */
	COMP_REF_EXT_0, /**< External reference input 0. */
	COMP_REF_EXT_1, /**< External reference input 1. */
	COMP_REF_EXT_2, /**< External reference input 2. */
	COMP_REF_EXT_3, /**< External reference input 3. */
	COMP_REF_EXT_4, /**< External reference input 4. */
	COMP_REF_EXT_5, /**< External reference input 5. */
	COMP_REF_EXT_6, /**< External reference input 6. */
	COMP_REF_EXT_7, /**< External reference input 7. */
	COMP_REF_INT, /**< Internal reference Vref_internal. */
	COMP_REF_INT_1_4, /**< Vref_internal * 1/4. */
	COMP_REF_INT_2_4, /**< Vref_internal * 2/4. */
	COMP_REF_INT_3_4, /**< Vref_internal * 3/4. */
};

/** @brief Comparator working modes. */
enum comp_work_mode {
	COMP_MODE_SINGLE_SHOT,
	COMP_MODE_CONTINUOUS,
	COMP_MODE_STOP_ON_EVENT
};

typedef void (*comp_callback)(struct device *dev, void *user_data);

struct comp_config {
	enum comp_op_mode op_mode;
	enum comp_work_mode work_mode;
	enum comp_polarity polarity;
	enum comp_reference reference;
	enum comp_power_mode power_mode;
	struct comp_hysteresis hysteresis;
	comp_callback callback;
	void *user_data;
	comp_event_t Event;
};

typedef int (*comp_api_configure)(struct device *dev,
				  const struct comp_config *conf_data);
typedef int (*comp_api_start)(struct device *dev /*callback data here??*/);
typedef int (*comp_api_stop)(struct device *dev);
typedef bool (*comp_api_read)(struct device *dev);
typedef bool (*comp_data_valid)(struct device *dev);



/**
 * @brief COMP driver API
 *
 * This is the mandatory API any COMP driver needs to expose.
 */
struct comp_driver_api {
	comp_api_configure configure;
	comp_api_read read;
	comp_api_start start;
	comp_api_stop stop;
	comp_data_valid data_valid;
};

/**
 * @brief comparator configure
 *
 * @param dev		Pointer to the device structure for the driver instance
 * @param conf_data	Pointer to configuration structure
 *
 * @return		0 on success, negative errno code on fail
 */
__syscall int comp_configure(struct device *dev,
			     const struct comp_config *conf_data);

static inline int z_impl_comp_configure(struct device *dev,
					const struct comp_config *conf_data)
{
		const struct comp_driver_api *api = dev->driver_api;

		return api->configure(dev, conf_data);
}

/**
 * @brief comparator start sampling
 *
 * @param dev		Pointer to the device structure for the driver instance
 *
 * @return		0 on success, negative errno code on fail
 */
__syscall int comp_start(struct device *dev /*callback data here??*/);

static inline int z_impl_comp_start(struct device *dev /*callback data here??*/)
{
		const struct comp_driver_api *api = dev->driver_api;

		return api->start(dev);
}

/**
 * @brief comparator stop sampling
 *
 * @param dev		Pointer to the device structure for the driver instance
 *
 * @return		0 on success, negative errno code on fail
 */
__syscall int comp_stop(struct device *dev);

static inline int z_impl_comp_stop(struct device *dev)
{
		const struct comp_driver_api *api = dev->driver_api;

		return api->stop(dev);
}

/**
 * @brief comparator get current output value
 *
 * @param dev		Pointer to the device structure for the driver instance
 *
 * @return		current value of comparator output
 */
__syscall bool read(struct device *dev);

static inline bool z_impl_comp_read(struct device *dev)
{
		const struct comp_driver_api *api = dev->driver_api;

		return api->read(dev);
}

/**
 * @brief check if comparator output is valid
 *
 * @param dev		Pointer to the device structure for the driver instance
 *
 * @return		true if comparator output value is valid
 * 			else - otherwise
 */
__syscall bool data_valid(struct device *dev);

static inline bool z_impl_comp_data_valid(struct device *dev)
{
		const struct comp_driver_api *api = dev->driver_api;

		return api->data_valid(dev);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#include <syscalls/comp.h>

#endif  /* ZEPHYR_INCLUDE_DRIVERS_COMP_H_ */
