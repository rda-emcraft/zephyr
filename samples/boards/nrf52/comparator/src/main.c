#include <zephyr.h>
#include <stdio.h>
#include <nrfx_comp.h>
#include <sys/printk.h>

/*
 * @brief maximum value for threshold configuration for hysteresis unit.
 * It should be equal to THUP/THDOWN maximum value (with all bits set)
 * please refer to product specification - TH register description
 */
#define MAX_TH_REG_VALUE 0x3F

/*
 * @brief reference type (single-ended mode only)
 * refer to product specification - REFSEL register description
 * allowed values:
 *   NRF_COMP_REF_Int1V2
 *   NRF_COMP_REF_Int1V8
 *   NRF_COMP_REF_Int2V4
 *   NRF_COMP_REF_VDD
 *   NRF_COMP_REF_ARef
 */
#define COMP_REFERENCE_TYPE NRF_COMP_REF_Int2V4

/*
 * @brief external reference select in single-ended mode or inverted input
 * select in differential mode
 * refer to product specification - EXTREFSEL register description
 * allowed values:
 *   NRF_COMP_EXT_REF_0
 *   NRF_COMP_EXT_REF_1
 */
#define COMP_EXTERNAL_REFERENCE NRF_COMP_EXT_REF_1

/*
 * @brief speed mode select
 * refer to product specification - MODE register description, field MAIN
 * allowed values:
 *   NRF_COMP_MAIN_MODE_SE
 *   NRF_COMP_MAIN_MODE_Diff
 */
#define COMP_OPERATING_MODE NRF_COMP_MAIN_MODE_SE

/*
 * @brief up/down threshold for hysteresis in [%] (single-ended mode only)
 * the final value of up/down threshold is calculated as fraction of chosen
 * reference voltage in [%] e.g:
 * HighThresholdValue = COMP_THRESHOLD_UP * Vref
 * LowThresholdValue = COMP_THRESHOLD_DOWN * Vref
 * For more information please refer to product specification -
 * 'Single-ended mode' section
 * allowed values:
 *    use positive value between 0 and 100
 */
#define COMP_THRESHOLD_UP 90
#define COMP_THRESHOLD_DOWN 80

/*
 * @brief mode select
 * refer to product specification - MODE register description, field SP
 * allowed values:
 *   NRF_COMP_SP_MODE_Low
 *   NRF_COMP_SP_MODE_Normal
 *   NRF_COMP_SP_MODE_High
 */
#define COMP_SPEED_MODE NRF_COMP_SP_MODE_High

/*
 * @brief hysteresis enable/disable (differential mode only)
 * refer to product specification - HYST register description
 * allowed values:
 *   NRF_COMP_HYST_NoHyst
 *   NRF_COMP_HYST_50mV
 */
#define COMP_HYSTERESIS NRF_COMP_HYST_50mV

/*
 * @brief positive input to be monitored in single-ended mode or non-inverted
 * input in differential mode
 * refer to product specification - PSEL register description
 * allowed values:
 *   NRF_COMP_INPUT_0
 *   NRF_COMP_INPUT_1
 *   NRF_COMP_INPUT_2
 *   NRF_COMP_INPUT_3
 *   NRF_COMP_INPUT_4
 *   NRF_COMP_INPUT_5
 *   NRF_COMP_INPUT_6
 *   NRF_COMP_INPUT_7
 */
#define COMP_INPUT NRF_COMP_INPUT_1

/*
 * @brief comparator interrupt priority
 * use values 0 - 7 (0 - highest priority)
 */
#define COMP_INTERRUPT_PRIORITY 7

/*
 * @brief events to be enabled
 * refer to product specification - EVENTS group registers description
 * allowed or-mask of values:
 *   NRFX_COMP_EVT_EN_READY_MASK
 *   NRFX_COMP_EVT_EN_DOWN_MASK
 *   NRFX_COMP_EVT_EN_UP_MASK
 *   NRFX_COMP_EVT_EN_CROSS_MASK
 *   if no events use 0
 */
#define COMP_EVENTS_GENERATE\
	NRFX_COMP_EVT_EN_READY_MASK |\
	NRFX_COMP_EVT_EN_DOWN_MASK |\
	NRFX_COMP_EVT_EN_UP_MASK |\
	NRFX_COMP_EVT_EN_CROSS_MASK

/*
 * @brief shortcuts between events and tasks
 * refer to product specification - SHORTS register description
 * allowed or-mask of values:
 *   NRFX_COMP_SHORT_STOP_AFTER_CROSS_EVT
 *   NRFX_COMP_SHORT_STOP_AFTER_UP_EVT
 *   NRFX_COMP_SHORT_STOP_AFTER_DOWN_EVT
 *   if no shortcuts use 0
 */
#define COMP_SHORTCUT_SET 0

static volatile bool device_is_ready;

#if (COMP_THRESHOLD_DOWN > COMP_THRESHOLD_UP)
#error COMP_THRESHOLD_DOWN cannot be higher than COMP_THRESHOLD_UP
#endif


/*
 * @brief function converts value expressed in percent to TH reg - compatible
 */
u8_t calc_th_reg_value(u8_t value_percent)
{
	u8_t ret_val = MAX_TH_REG_VALUE;
	if (value_percent > 100) {
		printk("\nvalue is greater than 100%%. Value is set to 100%%");
	}
	else {
		ret_val = value_percent * MAX_TH_REG_VALUE / 100;
		printk("\nTH reg value set to: %u", ret_val);
	}
	return ret_val;
}

/*
 * @brief user callback for events chosen by 'COMP_EVENTS_GENERATE' parameter
 */
void usr_callback(nrf_comp_event_t event)
{
	switch (event) {
	case NRF_COMP_EVENT_READY:
		printk("\n[%s Event]: READY", __func__);
		device_is_ready = true;
		break;
	case NRF_COMP_EVENT_DOWN:
		printk("\n[%s Event]: DOWN", __func__);
		break;
	case NRF_COMP_EVENT_UP:
		printk("\n[%s Event]: UP", __func__);
		break;
	case NRF_COMP_EVENT_CROSS:
		printk("\n[%s Event]: CROSS", __func__);
		break;
	default:
		printk("\n[%s Event]: UNKNOWN", __func__);
	}
}

void main(void)
{
	nrfx_comp_config_t comp_cfg = {
		.reference = COMP_REFERENCE_TYPE,
		.ext_ref = COMP_EXTERNAL_REFERENCE,
		.main_mode = COMP_OPERATING_MODE,
		.threshold = {
			.th_down = calc_th_reg_value(COMP_THRESHOLD_DOWN),
			.th_up = calc_th_reg_value(COMP_THRESHOLD_UP),
		},
		.speed_mode = COMP_SPEED_MODE,
		.hyst = COMP_HYSTERESIS,
		.input = COMP_INPUT,
		.interrupt_priority = COMP_INTERRUPT_PRIORITY
	};
	printk("\nNRFX comparator module");

	if (COMP_EVENTS_GENERATE) {
		printk("\nIRQ enabled");
		IRQ_CONNECT(DT_NORDIC_NRF_COMPARATOR_COMP_0_IRQ_0,
			    DT_NORDIC_NRF_COMPARATOR_COMP_0_IRQ_0_PRIORITY,
			    nrfx_isr,
			    nrfx_comp_irq_handler,
			    0);
	}

	int ret = nrfx_comp_init(&comp_cfg, &usr_callback);
	if (ret != NRFX_SUCCESS) {
		printk("\nnrfx_comp_init() returned %d", ret);
	}
	nrfx_comp_start(COMP_EVENTS_GENERATE, COMP_SHORTCUT_SET);
	while (!device_is_ready) {

	}
	while (1) {
		printk("\nState %u", nrfx_comp_sample());
		k_sleep(1000);
	}


}
