#include <zephyr.h>
//#include <drivers/comp.h>
#include <device.h>
#include <stdio.h>
#include <nrfx_comp.h>

#define DT_COMP_DEV_NAME	"COMP_0"

/*
 typedef enum
{
    NRF_COMP_EVENT_READY = offsetof(NRF_COMP_Type, EVENTS_READY),
    NRF_COMP_EVENT_DOWN  = offsetof(NRF_COMP_Type, EVENTS_DOWN),
    NRF_COMP_EVENT_UP    = offsetof(NRF_COMP_Type, EVENTS_UP),
    NRF_COMP_EVENT_CROSS = offsetof(NRF_COMP_Type, EVENTS_CROSS)
} nrf_comp_event_t;

typedef struct
{
    nrf_comp_ref_t          reference;
    nrf_comp_ext_ref_t      ext_ref;
    nrf_comp_main_mode_t    main_mode;
    nrf_comp_th_t           threshold;
    nrf_comp_sp_mode_t      speed_mode;
    nrf_comp_hyst_t         hyst;
#if defined (COMP_ISOURCE_ISOURCE_Msk) || defined (__NRFX_DOXYGEN__)
    nrf_isource_t           isource;
#endif
    nrf_comp_input_t        input;
    uint8_t                 interrupt_priority;
} nrfx_comp_config_t;


 */






void usr_callback(nrf_comp_event_t event)
{

}

nrfx_comp_config_t comp_cfg = NRFX_COMP_DEFAULT_CONFIG(0);



void main(void)
{

	//struct device *comp;
	nrfx_comp_init(&comp_cfg, &usr_callback);
	/*comp = device_get_binding(DT_COMP_DEV_NAME);
	if (comp == NULL) {
		printk("invalid device binding\n");
		return;
	}
	comp_configure(comp, &cfg);
	comp_start(comp);*/

}
