/* Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "nrf_sysctl_common.h"
#include <drivers/ipm.h>
#include <openamp/open_amp.h>
#include <drivers/sysctl/nrf_sysctl.h>
#include <zephyr.h>

#include <stdio.h>
#include <string.h>


#include <logging/log.h>

#include <zephyr.h>
#include <device.h>
#include <drivers/ipm.h>
#include <drivers/gpio.h>
#include <logging/log.h>

#include <openamp/open_amp.h>
#include <metal/sys.h>
#include <metal/device.h>
#include <metal/alloc.h>

LOG_MODULE_DECLARE(sysctl);
#define DO_NOT_USE_SEMAPHORE 0

#define LOC_REQ		"request"
#define SYSCTL_ACK	"OK"
//#define MASTER_MESSAGE "Master message"
#define ENDPOINT_NAME  "SomeEndpoint"
//#define REMOTE_MESSAGE "Remote response"

/* Shared memory configuration */
#define DT_IPC_SHM_BASE_ADDRESS 0x20070000
#define SHM_START_ADDR      ( DT_IPC_SHM_BASE_ADDRESS + 0x400 )
#define SHM_SIZE            ( 0x7c00 )
#define SHM_DEVICE_NAME     ( "sram0.shm" )

#define VRING_COUNT         ( 2 )
#define VRING_TX_ADDRESS    ( SHM_START_ADDR + SHM_SIZE - 0x400 )
#define VRING_RX_ADDRESS    ( VRING_TX_ADDRESS - 0x400 )
#define VRING_ALIGNMENT     ( 4 )
#define VRING_SIZE          ( 16 )

#define VDEV_STATUS_ADDR    ( DT_IPC_SHM_BASE_ADDRESS )

typedef enum
{
    NRF_SUCCESS             = 0x00, /**< Operation performed successfully. */
    NRF_ERROR_TIMEOUT       = 0x01, /**< Operation timed out. */
    NRF_ERROR_DATA_MISMATCH = 0X02, /**< Operation returned data different than predicted. */
    NRF_ERROR_NO_MEM        = 0X03, /**< No memory for operation. */
    NRF_ERROR_IPC_INIT      = 0X04, /**< Failed to initialize IPC. */
    NRF_ERROR_IPC_ENDPOINT  = 0X04, /**< Failed to initialize IPC. */

} nrf_ret_code_t;


//Z_NRF_SYSCTRL_CREATE_INSTANCE;

static struct k_sem ipc_sem;
#if IS_ENABLED(CONFIG_RPMSG_MASTER)
#if !DO_NOT_USE_SEMAPHORE
	//static K_SEM_DEFINE(sync_sem, 0, 1);
	static struct k_sem sync_sem;
#else
	static volatile bool cb_done = false;
#endif
#endif
static struct device * ipm_tx_handle;
static struct device * ipm_rx_handle;

static metal_phys_addr_t shm_physmap[] = { SHM_START_ADDR };
static struct metal_device shm_device =
{
.name = SHM_DEVICE_NAME,
.bus = NULL,
.num_regions = 1,
.regions =
{
    {
        .virt       = (void *) SHM_START_ADDR,
        .physmap    = shm_physmap,
        .size       = SHM_SIZE,
        .page_shift = 0xffffffff,
        .page_mask  = 0xffffffff,
        .mem_flags  = 0,
        .ops        = { NULL },
    },
},
.node = { NULL },
.irq_num = 0,
.irq_info = NULL
};

static struct virtqueue *    vq[2];
static struct rpmsg_endpoint ep;

static unsigned char virtio_get_status(struct virtio_device *vdev)
{
#if IS_ENABLED(CONFIG_RPMSG_MASTER)
	return VIRTIO_CONFIG_STATUS_DRIVER_OK;
#else
	return sys_read8(VDEV_STATUS_ADDR);
#endif
}

static void virtio_set_status(struct virtio_device *vdev, unsigned char status)
{
	sys_write8(status, VDEV_STATUS_ADDR);
}

static u32_t virtio_get_features(struct virtio_device *vdev)
{
	return 1 << VIRTIO_RPMSG_F_NS;
}

#if IS_ENABLED(CONFIG_RPMSG_MASTER)
static void virtio_set_features(struct virtio_device *vdev, u32_t features)
{
}
#endif

static void virtio_notify(struct virtqueue *vq)
{
	__ASSERT_MSG_INFO("Sending notify");
	int status = ipm_send(ipm_tx_handle, 0, 0, NULL, 0);
	if (status != 0)
	{
	    LOG_ERR("ipm_send failed to notify: %d", status);
	}
}

const struct virtio_dispatch dispatch =
{
	.get_status = virtio_get_status,
#if IS_ENABLED(CONFIG_RPMSG_MASTER)
	.set_status = virtio_set_status,
	.set_features = virtio_set_features,
#endif
	.get_features = virtio_get_features,
	.notify = virtio_notify,
};

static void ipm_callback(void *context, u32_t id, volatile void *data)
{
	__ASSERT_MSG_INFO("Got callback of id %u / %p, %p", id, vq[0], vq[1]);
	static uint32_t cb_cnt = 0;
	cb_cnt ++;
	if (cb_cnt == 2) z_clock_announce(IS_ENABLED(CONFIG_TICKLESS_KERNEL) ? 32768 : (false));//rtc1_nrf_isr(NULL);
	k_sem_give(&ipc_sem);
}

nrf_sysctl_msg_t *z_nrf_sysctl_decode_request(void * data)
{
	return (nrf_sysctl_msg_t*) data;
}

int endpoint_cb(struct rpmsg_endpoint * ept,
            void *                  data,
            size_t                  len,
            u32_t                   src,
            void *                  priv)
{
	nrf_sysctl_msg_t *rcv_msg = z_nrf_sysctl_decode_request(data);
	__ASSERT_MSG_INFO("Received id:%u, size: %hu",
			rcv_msg->id,
			rcv_msg->data_size);
#if IS_ENABLED(CONFIG_NRF_SYSCTL_SYSTEM_CONTROLLER)
	//rpmsg_send(ept, SYSCTL_ACK, sizeof(SYSCTL_ACK));
	//queue_add(rcv_msg);
#endif
	return RPMSG_SUCCESS;
}

static void rpmsg_service_unbind(struct rpmsg_endpoint * ep)
{
	rpmsg_destroy_ept(ep);
}

#if IS_ENABLED(CONFIG_RPMSG_MASTER)
void ns_bind_cb(struct rpmsg_device * rdev, const char * name, u32_t dest)
{
__ASSERT_MSG_INFO("Remote endpoint appeared:");

(void)rpmsg_create_ept(&ep,
                       rdev,
                       name,
                       RPMSG_ADDR_ANY,
                       dest,
                       endpoint_cb,
                       rpmsg_service_unbind);
#if DO_NOT_USE_SEMAPHORE
	cb_done = true;
#else
	//LOG_INF("GIVE");
	k_sem_give(&sync_sem);
#endif
}
#endif
nrf_sysctl_msg_t m = {
	.id = SYSTEM_CLOCK_SET_TIMEOUT,
	.data_size = sizeof(LOC_REQ),
	.timestamp = 0,
};

int z_nrf_sysctl_send_request(nrf_sysctl_msg_t *msg);

void z_nrf_sysctl_send_request1(void)
{

	z_nrf_sysctl_send_request(&m);
	//rpmsg_send(&ep, (void*)&m , sizeof(nrf_sysctl_msg_t));
}

int z_nrf_sysctl_send_request(nrf_sysctl_msg_t *msg)
{
	//memcpy((k_timeout_t*)p_test_ctx->settings.buf, &timeout, sizeof(k_timeout_t));
	//p_test_ctx->settings.ping_size = sizeof(k_timeout_t);
       // send_receive(p_test_ctx);
	//return msg->send(msg, NULL, NULL, NULL);
	//LOG_INF("requesting");
	__ASSERT_MSG_INFO("sending size %u", msg->data_size);
	rpmsg_send(&ep, (void*)msg , sizeof(nrf_sysctl_msg_t));
	return NRF_SUCCESS;
}


#if IS_ENABLED(CONFIG_NRF_SYSCTL_LOCAL_DOMAIN)
static struct k_sem *ipc_kick_send_to_sysctrl;
//extern inline struct k_spinlock * z_impl_k_sem_get_lock(void);

void local_domain_kick_to_send(void)
{

	//struct k_spinlock *sem_lock = z_impl_k_sem_get_lock();
	//k_spin_release(sem_lock);
	//k_yield();
	__ASSERT_MSG_INFO("G %p", ipc_kick_send_to_sysctrl);
	//k_sem_give(ipc_kick_send_to_sysctrl);
	//ipc_kick_send_to_sysctrl->count += (ipc_kick_send_to_sysctrl->count != ipc_kick_send_to_sysctrl->limit) ? 1U : 0U;
	//z_handle_obj_poll_events(&ipc_kick_send_to_sysctrl->poll_events, K_POLL_STATE_SEM_AVAILABLE);
	__ASSERT_MSG_INFO("N");
	//k_spin_lock(sem_lock);


}
#endif


int z_nrf_sysctl_init(struct k_sem *ipc_kick)
{
	k_sem_init(&ipc_sem, 0, 1);
#if IS_ENABLED(CONFIG_NRF_SYSCTL_LOCAL_DOMAIN)
	ipc_kick_send_to_sysctrl = ipc_kick;
#endif
#if IS_ENABLED(CONFIG_RPMSG_MASTER)
#if !DO_NOT_USE_SEMAPHORE
	k_sem_init(&sync_sem, 0, 1);
#endif
#endif
#if IS_ENABLED(CONFIG_RPMSG_REMOTE)
#if 1
	    /* Give Network core the UART pins */
	NRF_P0_S->PIN_CNF[25] = GPIO_PIN_CNF_MCUSEL_NetworkMCU
	                                << GPIO_PIN_CNF_MCUSEL_Pos;
	    NRF_P0_S->PIN_CNF[26] = GPIO_PIN_CNF_MCUSEL_NetworkMCU
	                                << GPIO_PIN_CNF_MCUSEL_Pos;
#endif
#endif
	    int status;

	    struct virtio_vring_info     rvrings[2];
#if IS_ENABLED(CONFIG_RPMSG_MASTER)
	    struct rpmsg_virtio_shm_pool shpool;
#else
	    struct rpmsg_device *      rdev;
#endif
	    struct virtio_device         vdev;
	    struct rpmsg_virtio_device   rvdev;
	    struct metal_io_region *     io;
	    struct metal_device *        device;

	    /* Libmetal setup */
	    struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	    status = metal_init(&metal_params);
	    if (status != 0)
	    {
	        LOG_ERR("metal_init: failed - error code %d", status);
	        return status;
	    }

	    status = metal_register_generic_device(&shm_device);
	    if (status != 0)
	    {
	        LOG_ERR("Couldn't register shared memory device: %d", status);
	        return status;
	    }

	    status = metal_device_open("generic", SHM_DEVICE_NAME, &device);
	    if (status != 0)
	    {
	        LOG_ERR("metal_device_open failed: %d", status);
	        return status;
	    }

	    io = metal_device_io_region(device, 0);
	    if (io == NULL)
	    {
	        LOG_ERR("metal_device_io_region failed to get region");
	        return NRF_ERROR_IPC_INIT;
	    }

	    /* IPM setup */
	    //LOG_ERR("1");
#if IS_ENABLED(CONFIG_RPMSG_REMOTE)
	    ipm_tx_handle = device_get_binding("IPM_1");
	    ipm_rx_handle = device_get_binding("IPM_0");
	    //k_sleep(K_MSEC(1000));
#else
	    ipm_rx_handle = device_get_binding("IPM_1");
	    ipm_tx_handle = device_get_binding("IPM_0");
#endif
	    if (!(ipm_rx_handle && ipm_tx_handle))
	    {
	        LOG_ERR("Could not get IPM device handle");
	        return NRF_ERROR_IPC_INIT;
	    }
	    //LOG_ERR("2");
	    ipm_register_callback(ipm_rx_handle, ipm_callback, NULL);

	    /* Virtqueue setup */
	    vq[0] = virtqueue_allocate(VRING_SIZE);
	    if (vq[0] == NULL)
	    {
	        LOG_ERR("virtqueue_allocate failed to alloc vq[0]");
	        return NRF_ERROR_IPC_INIT;
	    }
	    //LOG_ERR("3 %p", vq[0]);
	    vq[1] = virtqueue_allocate(VRING_SIZE);
	    if (vq[1] == NULL)
	    {
	        LOG_ERR("virtqueue_allocate failed to alloc vq[1]");
	        return NRF_ERROR_IPC_INIT;
	    }
	    //LOG_ERR("4 %p", vq[0]);
	    rvrings[0].io = io;
	    rvrings[0].info.vaddr = (void *)VRING_TX_ADDRESS;
	    rvrings[0].info.num_descs = VRING_SIZE;
	    rvrings[0].info.align = VRING_ALIGNMENT;
	    rvrings[0].vq = vq[0];

	    rvrings[1].io = io;
	    rvrings[1].info.vaddr = (void *)VRING_RX_ADDRESS;
	    rvrings[1].info.num_descs = VRING_SIZE;
	    rvrings[1].info.align = VRING_ALIGNMENT;
	    rvrings[1].vq = vq[1];
#if IS_ENABLED(CONFIG_RPMSG_REMOTE)
	    vdev.role = RPMSG_REMOTE;
#else
	    vdev.role = RPMSG_MASTER;
#endif
	    vdev.vrings_num = VRING_COUNT;
	    vdev.func = &dispatch;
	    vdev.vrings_info = &rvrings[0];
	    __ASSERT_MSG_INFO("CB1=%p %p", vq[0]->callback, vq[1]->callback);
#if IS_ENABLED(CONFIG_RPMSG_MASTER)
	    rpmsg_virtio_init_shm_pool(&shpool, (void *)SHM_START_ADDR, SHM_SIZE);
	    status = rpmsg_init_vdev(&rvdev, &vdev, ns_bind_cb, io, &shpool);
#else
	    status = rpmsg_init_vdev(&rvdev, &vdev, NULL, io, NULL);
#endif
	    if (status != 0)
	    {
	        LOG_ERR("rpmsg_init_vdev failed %d", status);
	        return status;
	    }

#if IS_ENABLED(CONFIG_RPMSG_REMOTE)
#if 1
	    /* Assing UART pins to Network core */
	    NRF_P0->PIN_CNF[25] = GPIO_PIN_CNF_MCUSEL_NetworkMCU << GPIO_PIN_CNF_MCUSEL_Pos;
	    NRF_P0->PIN_CNF[26] = GPIO_PIN_CNF_MCUSEL_NetworkMCU << GPIO_PIN_CNF_MCUSEL_Pos;
#endif
#endif
#if IS_ENABLED(CONFIG_RPMSG_REMOTE) //tak na prawde app?
	    /* Set network as secure */
	    NRF_SPU_S->EXTDOMAIN[0].PERM = (1 << 4);

	    /* Release the Network MCU */
	    NRF_RESET_S->NETWORK.FORCEOFF = 0;
	    /* Since we are using name service, we need to wait for a response
	     * from NS setup and than we need to process it
	     */
	    __ASSERT_MSG_INFO("Waiting for remote endpoint to appear...");
	    //virtqueue_notification(vq[0]);
#endif

#if IS_ENABLED(CONFIG_RPMSG_MASTER)
	    /* Wait till nameservice ep is setup */
#if DO_NOT_USE_SEMAPHORE
	    while (!cb_done)
	    {

	    }
	    cb_done = false;
#else
	    k_sem_take(&ipc_sem, K_FOREVER);
	    ////LOG_INF("XXX");
	    //while (1) {k_sleep(K_MSEC(100));}
	    virtqueue_notification(vq[0]);
	    k_sem_take(&sync_sem, K_FOREVER);
#endif
	    //LOG_ERR("6");

#else
	    rdev = rpmsg_virtio_get_rpmsg_device(&rvdev);

	    __ASSERT_MSG_INFO("Creating endpoint named: %s", ENDPOINT_NAME);
	    status = rpmsg_create_ept(&ep,
	                              rdev,
	                              ENDPOINT_NAME,
	                              RPMSG_ADDR_ANY,
	                              RPMSG_ADDR_ANY,
	                              endpoint_cb,
	                              rpmsg_service_unbind);
	    if (status != 0)
	    {
		__ASSERT_MSG_INFO("rpmsg_create_ept failed %d", status);
	        return;
	    }

	    __ASSERT_MSG_INFO("Initialized and waiting for message...");


#endif
static s32_t ticks[] = {1,5,3,7,8,9,2,3,1,67,43,32,5,567,4,3,2,4,5,6,3,2};
#if IS_ENABLED(CONFIG_NRF_SYSCTL_LOCAL_DOMAIN)
#if 1
	nrf_sysctl_msg_t m_to_send = {
		.id = 15,
		.type = SYSTEM_CLOCK_SET_TIMEOUT,
		.timestamp = 100,
		.data_size = 8,
	};
	while (1)
	{
		__ASSERT_MSG_INFO("kick?");
		//sync = true;


		k_sem_take(ipc_kick_send_to_sysctrl, K_FOREVER);
		virtqueue_notification(vq[1]);
		z_nrf_sysctl_send_request(&m_to_send);
		k_sleep(K_MSEC(100));
		__ASSERT_MSG_INFO("sent");
		//k_sem_take(&ipc_sem, K_FOREVER);

		__ASSERT_MSG_INFO("afterkick %u/%u", ipc_kick_send_to_sysctrl->count, ipc_kick_send_to_sysctrl->limit);

	}
#endif
#elif IS_ENABLED(CONFIG_NRF_SYSCTL_SYSTEM_CONTROLLER)
#if 1
	    u8_t cnt = 0;
	    while (1)
	    {

		__ASSERT_MSG_INFO("Waiting for message from Local Domain");
		k_sem_take(&ipc_sem, K_FOREVER);
		LOG_INF("OK");
		k_sleep(K_MSEC(7000));
		virtqueue_notification(vq[1]);
		z_nrf_sysctl_send_request1();
#elif IS_ENABLED(CONFIG_RPMSG_MASTER)
		virtqueue_notification(vq[0]);
#endif
		//k_sleep(K_MSEC(100));

	    }
#endif //1
	    return NRF_SUCCESS;
}
void nrf_sysctl_init(struct k_sem *ipc_kick)
{
	z_nrf_sysctl_init(ipc_kick);
}
