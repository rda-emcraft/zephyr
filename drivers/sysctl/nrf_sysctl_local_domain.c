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
	LOG_INF("Sending notify");
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
	//LOG_INF("Got callback of id %u / %p, %p", id, vq[0], vq[1]);
	k_sem_give(&ipc_sem);
}

nrf_sysctl_msg_t *z_nrf_sysctl_decode_request(void * data)
{
	return (nrf_sysctl_msg_t*) data;
}

#if IS_ENABLED(CONFIG_NRF_SYSCTL_SYSTEM_CONTROLLER)
#define CONFIG_MSG_QUEUE_SIZE 16
static struct
{
	nrf_sysctl_msg_t ring_buf[CONFIG_MSG_QUEUE_SIZE];
	nrf_sysctl_msg_t *sorted_list[CONFIG_MSG_QUEUE_SIZE];
	u8_t sorted_list_end;
} queue;
#define RING_BUF_INC(idx, limit) {idx = (++idx <= limit) ? idx : 0; }

static bool add_to_sorted_list(nrf_sysctl_msg_t * const msg)
{
	u8_t store_idx = queue.sorted_list_end; //index indicates where the new entry will be stored
	for (u8_t i = 0; i < queue.sorted_list_end; i ++)
	{
		if (queue.sorted_list[i] == NULL)
		{
			return false;
		}
		if (msg->timestamp < queue.sorted_list[i]->timestamp)
		{
			store_idx = i;
			for (u8_t j = queue.sorted_list_end; j > store_idx; j --)
			{
				queue.sorted_list[j] = queue.sorted_list[j - 1];
			}
			break;

		}
	}
	queue.sorted_list[store_idx] = msg;
	queue.sorted_list_end ++;
	return true;
}

static bool remove_from_sorted_list(nrf_sysctl_msg_t * msg)
{
	bool found = false;
	for (u8_t i = 0; i < queue.sorted_list_end; i ++)
	{
		if (found)
		{
			queue.sorted_list[i - 1] = queue.sorted_list[i];
		}
		if (queue.sorted_list[i] == msg)
		{
			found = true;
		}

	}
	queue.sorted_list_end --;
	return true;
}

static nrf_sysctl_msg_t * find_free_entry(void)
{
	for (u8_t i = 0; i < CONFIG_MSG_QUEUE_SIZE; i ++)
	{
		if (queue.ring_buf[i].id == 0)
		{
			return &queue.ring_buf[i];
		}
	}
	return NULL;
}

static bool queue_add(const nrf_sysctl_msg_t * const msg)
{
	__ASSERT(msg != NULL && msg->id != 0, "Invalid object");
	nrf_sysctl_msg_t *free_entry = find_free_entry();
	if (free_entry == NULL)
	{
		return false;
	}
	memcpy(free_entry, msg, sizeof(nrf_sysctl_msg_t));
	LOG_INF("Add: %hu", msg->id);
	return add_to_sorted_list(free_entry);
}

static nrf_sysctl_msg_t * queue_get_nearest(void)
{
	return queue.sorted_list[0];
}

static void queue_clear(void)
{
	memset(&queue, 0, sizeof(queue));
}

static nrf_sysctl_msg_id_t queue_remove_first(void)
{
	nrf_sysctl_msg_id_t id = queue.sorted_list[0]->id;
	queue.sorted_list[0]->id = 0;
	remove_from_sorted_list(queue.sorted_list[0]);
}

static void log_queue(void)
{
	k_sleep(K_MSEC(100));
	LOG_INF("Queue:");
	k_sleep(K_MSEC(1000));
	for (u8_t i = 0; i < queue.sorted_list_end; i ++)
	{
		LOG_INF("%hu. %u(%08x)", i, queue.sorted_list[i]->id, queue.sorted_list[i]->timestamp);
		k_sleep(K_MSEC(1000));
	}
}


#endif

int endpoint_cb(struct rpmsg_endpoint * ept,
            void *                  data,
            size_t                  len,
            u32_t                   src,
            void *                  priv)
{
	nrf_sysctl_msg_t *rcv_msg = z_nrf_sysctl_decode_request(data);
	LOG_INF("Received id:%u, size: %hu, timestamp: %x",
			rcv_msg->id,
			rcv_msg->data_size,
			(u64_t)rcv_msg->timestamp);
#if IS_ENABLED(CONFIG_NRF_SYSCTL_SYSTEM_CONTROLLER)
	rpmsg_send(ept, SYSCTL_ACK, sizeof(SYSCTL_ACK));
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

void z_nrf_sysctl_send_request1(void)
{
	__ASSERT_MSG_INFO("sending");
	rpmsg_send(&ep, (void*)&m , sizeof(nrf_sysctl_msg_t));
}

int z_nrf_sysctl_send_request(nrf_sysctl_msg_t *msg)
{
	//memcpy((k_timeout_t*)p_test_ctx->settings.buf, &timeout, sizeof(k_timeout_t));
	//p_test_ctx->settings.ping_size = sizeof(k_timeout_t);
       // send_receive(p_test_ctx);
	//return msg->send(msg, NULL, NULL, NULL);
	//LOG_INF("requesting");
	msg->timestamp =
	rpmsg_send(&ep, (void*)msg , sizeof(nrf_sysctl_msg_t));
	return NRF_SUCCESS;
}

#if IS_ENABLED(CONFIG_NRF_SYSCTL_LOCAL_DOMAIN)
static struct k_sem *ipc_kick_send_to_sysctrl;
extern inline struct k_spinlock * z_impl_k_sem_get_lock(void);

void local_domain_kick_to_send(void)
{

	struct k_spinlock *sem_lock = z_impl_k_sem_get_lock();
	//k_spin_release(sem_lock);
	//k_yield();
	__ASSERT_MSG_INFO("G %p", ipc_kick_send_to_sysctrl);
	//k_sem_give(ipc_kick_send_to_sysctrl);
	z_handle_obj_poll_events(&ipc_kick_send_to_sysctrl->poll_events, K_POLL_STATE_SEM_AVAILABLE);
	__ASSERT_MSG_INFO("N");
	//k_spin_lock(sem_lock);


}
#endif


static int z_nrf_sysctl_init(struct k_sem *ipc_kick)
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
	    LOG_INF("CB1=%p %p", vq[0]->callback, vq[1]->callback);
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
//	    virtqueue_notification(vq[0]);
#endif

#if IS_ENABLED(CONFIG_RPMSG_MASTER)
	    /* Wait till nameservice ep is setup */
#warning semaphore
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

	    LOG_INF("Creating endpoint named: %s", ENDPOINT_NAME);
	    status = rpmsg_create_ept(&ep,
	                              rdev,
	                              ENDPOINT_NAME,
	                              RPMSG_ADDR_ANY,
	                              RPMSG_ADDR_ANY,
	                              endpoint_cb,
	                              rpmsg_service_unbind);
	    if (status != 0)
	    {
	        LOG_ERR("rpmsg_create_ept failed %d", status);
	        return;
	    }

	    __ASSERT_MSG_INFO("Initialized and waiting for message...");


#endif
static s32_t ticks[] = {1,5,3,7,8,9,2,3,1,67,43,32,5,567,4,3,2,4,5,6,3,2};
#if IS_ENABLED(CONFIG_NRF_SYSCTL_LOCAL_DOMAIN)
#if 0
	    nrf_sysctl_msg_t my_msg = {
		.id = SYSTEM_CLOCK_SET_TIMEOUT,
		.data_size = sizeof(s32_t),
		//.data = LOC_REQ,
	    };

	    u32_t i = 0;
	    while (my_msg.id < ARRAY_SIZE(ticks))
	    {
		//k_sleep(K_MSEC(1000));
		for (uint32_t i = 20000000; i > 0; i--) arch_nop();
	        //LOG_INF("Sending message: %u", my_msg.id);
		my_msg.timestamp = (u64_t)ticks[my_msg.id];
	        z_nrf_sysctl_send_request(&my_msg);
	        my_msg.id ++;
	    }
#endif
#elif IS_ENABLED(CONFIG_NRF_SYSCTL_SYSTEM_CONTROLLER)
#if 1
	    u8_t cnt = 0;
	    while (1)
	    {

		    __ASSERT_MSG_INFO("Waiting for message from Local Domain");
		k_sem_take(&ipc_sem, K_FOREVER);
		__ASSERT_MSG_INFO("OK");
#if IS_ENABLED(CONFIG_RPMSG_REMOTE)
		if (cnt == ARRAY_SIZE(ticks) - 2)
		{
			log_queue();
			nrf_sysctl_msg_id_t id;
			for (uint8_t i = 0; i < 5; i++)
			{
				id = queue_remove_first();
			}
			log_queue();
		}

		virtqueue_notification(vq[1]);
		cnt ++;
#elif IS_ENABLED(CONFIG_RPMSG_MASTER)
		virtqueue_notification(vq[0]);
#endif
		//k_sleep(K_MSEC(100));

	    }
#endif
#endif //1
	    return NRF_SUCCESS;
}
void nrf_sysctl_init(struct k_sem *ipc_kick)
{
	z_nrf_sysctl_init(ipc_kick);
}
