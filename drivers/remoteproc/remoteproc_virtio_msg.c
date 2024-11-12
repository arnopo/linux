// SPDX-License-Identifier: GPL-2.0-only
/*
 * Remote processor virtio messaging transport bus
 *
 * Copyright (C) STMicroelectronics 2024
 *
 * Ohad Ben-Cohen <ohad@wizery.com>
 * Brian Swetland <swetland@google.com>
 */

#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include <linux/export.h>
#include <linux/mailbox_client.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/err.h>
#include <linux/kref.h>
#include <linux/slab.h>
#include <linux/virtio_config.h>
#include <linux/virtio_msg.h>


#include "remoteproc_internal.h"

struct rproc_vmdev_bus;

enum rproc_bus_status {
	RPROC_UNBOUND 	= 0x0,
	RPROC_BOUND 	= 0x1,
};

#define RPROC_VMSG_QUEUE_READY BIT(0)

#define MAGIC_Q_DEF 0x1A2B3C4D
/*
 * shared queue for one direction communication
 * note:
 *   - Who is in charge of initializing the queues, the guest or the host ?
 *     both can start first => magic number?
 *   - write only by one side, read only by the other
 */

struct amp_queue_head {
	u16		status;		/* magic & ready indication */
	u16		resv;		/* reserved for now, maybe restart detect */
	u16		write_idx;	/* head offset for my queue */
	u16		read_idx;	/* tail offset for other queue */
};

/* queue layout definition, static at */
struct amp_queue_def {
	u32	magic;		/* magic & ready indication
				 * ready = MAGIC_Q_DEF
				 * not ready = 0 or MAGIC_Q_DEF_NOT_READY
				 */
	u32	version;	/* fixed to 1 */
	u32	drv_peer_ord;	/* should be 0 if not used */
	u32	dev_peer_ord;	/* should be 1 if not used */

	/*
	 * size and count of queue elements from driver to device
	 * element size should be a power of 2,
	 *    some implementations may only support a given size such as 64
	 * number of elements should be > 1
	 */
	u16	drv_elt_size;
	u16	drv_num_elts;

	/*
	 * size and count of queue elements from device to driver
	 * same constraints as driver to device
	 */
	u16	dev_elt_size;
	u16	dev_num_elts;

	/*
	 * offsets from the start of the containing memory area
	 * each head is of type amp_queue_head_t
	 * each data is u8 data[num_elements][element_size]
	 */
	u64	drv_head_off;
	u64	drv_data_off;
	u64	dev_head_off;
	u64	dev_data_off;
};

/* queue layout definition, static at */
struct rproc_queue {
	u16	elt_size;	/* always 64 */
	u16	num_elt;	/* element count */

	struct	amp_queue_head *head;	/* offset of device side queue head */
	u8	*queue;	/* address of the queue */
};

struct rproc_vmdev {

	struct virtio_msg_device vmdev;
	struct list_head node;
	struct rproc_vmdev_bus *bus;

	u32 type;
	/* device id on the virtio-msg-bus */
	u16 dev_id;

	/* temporary fields for test */
	u32 status;
};

struct rproc_vmdev_bus {
	struct platform_device *pdev;

	struct list_head rvmdevs;
	struct mbox_client mb_cl;
	struct mbox_chan *mb_ch;

	struct 	mutex req_lock;
	struct work_struct scan_work;
	struct completion dev_notif;
	struct virtio_msg *response;

	struct amp_queue_def *shmem_queue;
	struct rproc_queue rx;
	struct rproc_queue tx;

	u32 state;
};

static int queue_def_check(struct amp_queue_def *q)
{
	if (q->magic != MAGIC_Q_DEF || q->version != 1)
		return -EINVAL;

	if (q->drv_elt_size < VIRTIO_MSG_MAX_SIZE ||
	    q->dev_elt_size < VIRTIO_MSG_MAX_SIZE) {
		pr_err("Invalid queue element sizes");
		return -EINVAL;
	}
	if (!q->drv_head_off || !q->dev_head_off ||
	    !q->drv_data_off || !q->drv_data_off) {
		pr_err("Invalid queue element sizes");
		return -EINVAL;
	}
	return 0;
}

static int queues_init(struct rproc_vmdev_bus *rvmdev_bus, struct amp_queue_def *q)
{
	struct rproc_queue *rx = &rvmdev_bus->rx;
	struct rproc_queue *tx = &rvmdev_bus->tx;
	int ret;

	ret = queue_def_check(q);
	if (ret)
		return ret;

	rx->elt_size = q->dev_elt_size;
	tx->elt_size = q->drv_elt_size;
	rx->num_elt = q->dev_num_elts;
	tx->num_elt = q->drv_num_elts;

	rx->head = (struct amp_queue_head *)(((uint8_t *)q) + q->dev_head_off);
	tx->head = (struct amp_queue_head *)(((uint8_t *)q) + q->drv_head_off);
	rx->queue = (uint8_t *)q + q->dev_data_off;
	tx->queue = (uint8_t *)q + q->drv_data_off;

	/* Allocate a cache-line aligned spsc-queue. */
	memset(tx->head, 0, sizeof(*tx->head));
	tx->head->status = RPROC_VMSG_QUEUE_READY;

	return 0;
}

static int queue_send(struct rproc_vmdev_bus *bus, void *buf, size_t msg_len)
{
	struct rproc_queue *rx = &bus->rx;
	struct rproc_queue *tx = &bus->tx;
	u8 head = tx->head->write_idx;
	u8 tail = rx->head->read_idx;
	u8 next;

	/* compute the head pointer */
	next = head + 1;
	if (next == tx->num_elt)
		next = 0;

	/* If the queue is full, bail out */
	if (next == tail)
		return -EBUSY;

	/* otherwise write in the packet */
	memcpy(&tx->queue[head * tx->num_elt], buf, msg_len);

	tx->head->write_idx = next;

	return 0;
}

static int queue_receive(struct rproc_vmdev_bus *bus, void *buf, size_t msg_len)
{
	struct rproc_queue *rx = &bus->rx;
	const struct rproc_queue *tx = &bus->tx;
	u8 head = rx->head->write_idx;
	u8 tail = tx->head->read_idx;

	if (head >= rx->num_elt || tail >= rx->num_elt)
		return -EINVAL;

	/* If the queue is empty, bail out */
	if (head == tail)
		return -ENOMEM;

	// otherwise read out the packet
	memcpy(buf, &rx->queue[tail * rx->num_elt], msg_len);

	if (++tail == rx->num_elt)
		tail = 0;

	tx->head->read_idx = tail;

	return 0;
}

#define vmdev_to_rvmdev(_vmdev) \
	container_of(_vmdev, struct rproc_vmdev, vmdev)

static int tx_msg(struct rproc_vmdev_bus *rvmdev_bus, void *msg_buf, size_t msg_len)
{
	int ret;

	dev_dbg(&rvmdev_bus->pdev->dev, "TX MSG: %40ph \n", msg_buf);

	queue_send(rvmdev_bus, msg_buf, msg_len);

	ret = mbox_send_message(rvmdev_bus->mb_ch, "msg");
	if (ret < 0) {
		dev_err(&rvmdev_bus->pdev->dev, "Error sending mbox message\n");
		return ret;
	}

	return 0;
}

static int rproc_vmsg_send(struct virtio_msg_device *vmdev,
				struct virtio_msg *request,
				struct virtio_msg *response)
{
	struct rproc_vmdev *rvmdev = vmdev_to_rvmdev(vmdev);
	struct rproc_vmdev_bus *rvmdev_bus;
	int ret = 0;

	if(!rvmdev || !rvmdev->bus)
		return -EINVAL;

	rvmdev_bus = rvmdev->bus;

	dev_dbg(&rvmdev_bus->pdev->dev,  "send message dev_id=%d, type/id=%04x\n",
			  vmdev->dev_id, request->id);

	/* We need to serialize messages that need a response */
	mutex_lock(&rvmdev_bus->req_lock);

	if (response)
		reinit_completion(&rvmdev_bus->dev_notif);

	rvmdev_bus->response = response;

	ret =tx_msg(rvmdev_bus, request, sizeof(struct virtio_msg));
	if (ret) {
		dev_err(&rvmdev_bus->pdev->dev, "message send fails (%d)\n", ret);
		goto out;
	}

	if (!response)
		goto out;

	if (!wait_for_completion_timeout(&rvmdev_bus->dev_notif, 5000)) {
		dev_err(&rvmdev_bus->pdev->dev,  "response wait timeout dev_id=%d, type/id=%04x\n",
			  vmdev->dev_id, request->id);
		dev_err(&rvmdev_bus->pdev->dev,  "ret =%d", ret);
		ret = -EAGAIN;
		goto out;
	}

	dev_dbg(&rvmdev_bus->pdev->dev, "RX MSG: %40ph \n", response);

	if (request->id != response->id || request->id != response->id) {
		/*
		 * TODO : We should have a mechanism to reinit the bus to synchronize
		 * host an guest
		 */
		 ret = -EPIPE;
	}

out:
	rvmdev_bus->response  = NULL;
	mutex_unlock(&rvmdev_bus->req_lock);

	return ret;
}

static const char *rproc_vmsg_bus_name(struct virtio_msg_device *vmdev)
{
	struct rproc_vmdev *rvmdev = vmdev_to_rvmdev(vmdev);

	pr_err("%s: %d\n", __func__, __LINE__);

	return rvmdev->bus->pdev->name;
}

static void rproc_vmsg_release(struct virtio_msg_device *vmdev)
{
	struct rproc_vmdev *rvmdev = vmdev_to_rvmdev(vmdev);

	pr_err("%s: %d\n", __func__, __LINE__);

	list_del(&rvmdev->node);
	kfree(rvmdev);
}

static int rproc_vmsg_vqs_prepare(struct virtio_msg_device *vmdev)
{
	pr_err("%s: %d\n", __func__, __LINE__);
	/* TODO */
	return 0;
}

static void rproc_vmsg_vqs_release(struct virtio_msg_device *vmdev)
{
	pr_err("%s: %d\n", __func__, __LINE__);
	/* TODO */

}

static struct virtio_msg_ops vmm_ops = {
	.send = rproc_vmsg_send,
	.bus_name = rproc_vmsg_bus_name,
	.release = rproc_vmsg_release,
	.prepare_vqs = rproc_vmsg_vqs_prepare,
	.release_vqs = rproc_vmsg_vqs_release,
};

static struct rproc_vmdev *get_rvmdev(struct rproc_vmdev_bus *rvmdev_bus,
				      struct virtio_msg *msg)
{
	struct rproc_vmdev *rvmdev;

	list_for_each_entry(rvmdev, &rvmdev_bus->rvmdevs, node) {
		if (rvmdev->vmdev.dev_id == msg->dev_id)
			return rvmdev;
	}

	return NULL;
}

static void rproc_vmsg_mb_callback(struct mbox_client *cl, void *data)
{
	struct rproc_vmdev_bus *rvmdev_bus = dev_get_drvdata(cl->dev);
	struct rproc_vmdev *rvmdev;
	struct virtio_msg msg;

	if(unlikely(rvmdev_bus->state == RPROC_UNBOUND)) {
		/* initialize the bus  */
		schedule_work(&rvmdev_bus->scan_work);
		return;
	}

	while (!queue_receive(rvmdev_bus, &msg, sizeof(msg))) {
		rvmdev = get_rvmdev(rvmdev_bus, &msg);

		switch (msg.id) {
		case  VIRTIO_MSG_EVENT_CONFIG:
		case  VIRTIO_MSG_EVENT_AVAIL:
		case  VIRTIO_MSG_EVENT_USED:
			virtio_msg_receive(&rvmdev->vmdev, &msg);
			break;
		default:
			if (rvmdev_bus->response)
				memcpy(rvmdev_bus->response, &msg, sizeof(msg));
			complete(&rvmdev_bus->dev_notif);
		}
	}
}

const struct mbox_client rproc_vmsg_client = {
	.rx_callback = rproc_vmsg_mb_callback,
	.tx_block = false,
};


static int rproc_vmsg_of_parse_mem(struct rproc_vmdev_bus *rvmdev_bus)
{
	struct device *dev = &rvmdev_bus->pdev->dev;
	struct of_phandle_iterator it;
	struct reserved_mem *rmem;
	unsigned int index = 0;
	int ret;

	/*
	 * two memory regions should be declared
	 *  - the index 0 is used to define  memory region for the virtio msg bus
	 *  - the index 1 is used as common pool for virtio drivers memory allocation
	 */
	of_for_each_phandle(&it, ret, dev->of_node, "memory-region", NULL, 0)	{
		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			dev_err(dev, "unable to acquire memory-region %s\n", it.node->name);
			continue;
		}
		dev_err(dev, "memory base %#x", rmem->base);

		if (!index) {
			rvmdev_bus->shmem_queue = (__force struct amp_queue_def *)
						  ioremap_wc(rmem->base, rmem->size);
			if (IS_ERR_OR_NULL(rvmdev_bus->shmem_queue)) {
				dev_err(dev, "Unable to map memory region: %zx[p]+%zx\n",
					rmem->base, rmem->size);
				return -ENOMEM;
			}
		} else if (index == 1) {
			ret = dma_declare_coherent_memory(dev, rmem->base, rmem->base, rmem->size);
			if (ret) {
				dev_err(dev, "Unable to declare coherent memory: %zx+%zx\n",
					rmem->base, rmem->size);
				return ret;
			}
		} else {
			dev_err(dev, "only 2 memory regions expected");
			return -EINVAL;
		}
		index++;
	}

	return 0;
}

static int rproc_vmsg_of_parse_vdev(struct rproc_vmdev_bus *rvmdev_bus)
{
	struct device *dev = &rvmdev_bus->pdev->dev;
	struct device_node *child_np;
	struct rproc_vmdev *rvmdev, *rvmdev_tmp;
	unsigned int vdev_id = 0;
	int ret;

	for_each_child_of_node(dev->of_node, child_np) {
		rvmdev = kzalloc(sizeof(*rvmdev), GFP_KERNEL);
		if (!rvmdev)
			return -ENOMEM;

		rvmdev->vmdev.vdev.dev.parent = dev;
		rvmdev->vmdev.ops = &vmm_ops;
		rvmdev->bus = rvmdev_bus;

		/* The virtio,id define the virtio type */
		ret = of_property_read_u32(child_np, "virtio,type", &rvmdev->type);
		if (ret)
			goto unreg_virt;

		list_add_tail(&rvmdev->node, &rvmdev_bus->rvmdevs);

		ret = virtio_msg_register(&rvmdev->vmdev);
		if(ret)
			goto unreg_virt;

		rvmdev->dev_id = vdev_id++;
		/* FIXME: Temporary code : only one virtio dev per bus supported */
		break;
	}

	return 0;

unreg_virt:
	list_for_each_entry_safe(rvmdev, rvmdev_tmp, &rvmdev_bus->rvmdevs, node) {
		virtio_msg_unregister(&rvmdev->vmdev);
		list_del(&rvmdev->node);
		kfree(rvmdev);
	}
	return ret;

}

static void proc_vmsg_scan_work(struct work_struct *ws)
{
	struct rproc_vmdev_bus *rvmdev_bus = container_of(ws, struct rproc_vmdev_bus, scan_work);
	int ret;

	if (rvmdev_bus->state == RPROC_BOUND)
		return;

	/* Assume first that the shared memory is initialised by the device */
	ret = queues_init(rvmdev_bus, rvmdev_bus->shmem_queue);
	if (ret)
		return;

	rvmdev_bus->state = RPROC_BOUND;

	ret = rproc_vmsg_of_parse_vdev(rvmdev_bus);
	WARN_ON (ret);
}

static int rproc_vmsg_of_parse(struct device *dev, struct rproc_vmdev_bus *rvmdev_bus)
{
	int is_64 = sizeof(phys_addr_t) / sizeof(u64);
	int ret;

	ret = rproc_vmsg_of_parse_mem(rvmdev_bus);
	if (ret)
		return ret;

	/* check for a mailbox */
	rvmdev_bus->mb_cl.tx_block = 1;
	rvmdev_bus->mb_cl.tx_done = NULL;
	rvmdev_bus->mb_cl.tx_tout = 500;
	rvmdev_bus->mb_cl.dev = dev;
	rvmdev_bus->mb_cl.rx_callback = rproc_vmsg_mb_callback;
	rvmdev_bus->mb_ch = mbox_request_channel(&rvmdev_bus->mb_cl, 0);
	if (IS_ERR(rvmdev_bus->mb_ch))
		return PTR_ERR(rvmdev_bus->mb_ch);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(is_64 ? 64 : 32));
	if (ret)
		dev_warn(dev, "Failed to enable %s DMA\n", is_64 ? "64-bit" : "32-bit");

	platform_set_drvdata(rvmdev_bus->pdev, rvmdev_bus);

	return ret;
}

static int rproc_vmsg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rproc_vmdev_bus *rvmdev_bus;
	int ret;

	rvmdev_bus = kzalloc(sizeof(*rvmdev_bus), GFP_KERNEL);
	if (!rvmdev_bus)
		return -ENOMEM;

	rvmdev_bus->pdev = pdev;
	INIT_LIST_HEAD(&rvmdev_bus->rvmdevs);

	ret = rproc_vmsg_of_parse(dev, rvmdev_bus);
	if (ret)
		return ret;

	mutex_init(&rvmdev_bus->req_lock);
	init_completion(&rvmdev_bus->dev_notif);

	INIT_WORK(&rvmdev_bus->scan_work,proc_vmsg_scan_work);

	rvmdev_bus->state = RPROC_UNBOUND;

	schedule_work(&rvmdev_bus->scan_work);
	return 0;
}

static void rproc_vmsg_remove(struct platform_device *pdev)
{
	struct rproc_vmdev_bus *rvmdev_bus = platform_get_drvdata(pdev);
	struct rproc_vmdev *rvmdev, *rvmdev_tmp;

	cancel_work_sync(&rvmdev_bus->scan_work);

	list_for_each_entry_safe(rvmdev, rvmdev_tmp, &rvmdev_bus->rvmdevs, node) {
		virtio_msg_unregister(&rvmdev->vmdev);
		list_del(&rvmdev->node);
		kfree(rvmdev);
	}

	mbox_free_channel(rvmdev_bus->mb_ch);

	dma_release_coherent_memory(&pdev->dev);
}

/* Platform driver */
static const struct of_device_id rproc_vmsg_match[] = {
	{ .compatible = "virtio-msg,rproc" },
	{},
};

static struct platform_driver rproc_vmsg_driver = {
	.probe		= rproc_vmsg_probe,
	.remove		= rproc_vmsg_remove,
	.driver		= {
		.name	= "rproc-virtio-msg",
		.of_match_table	= rproc_vmsg_match,
	},
};
module_platform_driver(rproc_vmsg_driver);


MODULE_AUTHOR("Arnaud Pouliquen <arnaud.pouliquen@foss.st.com>");
MODULE_DESCRIPTION("Virtio message transport remoteproc bus");
MODULE_LICENSE("GPL");