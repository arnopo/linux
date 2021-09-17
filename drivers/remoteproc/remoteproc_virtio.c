// SPDX-License-Identifier: GPL-2.0-only
/*
 * Remote processor messaging transport (OMAP platform-specific bits)
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
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
#include <linux/remoteproc.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_ring.h>
#include <linux/err.h>
#include <linux/kref.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "remoteproc_internal.h"

struct rproc_vdev_mbox {
	const unsigned char *name;
	struct mbox_chan *chan;
	struct mbox_client cl;
	struct work_struct vq_work;
	int notifyid;
};

static int copy_dma_range_map(struct device *to, struct device *from)
{
	const struct bus_dma_region *map = from->dma_range_map, *new_map, *r;
	int num_ranges = 0;

	if (!map)
		return 0;

	for (r = map; r->size; r++)
		num_ranges++;

	new_map = kmemdup(map, array_size(num_ranges + 1, sizeof(*map)),
			  GFP_KERNEL);
	if (!new_map)
		return -ENOMEM;
	to->dma_range_map = new_map;
	return 0;
}

static struct rproc_vdev *vdev_to_rvdev(struct virtio_device *vdev)
{
	struct platform_device *pdev;

	pdev = container_of(vdev->dev.parent, struct platform_device, dev);

	return platform_get_drvdata(pdev);
}

static  struct rproc *vdev_to_rproc(struct virtio_device *vdev)
{
	struct rproc_vdev *rvdev = vdev_to_rvdev(vdev);

	return rvdev->rproc;
}

/* kick the remote processor, and let it know which virtqueue to poke at */
static bool rproc_virtio_notify(struct virtqueue *vq)
{
	struct rproc_vring *rvring = vq->priv;
	struct rproc_vdev *rvdev = rvring->rvdev;
	struct rproc *rproc = rvdev->rproc;
	int notifyid = rvring->notifyid;
	unsigned int i;
	int err;

	dev_dbg(&rproc->dev, "kicking vq index: %d\n", notifyid);

	if (rvdev->mbox) {
		dev_dbg(&rproc->dev, "nb mbox: %d\n", rvdev->nb_mbox);
		for (i = 0; i < rvdev->nb_mbox; i++) {
			if (notifyid != rvdev->mbox[i].notifyid)
				continue;
			dev_dbg(&rproc->dev, "mbox: %s\n", rvdev->mbox[i].name);
			err = mbox_send_message(rvdev->mbox[i].chan, &rvdev->mbox[i].notifyid);
			if (err < 0)
				dev_err(&rproc->dev, "%s: failed (%s, err:%d)\n",
					__func__, rvdev->mbox[i].name, err);
		}
	} else {
		rproc->ops->kick(rproc, notifyid);
	}

	return true;
}

static int
rproc_parse_vring(struct rproc_vdev *rvdev, struct fw_rsc_vdev *rsc, int i)
{
	struct rproc *rproc = rvdev->rproc;
	struct device *dev = &rproc->dev;
	struct fw_rsc_vdev_vring *vring = &rsc->vring[i];
	struct rproc_vring *rvring = &rvdev->vring[i];

	dev_dbg(dev, "vdev rsc: vring%d: da 0x%x, qsz %d, align %d\n",
		i, vring->da, vring->num, vring->align);

	/* verify queue size and vring alignment are sane */
	if (!vring->num || !vring->align) {
		dev_err(dev, "invalid qsz (%d) or alignment (%d)\n",
			vring->num, vring->align);
		return -EINVAL;
	}

	rvring->num = vring->num;
	rvring->align = vring->align;
	rvring->rvdev = rvdev;

	return 0;
}

static void rproc_free_vring(struct rproc_vring *rvring)
{
	struct rproc *rproc = rvring->rvdev->rproc;
	int idx = rvring - rvring->rvdev->vring;
	struct fw_rsc_vdev *rsc;

	idr_remove(&rproc->notifyids, rvring->notifyid);

	/*
	 * At this point rproc_stop() has been called and the installed resource
	 * table in the remote processor memory may no longer be accessible. As
	 * such and as per rproc_stop(), rproc->table_ptr points to the cached
	 * resource table (rproc->cached_table).  The cached resource table is
	 * only available when a remote processor has been booted by the
	 * remoteproc core, otherwise it is NULL.
	 *
	 * Based on the above, reset the virtio device section in the cached
	 * resource table only if there is one to work with.
	 */
	if (rproc->table_ptr) {
		rsc = (void *)rproc->table_ptr + rvring->rvdev->rsc_offset;
		rsc->vring[idx].da = 0;
		rsc->vring[idx].notifyid = -1;
	}
}


/**
 * rproc_vq_interrupt() - tell remoteproc that a virtqueue is interrupted
 * @rproc: handle to the remote processor
 * @notifyid: index of the signalled virtqueue (unique per this @rproc)
 *
 * This function should be called by the platform-specific rproc driver,
 * when the remote processor signals that a specific virtqueue has pending
 * messages available.
 *
 * Return: IRQ_NONE if no message was found in the @notifyid virtqueue,
 * and otherwise returns IRQ_HANDLED.
 */
irqreturn_t rproc_vq_interrupt(struct rproc *rproc, int notifyid)
{
	struct rproc_vring *rvring;

	dev_dbg(&rproc->dev, "vq index %d is interrupted\n", notifyid);

	rvring = idr_find(&rproc->notifyids, notifyid);
	if (!rvring || !rvring->vq)
		return IRQ_NONE;

	return vring_interrupt(0, rvring->vq);
}
EXPORT_SYMBOL(rproc_vq_interrupt);

static struct virtqueue *rp_find_vq(struct virtio_device *vdev,
				    unsigned int id,
				    void (*callback)(struct virtqueue *vq),
				    const char *name, bool ctx)
{
	struct rproc_vdev *rvdev = vdev_to_rvdev(vdev);
	struct rproc *rproc = vdev_to_rproc(vdev);
	struct device *dev = &rproc->dev;
	struct rproc_mem_entry *mem;
	struct rproc_vring *rvring;
	struct fw_rsc_vdev *rsc;
	struct virtqueue *vq;
	void *addr;
	int num, size;

	/* we're temporarily limited to two virtqueues per rvdev */
	if (id >= ARRAY_SIZE(rvdev->vring))
		return ERR_PTR(-EINVAL);

	if (!name)
		return NULL;

	/* Search allocated memory region by name */
	mem = rproc_find_carveout_by_name(rproc, "vdev%dvring%d", rvdev->index,
					  id);
	if (!mem || !mem->va)
		return ERR_PTR(-ENOMEM);

	rvring = &rvdev->vring[id];
	addr = mem->va;
	num = rvring->num;

	/* zero vring */
	size = vring_size(num, rvring->align);
	memset(addr, 0, size);

	dev_dbg(dev, "vring%d: va %pK qsz %d notifyid %d\n",
		id, addr, num, rvring->notifyid);

	/*
	 * Create the new vq, and tell virtio we're not interested in
	 * the 'weak' smp barriers, since we're talking with a real device.
	 */
	vq = vring_new_virtqueue(id, num, rvring->align, vdev, false, ctx,
				 addr, rproc_virtio_notify, callback, name);
	if (!vq) {
		dev_err(dev, "vring_new_virtqueue %s failed\n", name);
		rproc_free_vring(rvring);
		return ERR_PTR(-ENOMEM);
	}

	vq->num_max = num;

	rvring->vq = vq;
	vq->priv = rvring;

	/* Update vring in resource table */
	rsc = (void *)rproc->table_ptr + rvdev->rsc_offset;
	rsc->vring[id].da = mem->da;

	return vq;
}

static void __rproc_virtio_del_vqs(struct virtio_device *vdev)
{
	struct virtqueue *vq, *n;
	struct rproc_vring *rvring;

	list_for_each_entry_safe(vq, n, &vdev->vqs, list) {
		rvring = vq->priv;
		rvring->vq = NULL;
		vring_del_virtqueue(vq);
	}
}

static void rproc_virtio_del_vqs(struct virtio_device *vdev)
{
	__rproc_virtio_del_vqs(vdev);
}

static int rproc_virtio_find_vqs(struct virtio_device *vdev, unsigned int nvqs,
				 struct virtqueue *vqs[],
				 vq_callback_t *callbacks[],
				 const char * const names[],
				 const bool * ctx,
				 struct irq_affinity *desc)
{
	int i, ret, queue_idx = 0;

	for (i = 0; i < nvqs; ++i) {
		if (!names[i]) {
			vqs[i] = NULL;
			continue;
		}

		vqs[i] = rp_find_vq(vdev, queue_idx++, callbacks[i], names[i],
				    ctx ? ctx[i] : false);
		if (IS_ERR(vqs[i])) {
			ret = PTR_ERR(vqs[i]);
			goto error;
		}
	}

	return 0;

error:
	__rproc_virtio_del_vqs(vdev);
	return ret;
}

static u8 rproc_virtio_get_status(struct virtio_device *vdev)
{
	struct rproc_vdev *rvdev = vdev_to_rvdev(vdev);
	struct fw_rsc_vdev *rsc;

	rsc = (void *)rvdev->rproc->table_ptr + rvdev->rsc_offset;

	return rsc->status;
}

static void rproc_virtio_set_status(struct virtio_device *vdev, u8 status)
{
	struct rproc_vdev *rvdev = vdev_to_rvdev(vdev);
	struct fw_rsc_vdev *rsc;

	rsc = (void *)rvdev->rproc->table_ptr + rvdev->rsc_offset;

	rsc->status = status;
	dev_dbg(&vdev->dev, "status: %d\n", status);
}

static void rproc_virtio_reset(struct virtio_device *vdev)
{
	struct rproc_vdev *rvdev = vdev_to_rvdev(vdev);
	struct fw_rsc_vdev *rsc;

	rsc = (void *)rvdev->rproc->table_ptr + rvdev->rsc_offset;

	rsc->status = 0;
	dev_dbg(&vdev->dev, "reset !\n");
}

/* provide the vdev features as retrieved from the firmware */
static u64 rproc_virtio_get_features(struct virtio_device *vdev)
{
	struct rproc_vdev *rvdev = vdev_to_rvdev(vdev);
	struct fw_rsc_vdev *rsc;

	rsc = (void *)rvdev->rproc->table_ptr + rvdev->rsc_offset;

	return rsc->dfeatures;
}

static void rproc_transport_features(struct virtio_device *vdev)
{
	/*
	 * Packed ring isn't enabled on remoteproc for now,
	 * because remoteproc uses vring_new_virtqueue() which
	 * creates virtio rings on preallocated memory.
	 */
	__virtio_clear_bit(vdev, VIRTIO_F_RING_PACKED);
}

static int rproc_virtio_finalize_features(struct virtio_device *vdev)
{
	struct rproc_vdev *rvdev = vdev_to_rvdev(vdev);
	struct fw_rsc_vdev *rsc;

	rsc = (void *)rvdev->rproc->table_ptr + rvdev->rsc_offset;

	/* Give virtio_ring a chance to accept features */
	vring_transport_features(vdev);

	/* Give virtio_rproc a chance to accept features. */
	rproc_transport_features(vdev);

	/* Make sure we don't have any features > 32 bits! */
	BUG_ON((u32)vdev->features != vdev->features);

	/*
	 * Remember the finalized features of our vdev, and provide it
	 * to the remote processor once it is powered on.
	 */
	rsc->gfeatures = vdev->features;

	return 0;
}

static void rproc_virtio_get(struct virtio_device *vdev, unsigned int offset,
			     void *buf, unsigned int len)
{
	struct rproc_vdev *rvdev = vdev_to_rvdev(vdev);
	struct fw_rsc_vdev *rsc;
	void *cfg;

	rsc = (void *)rvdev->rproc->table_ptr + rvdev->rsc_offset;
	cfg = &rsc->vring[rsc->num_of_vrings];

	if (offset + len > rsc->config_len || offset + len < len) {
		dev_err(&vdev->dev, "rproc_virtio_get: access out of bounds\n");
		return;
	}

	memcpy(buf, cfg + offset, len);
}

static void rproc_virtio_set(struct virtio_device *vdev, unsigned int offset,
			     const void *buf, unsigned int len)
{
	struct rproc_vdev *rvdev = vdev_to_rvdev(vdev);
	struct fw_rsc_vdev *rsc;
	void *cfg;

	rsc = (void *)rvdev->rproc->table_ptr + rvdev->rsc_offset;
	cfg = &rsc->vring[rsc->num_of_vrings];

	if (offset + len > rsc->config_len || offset + len < len) {
		dev_err(&vdev->dev, "rproc_virtio_set: access out of bounds\n");
		return;
	}

	memcpy(cfg + offset, buf, len);
}

static const struct virtio_config_ops rproc_virtio_config_ops = {
	.get_features	= rproc_virtio_get_features,
	.finalize_features = rproc_virtio_finalize_features,
	.find_vqs	= rproc_virtio_find_vqs,
	.del_vqs	= rproc_virtio_del_vqs,
	.reset		= rproc_virtio_reset,
	.set_status	= rproc_virtio_set_status,
	.get_status	= rproc_virtio_get_status,
	.get		= rproc_virtio_get,
	.set		= rproc_virtio_set,
};

/*
 * This function is called whenever vdev is released, and is responsible
 * to decrement the remote processor's refcount which was taken when vdev was
 * added.
 *
 * Never call this function directly; it will be called by the driver
 * core when needed.
 */
static void rproc_virtio_dev_release(struct device *dev)
{
	struct virtio_device *vdev = dev_to_virtio(dev);
	struct rproc_vdev *rvdev = vdev_to_rvdev(vdev);

	kfree(vdev);

	put_device(&rvdev->pdev->dev);
}

/**
 * rproc_add_virtio_dev() - register an rproc-induced virtio device
 * @rvdev: the remote vdev
 * @id: the device type identification (used to match it with a driver).
 *
 * This function registers a virtio device. This vdev's partent is
 * the rproc device.
 *
 * Return: 0 on success or an appropriate error value otherwise
 */
static int rproc_add_virtio_dev(struct rproc_vdev *rvdev, int id)
{
	struct rproc *rproc = rvdev->rproc;
	struct device *dev = &rvdev->pdev->dev;
	struct virtio_device *vdev;
	struct rproc_mem_entry *mem;
	int ret;

	if (!rproc->ops->kick && !rvdev->mbox) {
		ret = -EINVAL;
		dev_err(dev, ".kick method not defined for %s\n", rproc->name);
		goto out;
	}

	/* Try to find dedicated vdev buffer carveout */
	mem = rproc_find_carveout_by_name(rproc, "vdev%dbuffer", rvdev->index);
	if (mem) {
		phys_addr_t pa;

		if (mem->of_resm_idx != -1) {
			struct device_node *np = mem->dev->of_node;

			/* Associate reserved memory to vdev device */
			ret = of_reserved_mem_device_init_by_idx(dev, np,
								 mem->of_resm_idx);
			if (ret) {
				dev_err(dev, "Can't associate reserved memory\n");
				goto out;
			}
		} else {
			if (mem->va) {
				dev_warn(dev, "vdev %d buffer already mapped\n",
					 rvdev->index);
				pa = rproc_va_to_pa(mem->va);
			} else {
				/* Use dma address as carveout no memmapped yet */
				pa = (phys_addr_t)mem->dma;
			}

			/* Associate vdev buffer memory pool to vdev subdev */
			ret = dma_declare_coherent_memory(dev, pa,
							   mem->da,
							   mem->len);
			if (ret < 0) {
				dev_err(dev, "Failed to associate buffer\n");
				goto out;
			}
		}
	} else {
		struct device_node *np = rproc->dev.parent->of_node;

		/*
		 * If we don't have dedicated buffer, just attempt to re-assign
		 * the reserved memory from our parent. A default memory-region
		 * at index 0 from the parent's memory-regions is assigned for
		 * the rvdev dev to allocate from. Failure is non-critical and
		 * the allocations will fall back to global pools, so don't
		 * check return value either.
		 */
		dev_err(dev, "use parent memory!!!!!\n");
		of_reserved_mem_device_init_by_idx(dev, np, 0);
	}

	/* Allocate virtio device */
	vdev = kzalloc(sizeof(*vdev), GFP_KERNEL);
	if (!vdev) {
		ret = -ENOMEM;
		goto out;
	}
	vdev->id.device	= id,
	vdev->config = &rproc_virtio_config_ops,
	vdev->dev.parent = dev;
	vdev->dev.release = rproc_virtio_dev_release;

	/* Reference the vdev and vring allocations */
	get_device(dev);

	ret = register_virtio_device(vdev);
	if (ret) {
		put_device(&vdev->dev);
		dev_err(dev, "failed to register vdev: %d\n", ret);
		goto out;
	}

	dev_info(dev, "registered %s (type %d)\n", dev_name(&vdev->dev), id);

out:
	return ret;
}

/**
 * rproc_remove_virtio_dev() - remove an rproc-induced virtio device
 * @dev: the virtio device
 * @data: must be null
 *
 * This function unregisters an existing virtio device.
 *
 * Return: 0
 */
static int rproc_remove_virtio_dev(struct device *dev, void *data)
{
	struct virtio_device *vdev = dev_to_virtio(dev);

	unregister_virtio_device(vdev);
	return 0;
}

static int rproc_vdev_do_start(struct rproc_subdev *subdev)
{
	struct rproc_vdev *rvdev = container_of(subdev, struct rproc_vdev, subdev);

	return rproc_add_virtio_dev(rvdev, rvdev->id);
}

static void rproc_vdev_do_stop(struct rproc_subdev *subdev, bool crashed)
{
	struct rproc_vdev *rvdev = container_of(subdev, struct rproc_vdev, subdev);
	struct device *dev = &rvdev->pdev->dev;
	int ret;

	ret = device_for_each_child(dev, NULL, rproc_remove_virtio_dev);
	if (ret)
		dev_warn(dev, "can't remove vdev child device: %d\n", ret);
}

static int rproc_virtio_of_parse_mem(struct rproc_vdev *rvdev)
{
	struct device *dev = &rvdev->pdev->dev;
	struct device_node *np = dev->of_node;
	struct rproc *rproc = rvdev->rproc;
	struct of_phandle_iterator it;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;
	char name[32];
	u64 da;
	int index = 0;

	/* Register associated reserved memory regions */
	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);

	while (of_phandle_iterator_next(&it) == 0) {
		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			dev_err(dev, "unable to acquire memory-region\n");
			return -EINVAL;
		}

		if (rproc_pa_to_da(rproc, rmem->base, &da) < 0) {
			dev_err(dev, "memory region not valid %pa\n",
				&rmem->base);
			return -EINVAL;
		}

		snprintf(name, sizeof(name), "vdev%dbuffer", rvdev->index);
		if (strcmp(it.node->name, name)) {
			/* Register memory region */
			mem = rproc_platform_mem_entry_init(dev, rproc, NULL,
							    (dma_addr_t)rmem->base,
							    rmem->size, da, it.node->name);

		} else {
			/* Register reserved memory for vdev buffer alloc */
			mem = rproc_of_resm_mem_entry_init(dev, index, rmem->size,
							   rmem->base, it.node->name);
		}

		if (!mem)
			return -ENOMEM;

		rproc_add_carveout(rproc, mem);
		index++;
	}
	return 0;
}

static int rproc_virtio_bind(struct rproc_vdev *rvdev, struct fw_rsc_vdev *rsc, int rsc_offset)
{
	struct rproc *rproc = rvdev->rproc;
	struct device *dev = &rvdev->pdev->dev;
	int id, ret;

	rvdev->rsc_offset = rsc_offset;
	rvdev->subdev.start = rproc_vdev_do_start;
	rvdev->subdev.stop = rproc_vdev_do_stop;

	ret = rproc_virtio_of_parse_mem(rvdev);
	if (ret)
		return ret;

	/* parse the vrings */
	for (id = 0; id < rsc->num_of_vrings; id++) {
		ret = rproc_parse_vring(rvdev, rsc, id);
		if (ret)
			return ret;
	}

	/* remember the resource offset*/
	rvdev->rsc_offset = rsc_offset;

	/* allocate the vring resources */
	for (id = 0; id < rsc->num_of_vrings; id++) {
		ret = rproc_alloc_vring(rvdev, id);
		if (ret)
			goto unwind_vring_allocations;
		if (rvdev->mbox)
			if (id < rvdev->nb_mbox)
				rvdev->mbox[id].notifyid = rvdev->vring[id].notifyid;
	}

	rproc_add_subdev(rproc, &rvdev->subdev);

	/*
	 * We're indirectly making a non-temporary copy of the rproc pointer
	 * here, because the platform devicer or the vdev device will indirectly
	 * access the wrapping rproc.
	 *
	 * Therefore we must increment the rproc refcount here, and decrement
	 * it _only_ on platform remove.
	 */
	get_device(&rproc->dev);

	rvdev->bound = 1;

	dev_dbg(dev, "remote proc virtio dev %d bound\n",  rvdev->index);

	return 0;

unwind_vring_allocations:
	for (id--; id >= 0; id--) {
		rproc_free_vring(&rvdev->vring[id]);
		if (rvdev->mbox)
			if (id < rvdev->nb_mbox)
				rvdev->mbox[id].notifyid = -1;
	}
	return ret;
}

static void rproc_virtio_unbind(struct rproc_vdev *rvdev)
{
	struct rproc *rproc = rvdev->rproc;
	struct device *dev = &rvdev->pdev->dev;
	int id;

	if (!rvdev->bound)
		return;

	rproc_remove_subdev(rproc, &rvdev->subdev);
	rvdev->bound = 0;

	for (id = 0; id < ARRAY_SIZE(rvdev->vring); id++) {
		rproc_free_vring(&rvdev->vring[id]);
		if (rvdev->mbox)
			if (id < rvdev->nb_mbox)
				rvdev->mbox[id].notifyid = -1;
	}
	/* The remote proc device can be removed */
	put_device(&rproc->dev);

	dev_dbg(dev, "remote proc virtio dev %d unbound\n",  rvdev->index);
}

static void rproc_virtio_mb_vq_work(struct work_struct *work)
{
	struct rproc_vdev_mbox *mb = container_of(work, struct rproc_vdev_mbox, vq_work);
	struct rproc_vdev *rvdev = dev_get_drvdata(mb->cl.dev);
	struct rproc *rproc = rvdev->rproc;

	if (rproc_vq_interrupt(rproc, mb->notifyid) == IRQ_NONE)
		dev_dbg(&rproc->dev, "no message found in vq\n");
}

static void rproc_virtio_mb_callback(struct mbox_client *cl, void *data)
{
	struct rproc_vdev *rvdev = dev_get_drvdata(cl->dev);
	struct rproc_vdev_mbox *mb = container_of(cl, struct rproc_vdev_mbox, cl);

	queue_work(rvdev->mbox_wq, &mb->vq_work);
}

static void rproc_virtio_free_mbox(struct rproc_vdev *rvdev)
{
	unsigned int i;

	for (i = 0; i < rvdev->nb_mbox; i++) {
		if (rvdev->mbox[i].chan)
			mbox_free_channel(rvdev->mbox[i].chan);
		rvdev->mbox[i].chan = NULL;
	}
}

const struct mbox_client rproc_virtio_client = {
	.rx_callback = rproc_virtio_mb_callback,
	.tx_block = false,
};

static int rproc_virtio_of_parse(struct device *dev, struct rproc_vdev *rvdev)
{
	struct device_node *np = dev->of_node;
	struct rproc_vdev_mbox *mbox;
	int ret, i;

	/* The reg is used to specify the vdev index */
	if (of_property_read_u32(np, "reg", &rvdev->index))
		return -EINVAL;

	/* The virtio,id define the virtio type */
	if (of_property_read_u32(np, "virtio,id", &rvdev->id))
		return -EINVAL;

	/* check for the mailboxes */

	/* Register associated reserved memory regions */
	rvdev->nb_mbox = of_count_phandle_with_args(np, "mboxes", "#mbox-cells");

	dev_dbg(dev, "%pOF: nb mailbox found: %d\n", np, rvdev->nb_mbox);

	if (rvdev->nb_mbox == -ENOENT)
		return 0;

	rvdev->mbox = devm_kcalloc(dev, rvdev->nb_mbox, sizeof(struct rproc_vdev_mbox),
				   GFP_KERNEL);
	if (!rvdev->mbox)
		return -ENOMEM;

	rvdev->mbox_wq = create_workqueue(dev_name(dev));
	if (!rvdev->mbox_wq) {
		dev_err(dev, "cannot create workqueue\n");
		return -ENOMEM;
	}

	for (i = 0; i < rvdev->nb_mbox; i++) {
		mbox = &rvdev->mbox[i];
		mbox->name = devm_kasprintf(dev, GFP_KERNEL, "vq%d", i);

		mbox->cl = rproc_virtio_client;
		mbox->cl.dev = dev;

		mbox->chan = mbox_request_channel_byname(&mbox->cl, mbox->name);
		if (IS_ERR(mbox->chan)) {
			dev_err_probe(dev->parent, PTR_ERR(mbox->chan),
				      "failed to request mailbox %s\n", mbox->name);
			ret = PTR_ERR(mbox->chan);
			goto err_get_mbox;
		}
		/*
		 * Create a workqueue as nothing prevent that rproc_virtio_mb_callback is called
		 * under interrupt context.
		 */
		dev_dbg(dev, "%s: mailbox associated: %s\n", __func__, mbox->name);
		INIT_WORK(&mbox->vq_work, rproc_virtio_mb_vq_work);
		mbox->notifyid = -1;
	}

	return 0;

err_get_mbox:
	rproc_virtio_free_mbox(rvdev);
	destroy_workqueue(rvdev->mbox_wq);

	return ret;
}

static int rproc_virtio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rproc_vdev *rvdev;
	struct rproc *rproc = container_of(dev->parent, struct rproc, dev);
	int ret;

	rvdev = devm_kzalloc(dev, sizeof(*rvdev), GFP_KERNEL);
	if (!rvdev)
		return -ENOMEM;

	if (dev->of_node) {
		/*
		 * The platform device is declared in the device tree
		 * retrieve rproc struct through the remoteproc platform
		 */
		rproc = rproc_get_by_node(dev->parent->of_node);

		ret = rproc_virtio_of_parse(dev, rvdev);
		if (ret)
			return ret;
	} else {
		struct rproc_vdev_data *rvdev_data = pdev->dev.platform_data;

		if (!rvdev_data)
			return -EINVAL;

		rproc = container_of(dev->parent, struct rproc, dev);

		rvdev->id = rvdev_data->id;
		rvdev->index = rvdev_data->index;
	}

	rvdev->rproc = rproc;

	ret = copy_dma_range_map(dev, rproc->dev.parent);
	if (ret)
		return ret;

	/* Make device dma capable by inheriting from parent's capabilities */
	set_dma_ops(dev, get_dma_ops(rproc->dev.parent));

	ret = dma_coerce_mask_and_coherent(dev, dma_get_mask(rproc->dev.parent));
	if (ret) {
		dev_warn(dev, "Failed to set DMA mask %llx. Trying to continue... (%pe)\n",
			 dma_get_mask(rproc->dev.parent), ERR_PTR(ret));
	}

	platform_set_drvdata(pdev, rvdev);
	rvdev->pdev = pdev;


	rproc_add_rvdev(rproc, rvdev);

	rvdev->bind = rproc_virtio_bind;
	rvdev->unbind = rproc_virtio_unbind;

	return 0;
}

static int rproc_virtio_remove(struct platform_device *pdev)
{
	struct rproc_vdev *rvdev = dev_get_drvdata(&pdev->dev);

	if (rvdev->bound)
		rproc_virtio_unbind(rvdev);

	if (rvdev->nb_mbox)
		rproc_virtio_free_mbox(rvdev);

	if (rvdev->mbox_wq)
		destroy_workqueue(rvdev->mbox_wq);

	rproc_remove_rvdev(rvdev);

	of_reserved_mem_device_release(&pdev->dev);
	dma_release_coherent_memory(&pdev->dev);


	return 0;
}

/* Platform driver */
static const struct of_device_id rproc_virtio_match[] = {
	{ .compatible = "virtio,rproc" },
	{},
};

static struct platform_driver rproc_virtio_driver = {
	.probe		= rproc_virtio_probe,
	.remove		= rproc_virtio_remove,
	.driver		= {
		.name	= "rproc-virtio",
		.of_match_table	= rproc_virtio_match,
	},
};
builtin_platform_driver(rproc_virtio_driver);
