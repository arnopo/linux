// SPDX-License-Identifier: GPL-2.0
/*
 * Common part of all rpmsg busses
 *
 * Copyright (C) 2019 ST Microelectronics
 *
 * Maxime Mere <maxime.mere@st.com>
 */
#include "rpmsg_bus_common.h"

/**
 * __ept_release() - deallocate an rpmsg endpoint
 * @kref: the ept's reference count
 *
 * This function deallocates an ept, and is invoked when its @kref refcount
 * drops to zero.
 *
 * Never invoke this function directly!
 */
static void __ept_release(struct kref *kref)
{
	struct rpmsg_endpoint *ept = container_of(kref, struct rpmsg_endpoint,
						  refcount);
	/*
	 * At this point no one holds a reference to ept anymore,
	 * so we can directly free it
	 */
	kfree(ept);
}

/* For more info, see below documentation of rpmsg_create_ept() */
struct rpmsg_endpoint*
rpmsg_bus_create_ept(struct idr *endpoints,
		     const struct rpmsg_endpoint_ops *ops,
		     struct rpmsg_device *rpdev, rpmsg_rx_cb_t cb, void *priv,
		     u32 addr)
{
	int id_min, id_max, id;
	struct rpmsg_endpoint *ept;

	ept = kzalloc(sizeof(*ept), GFP_KERNEL);
	if (!ept)
		return NULL;

	kref_init(&ept->refcount);
	mutex_init(&ept->cb_lock);

	ept->rpdev = rpdev;
	ept->cb = cb;
	ept->priv = priv;
	ept->ops = ops;

	/* Do we need to allocate a local address ? */
	if (addr == RPMSG_ADDR_ANY) {
		id_min = RPMSG_RESERVED_ADDRESSES;
		id_max = 0;
	} else {
		id_min = addr;
		id_max = addr + 1;
	}

	/* Bind the endpoint to an rpmsg address (and allocate one if needed) */
	id = idr_alloc(endpoints, ept, id_min, id_max, GFP_KERNEL);
	if (id < 0) {
		if (rpdev)
			dev_err(&rpdev->dev, "idr_alloc failed: %d\n", id);
		goto free_ept;
	}
	ept->addr = id;

	return ept;

free_ept:
	kref_put(&ept->refcount, __ept_release);
	return NULL;
}

/**
 * rpmsg_bus_destroy_ept() - destroy an existing rpmsg endpoint
 * @vrp: virtproc which owns this ept
 * @ept: endpoing to destroy
 *
 * An internal function which destroy an ept without assuming it is
 * bound to an rpmsg channel. This is needed for handling the internal
 * name service endpoint, which isn't bound to an rpmsg channel.
 * See also rpmsg_bus_create_ept().
 */
void rpmsg_bus_destroy_ept(struct idr *endpoints, struct rpmsg_endpoint *ept)
{
	/* make sure new inbound messages can't find this ept anymore */
	idr_remove(endpoints, ept->addr);

	/* make sure in-flight inbound messages won't invoke cb anymore */
	mutex_lock(&ept->cb_lock);
	ept->cb = NULL;
	mutex_unlock(&ept->cb_lock);

	kref_put(&ept->refcount, __ept_release);
}

int rpmsg_bus_announce_create(struct rpmsg_device *rpdev, bool has_feature)
{
	struct device *dev = &rpdev->dev;
	int err = 0;

	/* need to tell remote processor's name service about this channel ? */
	if (rpdev->announce && rpdev->ept && has_feature) {
		struct rpmsg_ns_msg nsm;

		strncpy(nsm.name, rpdev->id.name, RPMSG_NAME_SIZE);
		nsm.addr = rpdev->ept->addr;
		nsm.flags = RPMSG_NS_CREATE;

		err = rpmsg_sendto(rpdev->ept, &nsm, sizeof(nsm),
				   RPMSG_NS_ADDR);
		if (err)
			dev_err(dev, "failed to announce service %d\n", err);
	}

	return err;
}

int rpmsg_bus_announce_destroy(struct rpmsg_device *rpdev, bool has_feature)
{
	struct device *dev = &rpdev->dev;
	int err = 0;

	/* tell remote processor's name service we're removing this channel */
	if (rpdev->announce && rpdev->ept && has_feature) {
		struct rpmsg_ns_msg nsm;

		strncpy(nsm.name, rpdev->id.name, RPMSG_NAME_SIZE);
		nsm.addr = rpdev->ept->addr;
		nsm.flags = RPMSG_NS_DESTROY;

		err = rpmsg_sendto(rpdev->ept, &nsm, sizeof(nsm),
				   RPMSG_NS_ADDR);
		if (err)
			dev_err(dev, "failed to announce service %d\n", err);
	}

	return err;
}

void rpmsg_bus_recv_single(struct mutex *endpoints_lock, struct idr *endpoints,
			   struct device *dev, struct rpmsg_hdr *msg,
			   unsigned int len)
{
	struct rpmsg_endpoint *ept;

	/* Use the dst addr to fetch the callback of the appropriate user */
	mutex_lock(endpoints_lock);

	ept = idr_find(endpoints, msg->dst);

	/* Let's make sure no one deallocates ept while we use it */
	if (ept)
		kref_get(&ept->refcount);

	mutex_unlock(endpoints_lock);

	if (ept) {
		/* Make sure ept->cb doesn't go away while we use it */
		mutex_lock(&ept->cb_lock);

		if (ept->cb)
			ept->cb(ept->rpdev, msg->data, msg->len, ept->priv,
				msg->src);

		mutex_unlock(&ept->cb_lock);

		/* farewell, ept, we don't need you anymore */
		kref_put(&ept->refcount, __ept_release);
	} else {
		dev_warn(dev, "msg received with no recipient\n");
	}
}

int rpmsg_bus_ns_cb(struct device *dev, struct rpmsg_device *rpdev,
		    void *data, int len, void *priv, u32 src,
		    create_chnl_t create_chnl)
{
	struct rpmsg_ns_msg *msg = data;
	struct rpmsg_device *newch;
	struct rpmsg_channel_info chinfo;
	int ret;

#if defined(CONFIG_DYNAMIC_DEBUG)
	dynamic_hex_dump("NS announcement: ", DUMP_PREFIX_NONE, 16, 1,
			 data, len, true);
#endif

	if (len != sizeof(*msg)) {
		dev_err(dev, "malformed ns msg (%d)\n", len);
		return -EINVAL;
	}

	/*
	 * the name service ept does _not_ belong to a real rpmsg channel,
	 * and is handled by the rpmsg bus itself.
	 * for sanity reasons, make sure a valid rpdev has _not_ sneaked
	 * in somehow.
	 */
	if (rpdev) {
		dev_err(dev, "anomaly: ns ept has an rpdev handle\n");
		return -EINVAL;
	}

	/* don't trust the remote processor for null terminating the name */
	msg->name[RPMSG_NAME_SIZE - 1] = '\0';

	dev_info(dev, "%sing channel %s addr 0x%x\n",
		 msg->flags & RPMSG_NS_DESTROY ? "destroy" : "creat",
		 msg->name, msg->addr);

	strncpy(chinfo.name, msg->name, sizeof(chinfo.name));
	chinfo.src = RPMSG_ADDR_ANY;
	chinfo.dst = msg->addr;

	if (msg->flags & RPMSG_NS_DESTROY) {
		ret = rpmsg_unregister_device(dev, &chinfo);
		if (ret)
			dev_err(dev, "rpmsg_destroy_channel failed: %d\n", ret);
	} else {
		newch = create_chnl(priv, &chinfo);
		if (!newch)
			dev_err(dev, "rpmsg_create_channel failed\n");
	}

	return 0;
}
