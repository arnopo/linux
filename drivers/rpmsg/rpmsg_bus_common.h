/* SPDX-License-Identifier: GPL-2.0 */
/*
 * remote processor messaging bus common part between all busses
 *
 * Copyright (C) 2019 ST Microelectronics
 *
 * Maxime Mere <maxime.mere@st.com>
 */

#ifndef __RPMSG_BUS_COMMON_H__
#define __RPMSG_BUS_COMMON_H__

#include <linux/idr.h>
#include <linux/mutex.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "rpmsg_internal.h"

typedef struct rpmsg_device *(*create_chnl_t)(void *prv,
					      struct rpmsg_channel_info *cinfo);

/**
 * struct rpmsg_hdr - common header for all rpmsg messages
 * @src: source address
 * @dst: destination address
 * @reserved: reserved for future use
 * @len: length of payload (in bytes)
 * @flags: message flags
 * @data: @len bytes of message payload data
 *
 * Every message sent(/received) on the rpmsg bus begins with this header.
 */
struct rpmsg_hdr {
	u32 src;
	u32 dst;
	u32 reserved;
	u16 len;
	u16 flags;
	u8 data[0];
} __packed;

/**
 * struct rpmsg_ns_msg - dynamic name service announcement message
 * @name: name of remote service that is published
 * @addr: address of remote service that is published
 * @flags: indicates whether service is created or destroyed
 *
 * This message is sent across to publish a new service, or announce
 * about its removal. When we receive these messages, an appropriate
 * rpmsg channel (i.e device) is created/destroyed. In turn, the ->probe()
 * or ->remove() handler of the appropriate rpmsg driver will be invoked
 * (if/as-soon-as one is registered).
 */
struct rpmsg_ns_msg {
	char name[RPMSG_NAME_SIZE];
	u32 addr;
	u32 flags;
} __packed;

/**
 * enum rpmsg_ns_flags - dynamic name service announcement flags
 *
 * @RPMSG_NS_CREATE: a new remote service was just created
 * @RPMSG_NS_DESTROY: a known remote service was just destroyed
 */
enum rpmsg_ns_flags {
	RPMSG_NS_CREATE		= 0,
	RPMSG_NS_DESTROY	= 1,
};

/*
 * Local addresses are dynamically allocated on-demand.
 * We do not dynamically assign addresses from the low 1024 range,
 * in order to reserve that address range for predefined services.
 */
#define RPMSG_RESERVED_ADDRESSES	(1024)

/* Address 53 is reserved for advertising remote services */
#define RPMSG_NS_ADDR			(53)

struct rpmsg_endpoint*
rpmsg_bus_create_ept(struct idr *endpoints,
		     const struct rpmsg_endpoint_ops *ops,
		     struct rpmsg_device *rpdev, rpmsg_rx_cb_t cb, void *priv,
		     u32 addr);

void rpmsg_bus_destroy_ept(struct idr *endpoints, struct rpmsg_endpoint *ept);

int rpmsg_bus_announce_create(struct rpmsg_device *rpdev, bool has_feature);
int rpmsg_bus_announce_destroy(struct rpmsg_device *rpdev, bool has_feature);

void rpmsg_bus_recv_single(struct mutex *endpoints_lock, struct idr *endpoints,
			   struct device *dev, struct rpmsg_hdr *msg,
			   unsigned int len);

int rpmsg_bus_ns_cb(struct device *dev, struct rpmsg_device *rpdev,
		    void *data, int len, void *priv, u32 src,
		    create_chnl_t create_chnl);

#endif
