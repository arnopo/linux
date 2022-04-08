/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _LINUX_RPMSG_FC_H
#define _LINUX_RPMSG_FC_H

#include <linux/mod_devicetable.h>
#include <linux/rpmsg.h>
#include <linux/rpmsg/byteorder.h>
#include <linux/types.h>

/**
 * enum rpmsg_ns_flags - dynamic endpoint announcement flags
 *
 * @RPMSG_EPT_BOUND: a new remote service was just created
 * @RPMSG_EPT_UNBOUND: a known remote service was just destroyed
 */
enum rpmsg_ept_flags {
	RPMSG_EPT_OFF	= 0,
	RPMSG_EPT_ON	= 1,
};

/**
 * struct rpmsg_ept_msg - dynamic endpoint announcement message
 * @src: address of the endpoint that sned the message
 * @dest: address of the destination endpoint.
 * @flags: indicates the state of the endpoint based on @rpmsg_ept_flags enum.
 *
 * This message is sent across to inform the remote about the state of a local
 * endpoint associated with a remote endpoint:
 * - a RPMSG_EPT_OFF can be send to inform that a local endpoint is suspended.
 * - a RPMSG_EPT_ON can be send to inform that a local endpoint is ready to communicate.
 *
 * When we receive these messages, the appropriate endpoint is informed.
 */
struct rpmsg_ept_msg {
	__rpmsg32 src;
	__rpmsg32 dst;
	__rpmsg32 flags;
} __packed;

/* Address 54 is reserved for flow control advertising */
#define RPMSG_FC_ADDR                   (54)

#if IS_ENABLED(CONFIG_RPMSG_FC)

int rpmsg_fc_register_device(struct rpmsg_device *rpdev);

#else

int rpmsg_fc_register_device(struct rpmsg_device *rpdev)
{
	/* This shouldn't be possible */
	WARN_ON(1);

	return -ENXIO;
}
#endif /* IS_ENABLED(CONFIG_RPMSG_FC)*/

#endif
