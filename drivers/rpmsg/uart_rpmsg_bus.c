// SPDX-License-Identifier: GPL-2.0
/*
 * Serdev based remote processor messaging bus
 *
 * Copyright (C) STMicroelectronics 2019
 * Author: Maxime MERE for STMicroelectronics.
 *
 * Based on virtio_rpmsg_bus.c
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/idr.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/rpmsg.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "rpmsg_internal.h"
#include "rpmsg_bus_common.h"

/**
 * struct buffer_manager - buffer manager instance structure
 * @rx_raw_tail:	pointer to the last receive raw byte
 * @rx_raw_left:	byte left inside the raw buffer
 * @rx_raw_buffer:	raw data buffer
 * @flag_msg_recv:	set when a message header is received
 * @first_byte_rx:	flag used when a magic number is detected
 * @buf_size:		size of the rx buffer
 * @msg_len:		data size received by serdev device
 * @rbuf:		address of rx buffers
 *
 * This structure aims to manage data before receiving or sending it. The idea
 * is a whole new service wich then is going to be used as adapter to permit
 * multiples rpmsg busses types.
 */
struct buffer_manager {
	unsigned char *rx_raw_tail;
	int rx_raw_left;
	unsigned char *rx_raw_buffer;
	uint8_t flag_msg_recv;
	uint8_t first_byte_rx;
	unsigned int buf_size;

	int msg_len;

	void *rbuf;
};

/**
 * struct serdev_info - virtual remote processor state
 * @serdev:	the serdev device
 * @bm:		buffer_manager instance
 * @endpoints:	idr of local endpoints, allows fast retrieval
 * @endpoints_lock: lock of the endpoints set
 * @ns_ept:	the bus's name service endpoint
 *
 * This structure stores the rpmsg state of a given serial device (there might
 * be several rpmsg_bus for each physical remote processor).
 */
struct serdev_info {
	struct serdev_device *serdev;
	struct buffer_manager bm;
	struct idr endpoints;
	struct mutex endpoints_lock;
	struct rpmsg_endpoint *ns_ept;
};

/* The rpmsg feature */
#define RPMSG_F_NS_SUPPORT	(1) /* we supports name service notifications */

/*The magic number used to identify rpmsg communications through uart*/
#define	SERDEV_RPMSG_MAGIC_NUMBER	(0xbe57)

/**
 * struct serdev_rpmsg_channel
 * @rpdev: rpmsg device handler
 * @srp: the remote processor this channel belongs to
 */
struct serdev_rpmsg_channel {
	struct rpmsg_device rpdev;
	struct serdev_info *srp;
};

#define to_serdev_rpmsg_channel(_rpdev) \
	container_of(_rpdev, struct serdev_rpmsg_channel, rpdev)

/*
 * We're allocating buffers of 512 bytes each for communications.
 *
 * Each buffer will have 16 bytes for the msg header and 496 bytes for
 * the payload.
 */
#define MAX_RPMSG_BUF_SIZE	(512)

/**
 * uart_rpmsg_send_off_chnl_raw() - send a message across the remote processor
 * @rpdev: the rpmsg channel
 * @src: source address
 * @dst: destination address
 * @data: payload of message
 * @len: length of payload
 * @wait: indicates whether caller should block while the data aren't sent in
 * the serial line
 *
 * This function is the base implementation for all of the rpmsg sending API.
 *
 * It will send @data of length @len to @dst, and say it's from @src. The
 * message will be sent to the remote processor which the @rpdev channel
 * belongs to.
 *
 * The message is sent via the uart link there is no buffer like in the virtio
 * bus.
 *
 * Normally drivers shouldn't use this function directly; instead, drivers
 * should use the appropriate rpmsg_{try}send{to, _offchannel} API
 * (see include/linux/rpmsg.h).
 *
 * Returns 0 on success and an appropriate error value on failure.
 */
static int uart_rpmsg_send_off_chnl_raw(struct rpmsg_device *rpdev,
					u32 src, u32 dst,
					void *data, int len, bool wait)
{
	struct serdev_rpmsg_channel *sch = to_serdev_rpmsg_channel(rpdev);
	struct serdev_info *srp = sch->srp;
	struct buffer_manager *bm = &srp->bm;
	struct serdev_device *serdev = srp->serdev;
	struct device *dev = &rpdev->dev;
	struct rpmsg_hdr *msg;
	int msg_size = sizeof(struct rpmsg_hdr) + len;
	int ret, timeout;
	u16 magic_number = SERDEV_RPMSG_MAGIC_NUMBER;

	/* bcasting isn't allowed */
	if (src == RPMSG_ADDR_ANY || dst == RPMSG_ADDR_ANY) {
		dev_err(dev, "invalid addr (src 0x%x, dst 0x%x)\n", src, dst);
		return -EINVAL;
	}

	/*
	 * We currently use fixed-sized buffers, and therefore the payload
	 * length is limited.
	 *
	 * One of the possible improvements here is either to support
	 * user-provided buffers (and then we can also support zero-copy
	 * messaging), or to improve the buffer allocator, to support
	 * variable-length buffer sizes.
	 */
	if (len > bm->buf_size - sizeof(struct rpmsg_hdr)) {
		dev_err(dev, "message is too big (%d)\n", len);
		return -EMSGSIZE;
	}

	/*
	 * With uart there is no shared memory so there is no need of buffers
	 * we're allocating memory to a pointer wich will be used by a serdev
	 * instead. Moreover we don't need to wait for a buffer.
	 */
	msg = kzalloc(msg_size, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	/*
	 * If wait is enabled timeout is set to zero to wait indefinitely.
	 * Otherwise timeout is set to 15 sec
	 */
	wait ? timeout = 0 : (timeout = msecs_to_jiffies(15000));

	msg->len = len;
	msg->src = src;
	msg->dst = dst;
	memcpy(msg->data, data, (size_t)len);

	dynamic_hex_dump("rpmsg_uart TX: ", DUMP_PREFIX_NONE, 16, 1,
			 &magic_number, sizeof(magic_number), true);
	/*
	 * The magic number is sent first to indicate what kind and how
	 * much data the remote proc is going to receive.
	 */
	ret = serdev_device_write(serdev, (uint8_t *)&magic_number,
				  sizeof(magic_number), timeout);
	if (ret != sizeof(magic_number)) {
		dev_err(&serdev->dev, "uart_rpmsg_send failed: %d\n", ret);
		goto err_send;
	}

	dynamic_hex_dump("rpmsg_uart TX: ", DUMP_PREFIX_NONE, 16, 1,
			 msg, msg_size, true);
	/* Then the RPMSG message itself is sent */
	ret = serdev_device_write(serdev, (uint8_t *)msg, msg_size, timeout);
	if (ret != msg_size)
		dev_err(&serdev->dev, "uart_rpmsg_send failed: %d\n", ret);
	else
		return 0;

err_send:
	kfree(msg);
	return ret < 0 ? ret : -EIO;
}

static int uart_rpmsg_send(struct rpmsg_endpoint *ept, void *data, int len)
{
	struct rpmsg_device *rpdev = ept->rpdev;
	u32 src = ept->addr, dst = rpdev->dst;

	return uart_rpmsg_send_off_chnl_raw(rpdev, src, dst, data, len, true);
}

static int uart_rpmsg_sendto(struct rpmsg_endpoint *ept, void *data, int len,
			     u32 dst)
{
	struct rpmsg_device *rpdev = ept->rpdev;
	u32 src = ept->addr;

	return uart_rpmsg_send_off_chnl_raw(rpdev, src, dst, data, len, true);
}

static int uart_rpmsg_send_offchannel(struct rpmsg_endpoint *ept, u32 src,
				      u32 dst, void *data, int len)
{
	struct rpmsg_device *rpdev = ept->rpdev;

	return uart_rpmsg_send_off_chnl_raw(rpdev, src, dst, data, len, true);
}

static int uart_rpmsg_trysend(struct rpmsg_endpoint *ept, void *data, int len)
{
	struct rpmsg_device *rpdev = ept->rpdev;
	u32 src = ept->addr, dst = rpdev->dst;

	return uart_rpmsg_send_off_chnl_raw(rpdev, src, dst, data, len, false);
}

static int uart_rpmsg_trysendto(struct rpmsg_endpoint *ept, void *data,
				int len, u32 dst)
{
	struct rpmsg_device *rpdev = ept->rpdev;
	u32 src = ept->addr;

	return uart_rpmsg_send_off_chnl_raw(rpdev, src, dst, data, len, false);
}

static int uart_rpmsg_trysend_offchannel(struct rpmsg_endpoint *ept, u32 src,
					 u32 dst, void *data, int len)
{
	struct rpmsg_device *rpdev = ept->rpdev;

	return uart_rpmsg_send_off_chnl_raw(rpdev, src, dst, data, len, false);
}

static int uart_get_buffer_size(struct rpmsg_endpoint *ept)
{
	struct rpmsg_device *rpdev = ept->rpdev;
	struct serdev_rpmsg_channel *sch = to_serdev_rpmsg_channel(rpdev);
	struct buffer_manager *bm = &sch->srp->bm;

	return bm->buf_size;
}

static void uart_rpmsg_destroy_ept(struct rpmsg_endpoint *ept)
{
	struct serdev_rpmsg_channel *sch = to_serdev_rpmsg_channel(ept->rpdev);
	struct serdev_info *srp = sch->srp;

	mutex_lock(&srp->endpoints_lock);
	rpmsg_bus_destroy_ept(&srp->endpoints, ept);
	mutex_unlock(&srp->endpoints_lock);
}

static const struct rpmsg_endpoint_ops uart_endpoint_ops = {
	.destroy_ept = uart_rpmsg_destroy_ept,
	.send = uart_rpmsg_send,
	.sendto = uart_rpmsg_sendto,
	.send_offchannel = uart_rpmsg_send_offchannel,
	.trysend = uart_rpmsg_trysend,
	.trysendto = uart_rpmsg_trysendto,
	.trysend_offchannel = uart_rpmsg_trysend_offchannel,
	.get_buffer_size = uart_get_buffer_size,
};

static struct rpmsg_endpoint *uart_create_ept(struct rpmsg_device *rpdev,
					      rpmsg_rx_cb_t cb,
					      void *priv,
					      struct rpmsg_channel_info chinfo)
{
	struct serdev_rpmsg_channel *sch = to_serdev_rpmsg_channel(rpdev);
	struct serdev_info *srp = priv ? priv : sch->srp;

	struct rpmsg_endpoint *ret;

	mutex_lock(&srp->endpoints_lock);
	ret = rpmsg_bus_create_ept(&srp->endpoints, &uart_endpoint_ops, rpdev,
				   cb, priv, chinfo.src);
	mutex_unlock(&srp->endpoints_lock);

	return ret;
}

/* Announcement are made when a new rpmsg driver is probbed or removed */
static int uart_rpmsg_announce_create(struct rpmsg_device *rpdev)
{
	return rpmsg_bus_announce_create(rpdev, RPMSG_F_NS_SUPPORT);
}

static int uart_rpmsg_announce_destroy(struct rpmsg_device *rpdev)
{
	return rpmsg_bus_announce_destroy(rpdev, RPMSG_F_NS_SUPPORT);
}

static const struct rpmsg_device_ops uart_rpmsg_ops = {
	.create_ept = uart_create_ept,
	.announce_create = uart_rpmsg_announce_create,
	.announce_destroy = uart_rpmsg_announce_destroy,
};

static void uart_rpmsg_release_device(struct device *dev)
{
	struct rpmsg_device *rpdev = to_rpmsg_device(dev);
	struct serdev_rpmsg_channel *sch = to_serdev_rpmsg_channel(rpdev);

	kfree(sch);
}

/*
 * create an rpmsg channel using its name and address info.
 * this function will be used to create both static and dynamic
 * channels.
 */
static struct rpmsg_device *
uart_rpmsg_create_channel(void *priv, struct rpmsg_channel_info *chinfo)
{
	struct serdev_info *srp = priv;
	struct serdev_rpmsg_channel *sch;
	struct rpmsg_device *rpdev;
	struct device *tmp, *dev = &srp->serdev->dev;
	int ret;

	/* make sure a similar channel doesn't already exist */
	tmp = rpmsg_find_device(dev, chinfo);
	if (tmp) {
		/* decrement the matched device's refcount back */
		put_device(tmp);
		dev_err(dev, "channel %s:%x:%x already exist\n",
			chinfo->name, chinfo->src, chinfo->dst);
		return NULL;
	}

	sch = kzalloc(sizeof(*sch), GFP_KERNEL);
	if (!sch)
		return NULL;

	/* Link the channel to our srp */
	sch->srp = srp;

	/* Assign public information to the rpmsg_device */
	rpdev = &sch->rpdev;
	rpdev->src = chinfo->src;
	rpdev->dst = chinfo->dst;
	rpdev->ops = &uart_rpmsg_ops;

	/*
	 * rpmsg server channels has predefined local address (for now),
	 * and their existence needs to be announced remotely
	 */
	rpdev->announce = rpdev->src != RPMSG_ADDR_ANY;

	strncpy(rpdev->id.name, chinfo->name, RPMSG_NAME_SIZE);

	rpdev->dev.parent = &srp->serdev->dev;
	rpdev->dev.release = uart_rpmsg_release_device;
	ret = rpmsg_register_device(rpdev);
	if (ret) {
		kfree(sch);
		return NULL;
	}

	return rpdev;
}

/* Send data to the appropriate device */
static int uart_rpmsg_recv_single(struct serdev_info *srp, struct device *dev,
				  struct rpmsg_hdr *msg, unsigned int len)
{
	struct buffer_manager *bm = &srp->bm;

	dynamic_hex_dump("rpmsg_uart RX: ", DUMP_PREFIX_NONE, 16, 1,
			 msg, sizeof(*msg) + msg->len, true);

	/*
	 * We currently use fixed-sized buffers, so trivially sanitize
	 * the reported payload length.
	 */
	if (len > bm->buf_size ||
	    msg->len > (len - sizeof(struct rpmsg_hdr))) {
		dev_err(dev, "inbound msg too big: (%d, %d)\n", len, msg->len);
		return -EINVAL;
	}

	rpmsg_bus_recv_single(&srp->endpoints_lock, &srp->endpoints, dev, msg,
			      len);

	return 0;
}

/* Called when rx buffer is ready to be read. */
static void uart_rpmsg_recv_done(struct serdev_info *srp)
{
	struct buffer_manager *bm = &srp->bm;
	struct device dev = srp->serdev->dev;
	struct rpmsg_hdr *msg;

	msg = (struct rpmsg_hdr *)bm->rbuf;

	uart_rpmsg_recv_single(srp, &dev, msg, bm->msg_len);
}

/**
 * uart_rpmsg_append_data - return the count of data received or an error
 * @srp:    Serial device information handler
 * @dat:    data received form serdev link
 * @len:    number of data in byte
 */
static int
uart_rpmsg_append_data(struct serdev_info *srp, unsigned char *dat, size_t len)
{
	struct buffer_manager *bm = &srp->bm;
	struct rpmsg_hdr s_hdr;
	struct device dev = srp->serdev->dev;
	int *byte_left = &bm->rx_raw_left;
	int ret, i;
	unsigned int byte_stored = 0;
	size_t old_len = len;

	dynamic_hex_dump("rpmsg_uart RX: ", DUMP_PREFIX_NONE, 16, 1,
			 dat, len, true);
	if (!bm->first_byte_rx) {
		/* Searching for the first transmit byte */
		for (i = 0; i < old_len; i++) {
			if (*((uint16_t *)dat) == SERDEV_RPMSG_MAGIC_NUMBER) {
			/* When done, we start append process */
				bm->first_byte_rx = true;

				/* We don't need store the magic number */
				len -= 2;
				dat += 2;

				memcpy(bm->rx_raw_tail, dat, len);
				*byte_left -= len;
				bm->rx_raw_tail += len;
				break;
			}
			/* In case non valid data we trash it */
			len--;
			dat++;
		}
	} else {
		/* Usual case of data reception */
		memcpy(bm->rx_raw_tail, dat, len);
		*byte_left -= len;
		bm->rx_raw_tail += len;
	}

	byte_stored = bm->buf_size - *byte_left;
	if (!bm->flag_msg_recv && byte_stored >= sizeof(s_hdr)) {
		/* Check in the header to know how many data we're waiting */
		memcpy(&s_hdr, bm->rx_raw_buffer, sizeof(s_hdr));
		if (s_hdr.len > bm->buf_size) {
			dev_err(&dev, "msg length too high");
			ret = -ENOMEM;
			goto err_bm;
		}
		/* byte_left will be now equal to the size of the rpmsg msg
		 * minus the byte already received
		 */
		*byte_left = s_hdr.len - (byte_stored - sizeof(s_hdr));
		bm->msg_len = s_hdr.len + sizeof(s_hdr);
		bm->flag_msg_recv = true;
	}

	/* byte_left can't be negative */
	/* Enter in this situation means that we received more data that the msg
	 * is supposed to have.
	 * For now the driver is not able to receive a second message while the
	 * first one isn't fully treated.
	 */
	if (*byte_left <= 0) {
		/* The message is complete, we can copy it in rbuf */
		memcpy(bm->rbuf, bm->rx_raw_buffer, bm->msg_len);

		/*Clean buffer manager*/
		bm->flag_msg_recv = false;
		bm->first_byte_rx = false;
		if (*byte_left < 0) {
			dev_err(&dev, "Too much data received %d", old_len);
			old_len += *byte_left;
			dev_err(&dev, "treated %d", old_len);
		}
		*byte_left = bm->buf_size;

		/*Put the tail to the beginning of the buffer*/
		bm->rx_raw_tail = bm->rx_raw_buffer;

		/* Notify rpmsg_recv_done */
		/* For the moment the function is directly called but
		 * it's possible to add it in a workqueue and add a
		 * mutex for the rbuf acces.
		 */
		uart_rpmsg_recv_done(srp);
	}

	return old_len;
err_bm:
	/* Reset of buffer manager */
	bm->flag_msg_recv = false;
	bm->first_byte_rx = false;
	*byte_left = bm->buf_size;
	bm->rx_raw_tail = bm->rx_raw_buffer;

	return ret;
}

/* Serdev receive callback called when somme raw data are received */
static int uart_rpmsg_receive(struct serdev_device *serdev,
			      const unsigned char *data, size_t count)
{
	struct serdev_info *srp = serdev_device_get_drvdata(serdev);

	return uart_rpmsg_append_data(srp, (uint8_t *)data, count);
}

static struct serdev_device_ops rpmsg_serdev_ops = {
	.receive_buf = uart_rpmsg_receive,
	.write_wakeup = serdev_device_write_wakeup,
};

static const struct of_device_id rpmsg_uart_of_match[] = {
	{
	 .compatible = "uart-rpmsg-bus",
	},
	{}
};
MODULE_DEVICE_TABLE(of, rpmsg_uart_of_match);

/* Invoked when a name service announcement arrives */
/*non factirizable car fait appel a rpmsg_create channel*/
static int uart_rpmsg_ns_cb(struct rpmsg_device *rpdev, void *data, int len,
			    void *priv, u32 src)
{
	struct serdev_info *srp = priv;
	struct device *dev = &srp->serdev->dev;

	return rpmsg_bus_ns_cb(dev, rpdev, data, len, priv, src,
			       uart_rpmsg_create_channel);
}

/* Buffer manager dedicated functions */

/**
 * buffer_manager_init - initialize buffer manager object
 * @bm:		the buffer manager instance
 * @buf_size:	the internal buffer size
 */
static inline int buffer_manager_init(struct buffer_manager *bm,
				      const int buf_size)
{
	if (buf_size < 0)
		return -EINVAL;

	/*Memory alloc rx_raw_left and rbuf*/
	bm->rx_raw_buffer = kzalloc(buf_size, GFP_KERNEL);
	if (!(bm->rx_raw_buffer))
		return -ENOMEM;

	bm->rbuf = kzalloc(buf_size, GFP_KERNEL);
	if (!(bm->rbuf)) {
		kfree(bm->rx_raw_buffer);
		return -ENOMEM;
	}

	bm->buf_size = buf_size;

	bm->rx_raw_tail = bm->rx_raw_buffer;
	bm->rx_raw_left = bm->buf_size;

	return 0;
}

/**
 * buffer_manager_deinit - clear buffer manager object
 * @bm:		the buffer manager instance
 */
static inline void buffer_manager_deinit(struct buffer_manager *bm)
{
	kfree(bm->rx_raw_buffer);
	kfree(bm->rbuf);
}

static int uart_rpmsg_probe(struct serdev_device *serdev)
{
	/*
	 * A faster speed can causes problems:
	 * some unexpected 0 can appear in the data.
	 * TODO: add a way to modify speed in userland
	 */
	u32 speed = 57600;
	u32 max_speed;
	struct serdev_info *srp;
	struct device_node *np = serdev->dev.of_node;
	struct buffer_manager bm;
	int err;

	err = of_property_read_u32(np, "max-speed", &max_speed);
	if (err) {
		dev_err(&serdev->dev, "Bad device description\n");
		return err;
	}
	if (speed > max_speed)
		speed = max_speed;

	srp = devm_kzalloc(&serdev->dev, sizeof(*srp), GFP_KERNEL);
	if (!srp)
		return -ENOMEM;

	srp->serdev = serdev;
	serdev_device_set_drvdata(serdev, srp);

	err = buffer_manager_init(&bm, MAX_RPMSG_BUF_SIZE);
	if (err)
		return err;

	srp->bm = bm;

	idr_init(&srp->endpoints);
	mutex_init(&srp->endpoints_lock);

	serdev_device_set_client_ops(serdev, &rpmsg_serdev_ops);
	err = serdev_device_open(serdev);
	if (err) {
		dev_err(&serdev->dev, "Unable to open device\n");
		goto err_serdev_open;
	}

	speed = serdev_device_set_baudrate(serdev, speed);
	dev_dbg(&serdev->dev, "Using baudrate: %u\n", speed);

	err = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (err)
		goto err_device_close;

	serdev_device_set_flow_control(serdev, false);


	/* A dedicated endpoint handles the name service msgs */
	mutex_lock(&srp->endpoints_lock);
	srp->ns_ept = rpmsg_bus_create_ept(&srp->endpoints,
					   &uart_endpoint_ops, NULL,
					   uart_rpmsg_ns_cb, srp,
					   RPMSG_NS_ADDR);
	mutex_unlock(&srp->endpoints_lock);
	if (!srp->ns_ept) {
		dev_err(&serdev->dev, "failed to create the ns ept\n");
		err = -ENOMEM;
		goto err_device_close;
	}

	dev_info(&serdev->dev, "rpmsg serdev host is online\n");

	/* Succes */
	return 0;

err_device_close:
	serdev_device_close(serdev);
err_serdev_open:
	buffer_manager_deinit(&bm);
	return err;
}

static int rpmsg_remove_device(struct device *dev, void *data)
{
	device_unregister(dev);

	return 0;
}

static void uart_rpmsg_remove(struct serdev_device *serdev)
{
	struct serdev_info *srp = serdev_device_get_drvdata(serdev);
	struct buffer_manager *bm = &srp->bm;
	int ret;

	serdev_device_close(serdev);

	ret = device_for_each_child(&serdev->dev, NULL, rpmsg_remove_device);
	if (ret)
		dev_warn(&serdev->dev, "can't remove rpmsg device: %d\n", ret);

	if (srp->ns_ept)
		rpmsg_bus_destroy_ept(&srp->endpoints, srp->ns_ept);

	/* Free memory of buffer manager */
	buffer_manager_deinit(bm);

	idr_destroy(&srp->endpoints);
}

static struct serdev_device_driver rpmsg_uart_driver = {
	.probe = uart_rpmsg_probe,
	.remove = uart_rpmsg_remove,
	.driver = {
		.name = "uart_rpmsg",
		.of_match_table = of_match_ptr(rpmsg_uart_of_match),
	},
};

static int __init rpmsg_uart_init(void)
{
	return serdev_device_driver_register(&rpmsg_uart_driver);
}

static void __exit rpmsg_uart_exit(void)
{
	serdev_device_driver_unregister(&rpmsg_uart_driver);
}

subsys_initcall(rpmsg_uart_init);
module_exit(rpmsg_uart_exit);

MODULE_AUTHOR("Maxime Mere <maxime.mere@st.com>");
MODULE_DESCRIPTION("serdev-based remote processor messaging bus");
MODULE_LICENSE("GPL v2");
