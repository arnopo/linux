// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2019 - All Rights Reserved
 * Author: Arnaud Pouliquen <arnaud.pouliquen@st.com> for STMicroelectronics.
 * Derived from the imx-rmpsg and omap-rpmsg implementations.
 */

#include <linux/input.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/rpmsg.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

struct vl53l0x_data {
	struct rpmsg_device *rpdev;
	struct iio_dev *iio_dev;
	struct completion completion;
	bool data_received;
	unsigned int val;

};

static int rpmsg_vl5310_cb(struct rpmsg_device *rpdev, void *buff, int len,
			   void *priv, u32 src)
{
	struct vl53l0x_data *data = dev_get_drvdata(&rpdev->dev);

	dev_dbg(&rpdev->dev, "%s:\n", __func__);
	data->val = *(unsigned int *)buff;

	complete(&data->completion);

	return 0;
}

static int vl53l0x_read_proximity(struct vl53l0x_data *data,
				  const struct iio_chan_spec *chan, int *val)
{
	struct rpmsg_device *rpdev = data->rpdev;
	int timeout, ret = 0;

	data->data_received = false;
	ret = rpmsg_send(rpdev->ept, "dist", sizeof("dist"));
	if (ret) {
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", ret);
		return ret;
	}
	timeout = wait_for_completion_timeout(&data->completion,
					      msecs_to_jiffies(2000));
	if (timeout == 0) {
		dev_err(&rpdev->dev, "timeout\n");
		return -ETIMEDOUT;
	}

	dev_dbg(&rpdev->dev, "%s: distance: %d\n", __func__, data->val);
	*val = data->val;
	reinit_completion(&data->completion);

	return 0;
}

static int vl53l0x_read_raw(struct iio_dev *indio_dev,
			    const struct iio_chan_spec *chan,
			    int *val, int *val2, long mask)
{
	struct vl53l0x_data *data = iio_priv(indio_dev);
	int ret;

	if (chan->type != IIO_DISTANCE)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = vl53l0x_read_proximity(data, chan, val);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = 1000;

		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static const struct iio_chan_spec vl53l0x_channels[] = {
	{
		.type = IIO_DISTANCE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
};

static const struct iio_info vl53l0x_info = {
	.read_raw = vl53l0x_read_raw,
};

static int rpmsg_vl5310_probe(struct rpmsg_device *rpdev)
{
	struct iio_dev *indio_dev;
	struct vl53l0x_data *data;

	indio_dev = devm_iio_device_alloc(&rpdev->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->rpdev = rpdev;
	data->iio_dev = indio_dev;

	indio_dev->dev.parent = &rpdev->dev;
	indio_dev->name = "vl53l0x";
	indio_dev->channels = vl53l0x_channels;
	indio_dev->num_channels = ARRAY_SIZE(vl53l0x_channels);
	indio_dev->info = &vl53l0x_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	init_completion(&data->completion);
	dev_set_drvdata(&rpdev->dev, data);

	return iio_device_register(indio_dev);
}

static void rpmsg_vl5310_remove(struct rpmsg_device *rpdev)
{
	struct vl53l0x_data *data = dev_get_drvdata(&rpdev->dev);

	iio_device_unregister(data->iio_dev);
	dev_info(&rpdev->dev, "rpmsg vl5310x device is removed\n");
}

static struct rpmsg_device_id rpmsg_vl5310_id_table[] = {
	{ .name = "rpmsg-vl5310x" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_vl5310_id_table);

static struct rpmsg_driver rpmsg_vl5310_rmpsg_drv = {
	.drv.name       = KBUILD_MODNAME,
	.drv.owner      = THIS_MODULE,
	.id_table       = rpmsg_vl5310_id_table,
	.probe	  = rpmsg_vl5310_probe,
	.callback       = rpmsg_vl5310_cb,
	.remove	 = rpmsg_vl5310_remove,
};

static int __init rpmsg_vl5310_init(void)
{
	return register_rpmsg_driver(&rpmsg_vl5310_rmpsg_drv);
}

static void __exit rpmsg_vl5310_exit(void)
{
	unregister_rpmsg_driver(&rpmsg_vl5310_rmpsg_drv);
}

module_init(rpmsg_vl5310_init);
module_exit(rpmsg_vl5310_exit);

MODULE_AUTHOR("Arnaud Pouliquen <arnaud.pouliquen@st.com>");
MODULE_DESCRIPTION("ST vl53l0x ToF ranging sensor driver over rpmsg");
MODULE_LICENSE("GPL");
