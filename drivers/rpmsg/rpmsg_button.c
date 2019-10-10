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
#include <linux/io.h>

static struct input_dev *button_dev;

static int rpmsg_but_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	int value =  (int)((u8 *)data)[0];

	input_report_key(button_dev, BTN_0, value);
	input_sync(button_dev);

	return 0;
}

static int rpmsg_but_probe(struct rpmsg_device *rpdev)
{
	int error;

	button_dev = input_allocate_device();
	if (!button_dev) {
		dev_err(&rpdev->dev, "button.c: Not enough memory\n");
		return -ENOMEM;
	}

	button_dev->name = "rpmsg virtual button";
	button_dev->evbit[0] = BIT_MASK(EV_KEY);
	button_dev->keybit[BIT_WORD(BTN_0)] = BIT_MASK(BTN_0);

	error = input_register_device(button_dev);
	if (error) {
		dev_err(&rpdev->dev, "button.c: Failed to register device\n");
		goto err_free_dev;
	}

	error = rpmsg_send(rpdev->ept, "Xon", sizeof("Xon"));
	if (error) {
		dev_dbg(&rpdev->dev, "rpmsg_send failed: %d\n", error);
		return 0;
	}

	return 0;

err_free_dev:
	input_free_device(button_dev);

	return error;
}

static void rpmsg_but_remove(struct rpmsg_device *rpdev)
{
	input_unregister_device(button_dev);
	dev_info(&rpdev->dev, "rpmsg button device is removed\n");
}

static struct rpmsg_device_id rpmsg_but_id_table[] = {
	{ .name = "rpmsg-button" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_but_id_table);

static struct rpmsg_driver rpmsg_but_rmpsg_drv = {
	.drv.name       = KBUILD_MODNAME,
	.drv.owner      = THIS_MODULE,
	.id_table       = rpmsg_but_id_table,
	.probe	  = rpmsg_but_probe,
	.callback       = rpmsg_but_cb,
	.remove	 = rpmsg_but_remove,
};

static int __init button_init(void)
{
	return register_rpmsg_driver(&rpmsg_but_rmpsg_drv);
}

static void __exit button_exit(void)
{
	unregister_rpmsg_driver(&rpmsg_but_rmpsg_drv);
}

module_init(button_init);
module_exit(button_exit);

MODULE_AUTHOR("Arnaud Pouliquen <arnaud.pouliquen@st.com>");
MODULE_DESCRIPTION("button over rpmsg");
MODULE_LICENSE("GPL");
