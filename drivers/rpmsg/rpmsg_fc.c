// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) STMicroelectronics 2022 - All Rights Reserved
 */
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rpmsg.h>
#include <linux/rpmsg/fc.h>
#include <linux/slab.h>

#include "rpmsg_internal.h"

/**
 * rpmsg_fc_register_device() - register name service device based on rpdev
 * @rpdev: prepared rpdev to be used for creating endpoints
 *
 * This function wraps rpmsg_register_device() preparing the rpdev for use as
 * basis for the rpmsg name service device.
 */
int rpmsg_fc_register_device(struct rpmsg_device *rpdev)
{
	strcpy(rpdev->id.name, "rpmsg_fc");
	rpdev->driver_override = "rpmsg_fc";
	rpdev->src = RPMSG_FC_ADDR;
	rpdev->dst = RPMSG_FC_ADDR;

	return rpmsg_register_device(rpdev);
}
EXPORT_SYMBOL(rpmsg_fc_register_device);

/* Invoked when a name service announcement arrives */
static int rpmsg_fc_cb(struct rpmsg_device *rpdev, void *data, int len,
		       void *priv, u32 src)
{
	struct rpmsg_ept_msg *msg = data;
	struct rpmsg_channel_info chinfo;
	struct device *dev = rpdev->dev.parent;
	bool enable;
	int ret;

#if defined(CONFIG_DYNAMIC_DEBUG)
	dynamic_hex_dump("FC announcement: ", DUMP_PREFIX_NONE, 16, 1,
			 data, len, true);
#endif

	if (len != sizeof(*msg)) {
		dev_err(dev, "malformed fc msg (%d)\n", len);
		return -EINVAL;
	}

	chinfo.src = rpmsg32_to_cpu(rpdev, msg->src);
	chinfo.dst = rpmsg32_to_cpu(rpdev, msg->dst);
	enable = rpmsg32_to_cpu(rpdev, msg->flags) & RPMSG_EPT_ON;

	dev_dbg(dev, "endpoint 0x%x  in state %sable\n", chinfo.dst, enable ? "en" : "dis");

	ret = rpmsg_channel_remote_fc(rpdev, &chinfo, enable);
	if (ret)
		dev_err(dev, "rpmsg_annouce_flow_ctrl failed: %d\n", ret);

	return ret;
}

static int rpmsg_fc_probe(struct rpmsg_device *rpdev)
{
	struct rpmsg_endpoint *fc_ept;
	struct rpmsg_channel_info fc_chinfo = {
		.src = RPMSG_FC_ADDR,
		.dst = RPMSG_FC_ADDR,
		.name = "flow_control_service",
	};

	/*
	 * Create the Flow control (FC) service endpoint associated to the RPMsg
	 * device. The endpoint will be automatically destroyed when the RPMsg
	 * device will be deleted.
	 */
	fc_ept = rpmsg_create_ept(rpdev, rpmsg_fc_cb, NULL, fc_chinfo);
	if (!fc_ept) {
		dev_err(&rpdev->dev, "failed to create the FC ept\n");
		return -ENOMEM;
	}
	rpdev->ept = fc_ept;

	return 0;
}

static struct rpmsg_driver rpmsg_fc_driver = {
	.drv.name = KBUILD_MODNAME,
	.probe = rpmsg_fc_probe,
};

static int rpmsg_fc_init(void)
{
	int ret;

	ret = register_rpmsg_driver(&rpmsg_fc_driver);
	if (ret < 0)
		pr_err("%s: Failed to register FC rpmsg driver\n", __func__);

	return ret;
}
postcore_initcall(rpmsg_fc_init);

static void rpmsg_fc_exit(void)
{
	unregister_rpmsg_driver(&rpmsg_fc_driver);
}
module_exit(rpmsg_fc_exit);

MODULE_DESCRIPTION("Name service announcement rpmsg driver");
MODULE_AUTHOR("Arnaud Pouliquen <arnaud.pouliquen@st.com>");
MODULE_ALIAS("rpmsg:" KBUILD_MODNAME);
MODULE_LICENSE("GPL");
