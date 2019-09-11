// SPDX-License-Identifier: GPL-2.0
/*
 * Virtual Driver for RPMSG based I2C bus controller
 *
 * Copyright (C) STMicroelectronics 2019
 *
 */

#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

/*correspond to the rpmsg buf size minus the rpmsg header*/
#define PAYLOAD_SIZE	(496 * sizeof(uint8_t))
/**
 * Will be replaced by:
 * #define PAYLOAD_SIZE	(rpmsg_get_mtu(rpdev->ept))
 */

/*rpmsg_i2c_msg result flag*/
#define RPMSG_I2C_ACK	0x01
#define RPMSG_I2C_NACK	0x02

static LIST_HEAD(rpmsg_i2c_list);   /* i2c adapter instances list */
static DEFINE_MUTEX(rpmsg_i2c_lock);  /* protect adpater list */

/**
 * struct rpmsg_i2c_msg - client specific data
 * @addr: 8-bit slave addr, including r/w bit
 * @result: used by the slave as an acknowledge
 * @count: number of bytes to be transferred
 * @buf: data buffer
 */
struct rpmsg_i2c_msg {
	u8 addr;
	u32 count;
	u8 result;
	u8 buf[0];
};

/**
 * struct rpmsg_i2c_dev - device private data
 * @is_read: indicate that driver waiting a rd callback
 * @is_write: indicate that driver waiting a wr callback
 * @complete: completion for waiting data
 * @adap: i2c adapter handler
 * @list: rpmsg_i2c adapter list
 * @rpdrv: rpmsg driver handler
 * @dev_id: rpmsg driver device id
 * @rpdev: rpmsg device handler
 * @msg:  i2c client specific data
 */
struct rpmsg_i2c_dev {
	bool is_read;
	bool is_write;
	struct completion complete;
	struct i2c_adapter adap;

	struct list_head list; /* adapter device list */
	struct rpmsg_driver rpdrv;
	struct rpmsg_device_id dev_id;

	struct rpmsg_device *rpdev;
	struct rpmsg_i2c_msg *msg;
};

static int rpmsg_i2c_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	struct rpmsg_i2c_dev *prv_data = dev_get_drvdata(&rpdev->dev);
	struct rpmsg_i2c_msg *msg = prv_data->msg;

	msg->result = 0;

	dev_dbg(&rpdev->dev, "incoming msg (src: 0x%x)\n", src);

	if (prv_data->is_read || prv_data->is_write) {
		memcpy(msg, data, len);

		if (prv_data->is_read || msg->result)
			complete(&prv_data->complete);
		else
			dev_err(&rpdev->dev, "Expecting ack or nack\n");
	} else {
		dev_err(&rpdev->dev, "unexpected message\n");
	}

	return 0;
}

static int rpmsg_i2c_write(struct rpmsg_i2c_dev *prv_data,
			   struct i2c_msg *msg)
{
	int ret;
	unsigned long time_left;
	struct rpmsg_device *rpdev = prv_data->rpdev;
	struct rpmsg_i2c_msg *r_msg = prv_data->msg;

	int msg_size = sizeof(struct rpmsg_i2c_msg) + msg->len;

	if (!rpdev)
		return -EIO;

	if (msg_size > PAYLOAD_SIZE)
		return -EINVAL;

	r_msg->addr = i2c_8bit_addr_from_msg(msg);
	r_msg->count = msg->len;
	r_msg->result = 0;
	memcpy(r_msg->buf, msg->buf, msg->len);

	prv_data->is_write = true;
	init_completion(&prv_data->complete);

	ret = rpmsg_send(rpdev->ept, r_msg, msg_size);
	if (ret) {
		dev_err(&rpdev->dev, "write: rpmsg_send failed: %d\n", ret);
		goto err_wr;
	}

	time_left = wait_for_completion_timeout(&prv_data->complete,
						prv_data->adap.timeout);
	if (!time_left)
		ret = -ETIMEDOUT;

	if (r_msg->result & RPMSG_I2C_NACK)
		ret = -EIO;
err_wr:

	prv_data->is_write = false;

	return ret;
}

static int rpmsg_i2c_read(struct rpmsg_i2c_dev *prv_data,
			  struct i2c_msg *msg)
{
	int ret;
	unsigned long time_left;
	struct rpmsg_device *rpdev = prv_data->rpdev;
	struct rpmsg_i2c_msg *r_msg = prv_data->msg;

	int msg_size = sizeof(struct rpmsg_i2c_msg);

	if (!rpdev)
		return -EIO;

	r_msg->addr = i2c_8bit_addr_from_msg(msg);
	r_msg->count = msg->len;
	r_msg->result = 0;

	prv_data->is_read = true;
	init_completion(&prv_data->complete);

	ret = rpmsg_send(rpdev->ept, r_msg, msg_size);
	if (ret) {
		dev_err(&rpdev->dev, "read: rpmsg_send failed: %d\n", ret);
		goto err_rd;
	}

	time_left = wait_for_completion_timeout(&prv_data->complete,
						prv_data->adap.timeout);
	prv_data->is_read = false;
	if (time_left) {
		msg->len = r_msg->count;
		memcpy(msg->buf, r_msg->buf, r_msg->count);
	} else {
		ret = -ETIMEDOUT;
		goto err_rd;
	}

	if (r_msg->result & RPMSG_I2C_NACK)
		ret = -EIO;

	return ret;
err_rd:
	prv_data->is_read = false;
	return ret;
}

static int rpmsg_i2c_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[],
			  int num)
{
	struct rpmsg_i2c_dev *prv_data = i2c_get_adapdata(i2c_adap);
	int err = 0, count = 0, i;
	struct i2c_msg *pmsg;
	u32 status;

	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];
		status = pmsg->flags;

		if (pmsg->addr < 0x03 || pmsg->addr > 0x77)
			return -ENXIO;

		if ((status & I2C_M_RD) != false)
			err = rpmsg_i2c_read(prv_data, pmsg);
		else
			err = rpmsg_i2c_write(prv_data, pmsg);

		if (err < 0)
			break;
		count++;
	}
	return (err < 0) ? err : count;
}

static u32 rpmsg_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm rpmsg_i2c_algo = {
	.master_xfer = rpmsg_i2c_xfer,
	.functionality = rpmsg_i2c_func,
};

static int rpmsg_i2c_probe(struct rpmsg_device *rpdev)
{
	struct rpmsg_i2c_dev *tmp;

	mutex_lock(&rpmsg_i2c_lock);
	list_for_each_entry(tmp, &rpmsg_i2c_list, list) {
		if (!strcmp(tmp->dev_id.name, rpdev->id.name)) {
			/* Case when there is a match betwen proc id and service
			 * name
			 */
			if (!tmp->rpdev) {
				dev_info(&rpdev->dev,
					 "new channel: 0x%x -> 0x%x!\n",
					 rpdev->src, rpdev->dst);
				tmp->rpdev = (struct rpmsg_device *)rpdev;
				dev_set_drvdata(&rpdev->dev, tmp);
				mutex_unlock(&rpmsg_i2c_lock);

				dev_dbg(&rpdev->dev,
					"adapter linked with rpmsg device\n");
				return 0;
			}
			mutex_unlock(&rpmsg_i2c_lock);
			return -EBUSY;
		}
	}
	mutex_unlock(&rpmsg_i2c_lock);

	return -ENODEV;
}

static void rpmsg_i2c_remove(struct rpmsg_device *rpdev)
{
	struct rpmsg_i2c_dev *prv_data = dev_get_drvdata(&rpdev->dev);

	prv_data->rpdev = (struct rpmsg_device *)NULL;
}

static int rpmsg_i2c_platform_probe(struct platform_device *pdev)
{
	int ret;
	struct rpmsg_i2c_dev *prv_data;
	struct i2c_adapter *adap;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	const char *rproc_id_name;

	prv_data = devm_kzalloc(&pdev->dev, sizeof(*prv_data), GFP_KERNEL);
	if (!prv_data)
		return -ENOMEM;

	adap = &prv_data->adap;
	i2c_set_adapdata(adap, prv_data);

	snprintf(adap->name, sizeof(adap->name), "RPMSG I2C adapter");
	adap->owner = THIS_MODULE;
	adap->timeout = msecs_to_jiffies(50); /* 50 ms timeout */
	adap->retries = 0;
	adap->algo = &rpmsg_i2c_algo;
	adap->dev.parent = dev;

	prv_data->msg = devm_kzalloc(&pdev->dev, PAYLOAD_SIZE, GFP_KERNEL);
	if (!prv_data->msg)
		return -ENOMEM;

	prv_data->rpdev = NULL;
	prv_data->is_read = false;
	prv_data->is_write = false;

	platform_set_drvdata(pdev, prv_data);

	ret = i2c_add_adapter(adap);
	if (ret) {
		dev_err(dev, "failed to add I2C adapter: %d\n", ret);
		return ret;
	}

	/*Generation of a specific rpmsg driver for this particular instance*/
	ret = of_property_read_string(np, "proc-id", &rproc_id_name);
	if (ret) {
		dev_err(dev, "Error, proc-id property is missing: %d\n", ret);
		goto err_rpmsg;
	}
	strcpy(prv_data->dev_id.name, rproc_id_name);

	prv_data->rpdrv.drv.name = "rpmsg_i2c";
	prv_data->rpdrv.id_table = &prv_data->dev_id;
	prv_data->rpdrv.probe = rpmsg_i2c_probe;
	prv_data->rpdrv.callback = rpmsg_i2c_cb;
	prv_data->rpdrv.remove = rpmsg_i2c_remove;

	ret = register_rpmsg_driver(&prv_data->rpdrv);
	if (ret)
		goto err_rpmsg;

	/* Adding the adapter inside a list head to match it later with the
	 * corresponding rpmsg device
	 */
	mutex_lock(&rpmsg_i2c_lock);
	list_add_tail(&prv_data->list, &rpmsg_i2c_list);
	mutex_unlock(&rpmsg_i2c_lock);

	dev_info(dev, "I2C adapter: %s probbed\n", prv_data->adap.name);

	return 0;

err_rpmsg:

	i2c_del_adapter(&prv_data->adap);
	return ret;
}

static int rpmsg_i2c_platform_remove(struct platform_device *pdev)
{
	struct rpmsg_i2c_dev *prv_data = platform_get_drvdata(pdev);

	list_del(&prv_data->list);
	i2c_del_adapter(&prv_data->adap);

	return 0;
}

static const struct of_device_id rpmsg_i2c_dt_ids[] = {
	{ .compatible = "i2c-rpbus" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rpmsg_i2c_dt_ids);

static struct platform_driver rpmsg_i2c_platform_driver = {
	.driver = {
		.name	= "rpmsg_i2c",
		.of_match_table = rpmsg_i2c_dt_ids,
	},
	.probe		= rpmsg_i2c_platform_probe,
	.remove		= rpmsg_i2c_platform_remove
};

static int __init rpmsg_i2c_init(void)
{
	return platform_driver_register(&rpmsg_i2c_platform_driver);
}

static void __exit rpmsg_i2c_exit(void)
{
	platform_driver_unregister(&rpmsg_i2c_platform_driver);
}

module_init(rpmsg_i2c_init);
module_exit(rpmsg_i2c_exit);

MODULE_AUTHOR("Maxime Mere <maxime.mere@st.com>");
MODULE_DESCRIPTION("virtio remote processor messaging I2C driver");
MODULE_LICENSE("GPL v2");

