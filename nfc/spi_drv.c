/******************************************************************************
 * Copyright (C) 2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2013-2021 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ******************************************************************************/
/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>

#include "common.h"

/**
 * spi_disable_irq()
 *
 * Check if interrupt is disabled or not
 * and disable interrupt
 *
 * Return: int
 */
int spi_disable_irq(struct nfc_dev *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->spi_dev.irq_enabled_lock, flags);
	if (dev->spi_dev.irq_enabled) {
		disable_irq_nosync(dev->spi_dev.client->irq);
		dev->spi_dev.irq_enabled = false;
	}
	spin_unlock_irqrestore(&dev->spi_dev.irq_enabled_lock, flags);

	return 0;
}

/**
 * spi_enable_irq()
 *
 * Check if interrupt is enabled or not
 * and enable interrupt
 *
 * Return: int
 */
int spi_enable_irq(struct nfc_dev *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->spi_dev.irq_enabled_lock, flags);
	if (!dev->spi_dev.irq_enabled) {
		dev->spi_dev.irq_enabled = true;
		enable_irq(dev->spi_dev.client->irq);
	}
	spin_unlock_irqrestore(&dev->spi_dev.irq_enabled_lock, flags);
	return 0;
}

/**
 * add_spi_write_prefix() - Adds Write Prefix to the Buffer
 * @cmd: Takes uint8_t* buffer as an input
 *
 * Function to add prefix 0x7F to the write buffer
 *
 * Return: uint8_t* buffer
 */
uint8_t *add_spi_write_prefix(uint8_t *cmd)
{
	*cmd++ = WRITE_PREFIX_ON_WRITE;
	return cmd;
}

/**
 * add_write_prefix_for_spi_read() - Adds Write Prefix to the Buffer for spi read
 * @cmd: Takes uint8_t* buffer as an input
 *
 * Function to add prefix 0xFF to the write buffer
 *
 * Return: uint8_t* buffer
 */
uint8_t *add_write_prefix_for_spi_read(uint8_t *cmd)
{
	*cmd++ = WRITE_PREFIX_ON_READ;
	return cmd;
}

/**
 * get_spi_command_length() - Adds Write Prefix to the Buffer
 * @actual_length: Takes integer value as actual command length
 *
 * Function adds prefix to the command length
 *
 * Return: integer value by adding prefix length to actual command length
 */
int get_spi_command_length(int actual_length)
{
	return actual_length + PREFIX_LENGTH;
}

static irqreturn_t spi_irq_handler(int irq, void *dev_id)
{
	struct nfc_dev *nfc_dev = dev_id;
	struct spi_dev *spi_dev = &nfc_dev->spi_dev;

	if (device_may_wakeup(&spi_dev->client->dev))
		pm_wakeup_event(&spi_dev->client->dev, WAKEUP_SRC_TIMEOUT);

	spi_disable_irq(nfc_dev);
	wake_up(&nfc_dev->read_wq);

	return IRQ_HANDLED;
}

int nfc_spi_read(struct nfc_dev *nfc_dev, char *buf, size_t count, int timeout)
{
	int ret;
	int cmd_length = 0;
	struct spi_dev *spi_dev = &nfc_dev->spi_dev;
	struct platform_gpio *nfc_gpio = &nfc_dev->configs.gpio;

	pr_debug("%s: reading %zu bytes.\n", __func__, count);

	if (timeout > NCI_CMD_RSP_TIMEOUT_MS)
		timeout = NCI_CMD_RSP_TIMEOUT_MS;

	if (count > MAX_NCI_BUFFER_SIZE)
		count = MAX_NCI_BUFFER_SIZE;

	if (!gpio_get_value(nfc_gpio->irq)) {
		while (1) {
			ret = 0;
			if (!spi_dev->irq_enabled) {
				spi_dev->irq_enabled = true;
				enable_irq(spi_dev->client->irq);
			}
			if (!gpio_get_value(nfc_gpio->irq)) {
				if (timeout) {
					ret = wait_event_interruptible_timeout(
						nfc_dev->read_wq,
						!spi_dev->irq_enabled,
						msecs_to_jiffies(timeout));

					if (ret <= 0) {
						pr_err("%s: timeout error\n",
						       __func__);
						goto err;
					}
				} else {
					ret = wait_event_interruptible(
						nfc_dev->read_wq,
						!spi_dev->irq_enabled);
					if (ret) {
						pr_err("%s: err wakeup of wq\n",
						       __func__);
						goto err;
					}
				}
			}
			spi_disable_irq(nfc_dev);

			if (gpio_get_value(nfc_gpio->irq))
				break;
			if (!gpio_get_value(nfc_gpio->ven)) {
				pr_info("%s: releasing read\n", __func__);
				ret = -EIO;
				goto err;
			}
			pr_warn("%s: spurious interrupt detected\n", __func__);
		}
	}

	add_write_prefix_for_spi_read(spi_dev->tmp_write_kbuf);
	memset(buf, 0x00, count);
	/* Read data */
	ret = spi_write_then_read(
		spi_dev->client,
		spi_dev->tmp_write_kbuf,
		PREFIX_LENGTH, buf, count);
	if (ret < 0) {
		pr_err("%s: returned %d\n", __func__, ret);
		goto err;
	}
	return count;
err:
	return ret;
}

int nfc_spi_write(struct nfc_dev *nfc_dev, const char *buf, size_t count,
	       int max_retry_cnt)
{
	int ret = -EINVAL;
	int retry_cnt;
	struct platform_gpio *nfc_gpio = &nfc_dev->configs.gpio;

	if (count <= 0)
		return ret;

	if (count > MAX_DL_BUFFER_SIZE + PREFIX_LENGTH)
		count = MAX_DL_BUFFER_SIZE + PREFIX_LENGTH;

	pr_debug("%s: writing %zu bytes.\n", __func__, count);
	/*
	 * Wait for any pending read for max 15ms before write
	 * This is to avoid any packet corruption during read, when
	 * the host cmds resets NFCC during any parallel read operation
	 */
	for (retry_cnt = 1; retry_cnt <= MAX_WRITE_IRQ_COUNT; retry_cnt++) {
		if (gpio_get_value(nfc_gpio->irq)) {
			pr_warn("%s: irq high during write, wait\n", __func__);
			usleep_range(NFC_WRITE_IRQ_WAIT_TIME_US,
				     NFC_WRITE_IRQ_WAIT_TIME_US + 100);
		} else {
			break;
		}
		if (retry_cnt == MAX_WRITE_IRQ_COUNT &&
			     gpio_get_value(nfc_gpio->irq)) {
			pr_warn("%s: allow after maximum wait\n", __func__);
		}
	}
	for (retry_cnt = 1; retry_cnt <= max_retry_cnt; retry_cnt++) {
		struct spi_transfer transfer = {
			.tx_buf = buf,
			.rx_buf = nfc_dev->spi_dev.tmp_read_kbuf,
			.len = count,
		};
		struct spi_message message;

		spi_message_init(&message);
		spi_message_add_tail(&transfer, &message);
		ret = spi_sync(nfc_dev->spi_dev.client, &message);

		// Checking for the first read byte is 0xFF
		if ((nfc_dev->spi_dev.tmp_read_kbuf[0] ==
		     MISO_VAL_ON_WRITE_SUCCESS) &&
		    (ret == 0)) {
			count = count - PREFIX_LENGTH;
			break;
		}
		if ((nfc_dev->spi_dev.tmp_read_kbuf[0] !=
		     MISO_VAL_ON_WRITE_SUCCESS) &&
		    (retry_cnt >= max_retry_cnt)) {
			pr_debug("%s, Write failed returning -1 ", __func__);
			return 0;
		} else if (ret <= 0 && (nfc_dev->spi_dev.tmp_read_kbuf[0] !=
					MISO_VAL_ON_WRITE_SUCCESS)) {
			pr_warn("%s: write failed ret(%d), maybe in standby\n",
				__func__, ret);
			usleep_range(WRITE_RETRY_WAIT_TIME_US,
				     WRITE_RETRY_WAIT_TIME_US + 100);
		}
	}
	return count;
}

ssize_t nfc_spi_dev_read(struct file *filp, char __user *buf, size_t count,
			 loff_t *offset)
{
	int ret;
	struct nfc_dev *nfc_dev = (struct nfc_dev *)filp->private_data;

	if (filp->f_flags & O_NONBLOCK) {
		pr_err("%s: f_flags has nonblock. try again\n", __func__);
		return -EAGAIN;
	}
	mutex_lock(&nfc_dev->read_mutex);
	ret = nfc_spi_read(nfc_dev, nfc_dev->read_kbuf, count, 0);
	if (ret > 0) {
		if (copy_to_user(buf, nfc_dev->read_kbuf, ret)) {
			pr_warn("%s: failed to copy to user space\n", __func__);
			ret = -EFAULT;
		}
	}
	mutex_unlock(&nfc_dev->read_mutex);
	return ret;
}

ssize_t nfc_spi_dev_write(struct file *filp, const char __user *buf,
			  size_t count, loff_t *offset)
{
	int ret;
	struct nfc_dev *nfc_dev = (struct nfc_dev *)filp->private_data;

	if (count > MAX_DL_BUFFER_SIZE)
		count = MAX_DL_BUFFER_SIZE;

	mutex_lock(&nfc_dev->write_mutex);
	if (copy_from_user(add_spi_write_prefix(nfc_dev->write_kbuf), buf,
			   count)) {
		pr_err("%s: failed to copy from user space\n", __func__);
		mutex_unlock(&nfc_dev->write_mutex);
		return -EFAULT;
	}
	ret = nfc_spi_write(nfc_dev, nfc_dev->write_kbuf,
			 get_spi_command_length(count), NO_RETRY);
	mutex_unlock(&nfc_dev->write_mutex);
	return ret;
}

static const struct file_operations nfc_spi_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = nfc_spi_dev_read,
	.write = nfc_spi_dev_write,
	.open = nfc_dev_open,
	.release = nfc_dev_close,
	.unlocked_ioctl = nfc_dev_ioctl,
};

int nfc_spi_dev_probe(struct spi_device *client)
{
	int ret = 0;
	struct nfc_dev *nfc_dev = NULL;
	struct spi_dev *spi_dev = NULL;
	struct platform_configs nfc_configs;
	struct platform_gpio *nfc_gpio = &nfc_configs.gpio;
	unsigned int max_speed_hz;
	struct device_node *np = client->dev.of_node;

	pr_debug("%s: enter\n", __func__);
	/* retrieve details of gpios from dt */
	ret = nfc_parse_dt(&client->dev, &nfc_configs, PLATFORM_IF_SPI);
	if (ret) {
		pr_err("%s: failed to parse dt\n", __func__);
		goto err;
	}

	nfc_dev = kzalloc(sizeof(struct nfc_dev), GFP_KERNEL);
	if (nfc_dev == NULL) {
		ret = -ENOMEM;
		goto err;
	}
	nfc_dev->read_kbuf = kzalloc(MAX_NCI_BUFFER_SIZE, GFP_DMA | GFP_KERNEL);
	if (!nfc_dev->read_kbuf) {
		ret = -ENOMEM;
		goto err_free_nfc_dev;
	}
	nfc_dev->write_kbuf = kzalloc(MAX_DL_BUFFER_SIZE + PREFIX_LENGTH,
				      GFP_DMA | GFP_KERNEL);
	if (!nfc_dev->write_kbuf) {
		ret = -ENOMEM;
		goto err_free_read_kbuf;
	}
	nfc_dev->interface = PLATFORM_IF_SPI;
	nfc_dev->nfc_state = NFC_STATE_NCI;
	nfc_dev->spi_dev.client = client;
	spi_dev = &nfc_dev->spi_dev;
	nfc_dev->nfc_read = nfc_spi_read;
	nfc_dev->nfc_write = nfc_spi_write;
	nfc_dev->nfc_enable_intr = spi_enable_irq;
	nfc_dev->nfc_disable_intr = spi_disable_irq;
	client->bits_per_word = 8;
	client->mode = SPI_MODE_0;
	ret = of_property_read_u32(np, "spi-max-frequency", &max_speed_hz);
	if (ret < 0) {
		pr_err("%s: There's no spi-max-frequency property\n", __func__);
		goto err_free_write_kbuf;
	}
	client->max_speed_hz = max_speed_hz;
	pr_info("%s: device tree set SPI clock Frequency %u\n", __func__,
		client->max_speed_hz);

	spi_dev->tmp_write_kbuf = kzalloc(PREFIX_LENGTH, GFP_DMA | GFP_KERNEL);
	if (!spi_dev->tmp_write_kbuf) {
		ret = -ENOMEM;
		goto err_free_write_kbuf;
	}
	spi_dev->tmp_read_kbuf = kzalloc(MAX_DL_BUFFER_SIZE + PREFIX_LENGTH,
					 GFP_DMA | GFP_KERNEL);
	if (!spi_dev->tmp_read_kbuf) {
		ret = -ENOMEM;
		goto err_free_tmp_write_kbuf;
	}
	ret = spi_setup(client);
	if (ret < 0) {
		pr_err("%s: failed to do spi_setup()\n", __func__);
		goto err_free_tmp_read_kbuf;
	}
	ret = configure_gpio(nfc_gpio->ven, GPIO_OUTPUT);
	if (ret) {
		pr_err("%s: unable to request nfc reset gpio [%d]\n", __func__,
		       nfc_gpio->ven);
		goto err_free_tmp_read_kbuf;
	}
	ret = configure_gpio(nfc_gpio->irq, GPIO_IRQ);
	if (ret <= 0) {
		pr_err("%s: unable to request nfc irq gpio [%d]\n", __func__,
		       nfc_gpio->irq);
		goto err_free_gpio;
	}
	client->irq = ret;
	ret = configure_gpio(nfc_gpio->dwl_req, GPIO_OUTPUT);
	if (ret) {
		pr_err("%s: unable to request nfc firm downl gpio [%d]\n",
		       __func__, nfc_gpio->dwl_req);
	}

	/* copy the retrieved gpio details from DT */
	memcpy(&nfc_dev->configs, &nfc_configs,
	       sizeof(struct platform_configs));

	/* init mutex and queues */
	init_waitqueue_head(&nfc_dev->read_wq);
	mutex_init(&nfc_dev->read_mutex);
	mutex_init(&nfc_dev->write_mutex);
	mutex_init(&nfc_dev->dev_ref_mutex);
	spin_lock_init(&spi_dev->irq_enabled_lock);
	ret = nfc_misc_register(nfc_dev, &nfc_spi_dev_fops, DEV_COUNT,
				NFC_CHAR_DEV_NAME, CLASS_NAME);
	if (ret) {
		pr_err("%s: nfc_misc_register failed\n", __func__);
		goto err_mutex_destroy;
	}
	/* interrupt initializations */
	pr_info("%s: requesting IRQ %d\n", __func__, client->irq);
	spi_dev->irq_enabled = true;
	ret = request_irq(client->irq, spi_irq_handler, IRQF_TRIGGER_HIGH,
			  spi_dev->device.name, nfc_dev);
	if (ret) {
		pr_err("%s: request_irq failed\n", __func__);
		goto err_nfc_misc_unregister;
	}
	spi_disable_irq(nfc_dev);
	gpio_set_ven(nfc_dev, 1);
	gpio_set_ven(nfc_dev, 0);
	gpio_set_ven(nfc_dev, 1);
	device_init_wakeup(&client->dev, true);
	spi_set_drvdata(client, nfc_dev);
	spi_dev->irq_wake_up = false;

	pr_info("%s: probing nfc spi successfully\n", __func__);
	return 0;
err_nfc_misc_unregister:
	nfc_misc_unregister(nfc_dev, DEV_COUNT);
err_mutex_destroy:
	mutex_destroy(&nfc_dev->dev_ref_mutex);
	mutex_destroy(&nfc_dev->read_mutex);
	mutex_destroy(&nfc_dev->write_mutex);
err_free_gpio:
	gpio_free_all(nfc_dev);
err_free_tmp_read_kbuf:
	if(spi_dev)
		kfree(spi_dev->tmp_read_kbuf);
err_free_tmp_write_kbuf:
	if(spi_dev)
		kfree(spi_dev->tmp_write_kbuf);
err_free_write_kbuf:
	kfree(nfc_dev->write_kbuf);
err_free_read_kbuf:
	kfree(nfc_dev->read_kbuf);
err_free_nfc_dev:
	kfree(nfc_dev);
err:
	pr_err("%s: probing not successful, check hardware\n", __func__);
	return ret;
}

int nfc_spi_dev_remove(struct spi_device *client)
{
	int ret = 0;
	struct nfc_dev *nfc_dev = NULL;

	pr_info("%s: remove device\n", __func__);
	nfc_dev = dev_get_drvdata(&client->dev);
	if (!nfc_dev) {
		pr_err("%s: device doesn't exist anymore\n", __func__);
		ret = -ENODEV;
		return ret;
	}
	if (nfc_dev->dev_ref_count > 0) {
		pr_err("%s: device already in use\n", __func__);
		return -EBUSY;
	}
	device_init_wakeup(&client->dev, false);
	free_irq(client->irq, nfc_dev);
	nfc_misc_unregister(nfc_dev, DEV_COUNT);
	mutex_destroy(&nfc_dev->read_mutex);
	mutex_destroy(&nfc_dev->write_mutex);
	gpio_free_all(nfc_dev);
	kfree(nfc_dev->read_kbuf);
	kfree(nfc_dev->write_kbuf);
	kfree(nfc_dev);
	return ret;
}

int nfc_spi_dev_suspend(struct device *device)
{
	struct spi_device *client = to_spi_device(device);
	struct nfc_dev *nfc_dev = dev_get_drvdata(&client->dev);
	struct spi_dev *spi_dev = &nfc_dev->spi_dev;

	if (device_may_wakeup(&client->dev) && spi_dev->irq_enabled) {
		if (!enable_irq_wake(client->irq))
			spi_dev->irq_wake_up = true;
	}
	return 0;
}

int nfc_spi_dev_resume(struct device *device)
{
	struct spi_device *client = to_spi_device(device);
	struct nfc_dev *nfc_dev = dev_get_drvdata(&client->dev);
	struct spi_dev *spi_dev = &nfc_dev->spi_dev;

	if (device_may_wakeup(&client->dev) && spi_dev->irq_wake_up) {
		if (!disable_irq_wake(client->irq))
			spi_dev->irq_wake_up = false;
	}
	return 0;
}

static const struct of_device_id nfc_dev_match_table[] = {
	{
		.compatible = NFC_DRV_STR,
	},
	{}
};

static const struct dev_pm_ops nfc_spi_dev_pm_ops = { SET_SYSTEM_SLEEP_PM_OPS(
	nfc_spi_dev_suspend, nfc_spi_dev_resume) };

static struct spi_driver nfc_spi_dev_driver = {
	.probe = nfc_spi_dev_probe,
	.remove = nfc_spi_dev_remove,
	.driver = {
		.name = NFC_DEV_ID,
		.pm = &nfc_spi_dev_pm_ops,
		.of_match_table = nfc_dev_match_table,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};

MODULE_DEVICE_TABLE(of, nfc_dev_match_table);

static int __init nfc_spi_dev_init(void)
{
	int ret = 0;

	pr_info("%s: Loading NXP NFC SPI driver\n", __func__);
	ret = spi_register_driver(&nfc_spi_dev_driver);
	if (ret != 0)
		pr_err("%s: NFC SPI add driver error ret %d\n", __func__, ret);
	return ret;
}

module_init(nfc_spi_dev_init);

static void __exit nfc_spi_dev_exit(void)
{
	pr_info("%s: Unloading NXP NFC SPI driver\n", __func__);
	spi_unregister_driver(&nfc_spi_dev_driver);
}

module_exit(nfc_spi_dev_exit);
MODULE_ALIAS("spi:nxpnfc");
MODULE_DESCRIPTION("NXP NFC SPI driver");
MODULE_LICENSE("GPL");
