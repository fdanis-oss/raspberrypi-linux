/*
 *  Bluetooth HCI UART H4 driver for Broadcom devices
 *
 *  Copyright (C) 2015 Marcel Holtmann <marcel@holtmann.org>
 *  Copyright (C) 2017 Frederic Danis <frederic.danis.oss@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/serdev.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned/le_struct.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "hci_uart.h"
#include "btbcm.h"

struct bcm_bt_dev {
	struct hci_uart hu;
	struct serdev_device *serdev;

/*
	struct gpio_desc *reset;
	struct gpio_desc *wakeup_host;
	struct gpio_desc *wakeup_bt;
	unsigned long sysclk_speed;

	int wake_irq;
*/
	struct sk_buff *rx_skb;
	struct sk_buff_head txq;
	bdaddr_t bdaddr;

	int init_error;
//	struct completion init_completion;

//	u8 man_id;
//	u8 ver_id;

//	bool initialized;
//	bool tx_enabled;
//	bool rx_enabled;
};

static int bcm_serdev_setup(struct hci_uart *hu)
{
	return -1;
}

static int bcm_serdev_open(struct hci_uart *hu)
{
#if 1
	struct device *dev = &hu->serdev->dev;

	dev_dbg(dev, "protocol open");

	serdev_device_open(hu->serdev);

	return 0;
#else
	return -1;
#endif
}

static int bcm_serdev_flush(struct hci_uart *hu)
{
	struct bcm_bt_dev *btdev = hu->priv;

	dev_dbg(&btdev->serdev->dev, "flush device");

	skb_queue_purge(&btdev->txq);

	return 0;
}

static int bcm_serdev_close(struct hci_uart *hu)
{
	struct bcm_bt_dev *btdev = hu->priv;
	struct device *dev = &btdev->serdev->dev;

	dev_dbg(dev, "close device");

//	btdev->initialized = false;

	skb_queue_purge(&btdev->txq);

	kfree_skb(btdev->rx_skb);

	/* disable module */
//	gpiod_set_value(btdev->reset, 1);
//	gpiod_set_value(btdev->wakeup_bt, 0);

	serdev_device_close(btdev->serdev);

	return 0;
}

static const struct hci_uart_proto bcm_serdev_proto = {
	.id		= HCI_UART_BCM_SERDEV,
	.name		= "BCM",
	.open		= bcm_serdev_open,
	.close		= bcm_serdev_close,
//	.recv		= bcm_serdev_recv,
//	.enqueue	= bcm_serdev_enqueue,
//	.dequeue	= bcm_serdev_dequeue,
	.flush		= bcm_serdev_flush,
	.setup		= bcm_serdev_setup,
	.manufacturer	= 1,
};

static int bcm_bluetooth_serdev_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct bcm_bt_dev *btdev;
//	struct clk *sysclk;
	int err = 0;

	btdev = devm_kzalloc(dev, sizeof(*btdev), GFP_KERNEL);
	if (!btdev)
		return -ENOMEM;

	btdev->hu.serdev = btdev->serdev = serdev;
	serdev_device_set_drvdata(serdev, btdev);

/*
	btdev->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(btdev->reset)) {
		err = PTR_ERR(btdev->reset);
		dev_err(dev, "could not get reset gpio: %d", err);
		return err;
	}

	btdev->wakeup_host = devm_gpiod_get(dev, "host-wakeup", GPIOD_IN);
	if (IS_ERR(btdev->wakeup_host)) {
		err = PTR_ERR(btdev->wakeup_host);
		dev_err(dev, "could not get host wakeup gpio: %d", err);
		return err;
	}

	btdev->wake_irq = gpiod_to_irq(btdev->wakeup_host);

	err = devm_request_threaded_irq(dev, btdev->wake_irq, NULL,
		wakeup_handler,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"wakeup", btdev);
	if (err) {
		dev_err(dev, "could request wakeup irq: %d", err);
		return err;
	}

	btdev->wakeup_bt = devm_gpiod_get(dev, "bluetooth-wakeup",
					   GPIOD_OUT_LOW);
	if (IS_ERR(btdev->wakeup_bt)) {
		err = PTR_ERR(btdev->wakeup_bt);
		dev_err(dev, "could not get BT wakeup gpio: %d", err);
		return err;
	}

	sysclk = devm_clk_get(dev, "sysclk");
	if (IS_ERR(sysclk)) {
		err = PTR_ERR(sysclk);
		dev_err(dev, "could not get sysclk: %d", err);
		return err;
	}

	clk_prepare_enable(sysclk);
	btdev->sysclk_speed = clk_get_rate(sysclk);
	clk_disable_unprepare(sysclk);
*/
	skb_queue_head_init(&btdev->txq);

	btdev->hu.priv = btdev;
	btdev->hu.alignment = 2; /* Nokia H4+ is word aligned */

	err = hci_uart_register_device(&btdev->hu, &bcm_serdev_proto);
	if (err) {
		dev_err(dev, "could not register bluetooth uart: %d", err);
		return err;
	}

	dev_info(dev, "%s device registered.\n", "");

	return 0;
}

static void bcm_bluetooth_serdev_remove(struct serdev_device *serdev)
{
	struct bcm_bt_dev *btdev = serdev_device_get_drvdata(serdev);
	struct hci_uart *hu = &btdev->hu;
	struct hci_dev *hdev = hu->hdev;

	cancel_work_sync(&hu->write_work);

	hci_unregister_dev(hdev);
	hci_free_dev(hdev);
	hu->proto->close(hu);
}
#ifdef CONFIG_OF
static const struct of_device_id bcm_bluetooth_of_match[] = {
	{ .compatible = "brcm,bcm43438-bt", },
	{},
};
MODULE_DEVICE_TABLE(of, bcm_bluetooth_of_match);
#endif

static struct serdev_device_driver bcm_bluetooth_serdev_driver = {
	.probe = bcm_bluetooth_serdev_probe,
	.remove = bcm_bluetooth_serdev_remove,
	.driver = {
		.name = "bcm-bluetooth",
		.of_match_table = of_match_ptr(bcm_bluetooth_of_match),
	},
};

#if 1
module_serdev_device_driver(bcm_bluetooth_serdev_driver);

MODULE_LICENSE("GPL");
#else
int __init bcm_bluetooth_init(void)
{
	serdev_device_driver_register(&bcm_bluetooth_serdev_driver);

	return hci_uart_register_proto(&bcm_serdev_proto);
}

int __exit bcm_bluetooth_deinit(void)
{
	serdev_device_driver_unregister(&bcm_bluetooth_serdev_driver);

	return hci_uart_unregister_proto(&bcm_serdev_proto);
}
#endif
