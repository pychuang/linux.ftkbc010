/*
 * Faraday FTKBC010 Keyboard Controller
 *
 * (C) Copyright 2009 Faraday Technology
 * Po-Yu Chuang <ratbert@faraday-tech.com>
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <asm/io.h>

#include "ftkbc010.h"

#define KB_ROWS		4
#define KB_COLS		4
#define SCANCODE(x, y)	((x) * KB_ROWS + (y))

static unsigned char ftkbc010_default_keycode[] = {
	KEY_BACK, KEY_MENU, KEY_HOME, 0,		/* 0 - 3 */
	0, KEY_UP, 0, 0,				/* 4 - 7 */
	KEY_RIGHT, KEY_REPLY, KEY_LEFT, KEY_VOLUMEUP,	/* 8 - 11 */
	KEY_END, KEY_DOWN, KEY_SEND, KEY_VOLUMEDOWN,	/* 12 - 15 */
};

struct ftkbc010 {
	unsigned char keycode[ARRAY_SIZE(ftkbc010_default_keycode)];
	struct input_dev *input;
	struct resource *res;
	void *base;
	int irq;
};

/******************************************************************************
 * internal functions
 *****************************************************************************/
static void ftkbc010_enable(struct ftkbc010 *ftkbc010)
{
	struct device *dev = &ftkbc010->input->dev;
	int reg;

	reg = FTKBC010_ASP_PERIOD(0x98fff);

	dev_info(dev, "[ASP] = %08x\n", reg);
	iowrite32(reg, ftkbc010->base + FTKBC010_OFFSET_ASP);

	reg = FTKBC010_CR_ENABLE
	    | FTKBC010_CR_EN_TXINT
	    | FTKBC010_CR_EN_RXINT
	    | FTKBC010_CR_CL_TXINT
	    | FTKBC010_CR_CL_RXINT
	    | FTKBC010_CR_EN_PAD
	    | FTKBC010_CR_AUTOSCAN
	    | FTKBC010_CR_CL_PADINT;

	dev_info(dev, "[CR]  = %08x\n", reg);
	iowrite32(reg, ftkbc010->base + FTKBC010_OFFSET_CR);
}

static void ftkbc010_disable(struct ftkbc010 *ftkbc010)
{
	iowrite32(0, ftkbc010->base + FTKBC010_OFFSET_CR);
}

/******************************************************************************
 * interrupt handler
 *****************************************************************************/
static irqreturn_t ftkbc010_interrupt(int irq, void *dev_id)
{
	struct ftkbc010 *ftkbc010 = dev_id;
	struct input_dev *input = ftkbc010->input;
	unsigned int status;
	unsigned int cr;

	status = ioread32(ftkbc010->base + FTKBC010_OFFSET_ISR);

	/*
	 * The clear interrupt bits are inside control register,
	 * so we need to read it out and then modify it -
	 * completely brain-dead design.
	 */
	cr = ioread32(ftkbc010->base + FTKBC010_OFFSET_CR);

	if (status & FTKBC010_ISR_RXINT) {
		dev_dbg(&input->dev, "RXINT\n");
		cr |= FTKBC010_CR_CL_RXINT;
	}

	if (status & FTKBC010_ISR_TXINT) {
		dev_dbg(&input->dev, "TXINT\n");
		cr |= FTKBC010_CR_CL_TXINT;
	}

	if (status & FTKBC010_ISR_PADINT) {
		unsigned int data;
		unsigned int x, y;
		unsigned int scancode;
		unsigned int keycode;

		data = ioread32(ftkbc010->base + FTKBC010_OFFSET_XC);
		x = ffs(0xff - FTKBC010_XC_L(data)) - 1;

		data = ioread32(ftkbc010->base + FTKBC010_OFFSET_YC);
		y = ffs(0xff - FTKBC010_YC_L(data)) - 1;

		scancode = SCANCODE(x, y);

		keycode = ftkbc010->keycode[scancode];

		input_report_key(input, keycode, 1);
		input_sync(input);

		/* We cannot tell key release - simulate one */
		input_report_key(input, keycode, 0);
		input_sync(input);

		dev_info(&input->dev, "(%x, %x) scancode %d, keycode %d\n",
			x, y, scancode, keycode);

		cr |= FTKBC010_CR_CL_PADINT;
	}

	iowrite32(cr, ftkbc010->base + FTKBC010_OFFSET_CR);

	return IRQ_HANDLED;
}

/******************************************************************************
 * struct platform_driver functions
 *****************************************************************************/
static int __devinit ftkbc010_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ftkbc010 *ftkbc010;
	struct input_dev *input;
	struct resource *res;
	int irq;
	int ret;
	int i;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "No mmio resource defined.\n");
		return -ENXIO;
	}

	if ((irq = platform_get_irq(pdev, 0)) < 0) {
		dev_err(dev, "No irq resource defined.\n");
		ret = irq;
		goto err_get_irq;
	}

	/*
	 * Allocate driver private data
	 */
	ftkbc010 = kzalloc(sizeof(struct ftkbc010), GFP_KERNEL);
	if (!ftkbc010) {
		dev_err(dev, "Failed to allocate memory.\n");
		ret = -ENOMEM;
		goto err_alloc_priv;
	}
	platform_set_drvdata(pdev, ftkbc010);

	/*
	 * Allocate input_dev
	 */
	input = input_allocate_device();
	if (!input) {
		dev_err(dev, "Failed to allocate input device.\n");
		ret = -EBUSY;
		goto err_alloc_input_dev;
	}
	ftkbc010->input = input;

	/*
	 * Map io memory
	 */
	ftkbc010->res = request_mem_region(res->start, res->end - res->start, dev_name(dev));
	if (!ftkbc010->res) {
		dev_err(dev, "Resources is unavailable.\n");
		ret = -EBUSY;
		goto err_req_mem_region;
	}

	ftkbc010->base = ioremap(res->start, res->end - res->start);
	if (!ftkbc010->base) {
		dev_err(dev, "Failed to map registers.\n");
		ret = -ENOMEM;
		goto err_ioremap;
	}

	ret = request_irq(irq, ftkbc010_interrupt, 0, pdev->name, ftkbc010);
	if (ret < 0) {
		dev_err(dev, "Failed to request irq %d\n", irq);
		goto err_req_irq;
	}
	ftkbc010->irq = irq;

	input->name = "Faraday keyboard controller";

	/*
	 * Setup default scancode to keycode mapping
	 */
	memcpy(ftkbc010->keycode, ftkbc010_default_keycode, sizeof(ftkbc010->keycode));

	input->keycode		= ftkbc010->keycode;
	input->keycodesize	= sizeof(unsigned char);
	input->keycodemax	= ARRAY_SIZE(ftkbc010_default_keycode);

	/*
	 * Declare what events and event codes can be generated
	 */
	input->evbit[0] = BIT_MASK(EV_KEY);

	for (i = 0; i < ARRAY_SIZE(ftkbc010_default_keycode); i++)
		set_bit(ftkbc010->keycode[i], input->keybit);

	clear_bit(0, input->keybit);

	/*
	 * Enable hardware
	 */
	ftkbc010_enable(ftkbc010);

	/*
	 * All went ok, so register to the input system
	 */
	ret = input_register_device(input);
	if (ret)
		goto err_register_input;

	dev_info(dev, "irq %d, mapped at %p\n", irq, ftkbc010->base);
	return 0;

err_register_input:
	free_irq(irq, ftkbc010);
err_req_irq:
	iounmap(ftkbc010->base);
err_ioremap:
//	release_mem_region(res->start, res->end - res->start);
err_req_mem_region:
	input_free_device(ftkbc010->input);
err_alloc_input_dev:
	platform_set_drvdata(pdev, ftkbc010);
	kfree(ftkbc010);
err_alloc_priv:
err_get_irq:
	release_resource(res);
	return ret;
}

static int __devexit ftkbc010_remove(struct platform_device *pdev)
{
	struct ftkbc010 *ftkbc010 = platform_get_drvdata(pdev);
	struct resource *res = ftkbc010->res;

	ftkbc010_disable(ftkbc010);

	input_unregister_device(ftkbc010->input);
	free_irq(ftkbc010->irq, ftkbc010);
	iounmap(ftkbc010->base);
//	release_mem_region(res->start, res->end - res->start);
	input_free_device(ftkbc010->input);
	platform_set_drvdata(pdev, ftkbc010);
	kfree(ftkbc010);
	release_resource(res);

	return 0;
}

static struct platform_driver ftkbc010_driver = {
	.probe		= ftkbc010_probe,
	.remove		= __devexit_p(ftkbc010_remove),
	.driver		= {
		.name	= "ftkbc010",
	},
};

/******************************************************************************
 * initialization / finalization
 *****************************************************************************/
static int __init ftkbc010_init(void)
{
	return platform_driver_register(&ftkbc010_driver);
}

static void __exit ftkbc010_exit(void)
{
	platform_driver_unregister(&ftkbc010_driver);
}

module_init(ftkbc010_init);
module_exit(ftkbc010_exit);

MODULE_DESCRIPTION("FTKBC010 keyboard driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Po-Yu Chuang <ratbert@faraday-tech.com>");
