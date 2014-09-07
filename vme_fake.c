/*
 * Fake VME bridge support.
 *
 * This drive provides a fake VME bridge chip, this enables debugging of the
 * VME framework in the absense of a VME system.
 *
 * This driver has to do a number of things in software that would be driven
 * by hardware if it was available, it will also result in extra overhead at
 * times when compared with driving actual hardware.
 *
 * Author: Martyn Welch <martyn@welches.me.uk>
 * Copyright (c) 2014 Martyn Welch
 *
 * Based on vme_tsi148.c:
 *
 * Author: Martyn Welch <martyn.welch@ge.com>
 * Copyright 2008 GE Intelligent Platforms Embedded Systems, Inc.
 *
 * Based on work by Tom Armistead and Ajit Prem
 * Copyright 2004 Motorola Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

/*
TODO:
 * DMA
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/byteorder/generic.h>
#include <linux/vme.h>

#include "vme_bridge.h"


/*
 *  Define the number of each that the fake driver supports.
 */
#define FAKE_MAX_MASTER		8	/* Max Master Windows */
#define FAKE_MAX_SLAVE		8	/* Max Slave Windows */
#define FAKE_MAX_DMA		2	/* Max DMA Controllers */
#define FAKE_MAX_MAILBOX	4	/* Max Mail Box registers */
#define FAKE_MAX_SEMAPHORE	8	/* Max Semaphores */


/* Structures to hold information normally held in device registers */
struct fake_slave_window {
	int enabled;
	unsigned long long vme_base;
	unsigned long long size;
	dma_addr_t buf_base;
	u32 aspace;
	u32 cycle;
};

struct fake_master_window {
	int enabled;
	unsigned long long vme_base;
	unsigned long long size;
	u32 aspace;
	u32 cycle;
	u32 dwidth;
};


/* Structure used to hold driver specific information */
struct fake_driver {
	struct vme_bridge *parent;
	struct fake_slave_window slaves[FAKE_MAX_SLAVE];
	struct fake_master_window masters[FAKE_MAX_MASTER];
#ifdef VME_DMA_EN
	wait_queue_head_t dma_queue[2];
#endif
	u32 lm_enabled;
	unsigned long long lm_base;
	u32 lm_aspace;
	u32 lm_cycle;
	void (*lm_callback[4])(int);	/* Called in interrupt handler */
	struct tasklet_struct int_tasklet;
	int int_level;
	int int_statid;
	void *crcsr_kernel;
	dma_addr_t crcsr_bus;
	struct mutex vme_int;		/*
					 * Only one VME interrupt can be
					 * generated at a time, provide locking
					 */
};


#ifdef VME_DMA_EN
/*
 * Layout of a DMAC Linked-List Descriptor
 *
 * Note: This structure is accessed via the chip and therefore must be
 *       correctly laid out - It must also be aligned on 64-bit boundaries.
 */
struct fake_dma_descriptor {
	__be32 dsau;      /* Source Address */
	__be32 dsal;
	__be32 ddau;      /* Destination Address */
	__be32 ddal;
	__be32 dsat;      /* Source attributes */
	__be32 ddat;      /* Destination attributes */
	__be32 dnlau;     /* Next link address */
	__be32 dnlal;
	__be32 dcnt;      /* Byte count */
	__be32 ddbs;      /* 2eSST Broadcast select */
};

struct fake_dma_entry {
	/*
	 * The descriptor needs to be aligned on a 64-bit boundary, we increase
	 * the chance of this by putting it first in the structure.
	 */
	struct fake_dma_descriptor descriptor;
	struct list_head list;
	dma_addr_t dma_handle;
};
#endif

/* Module parameter */
static int geoid;

static const char driver_name[] = "vme_fake";

static struct vme_bridge *exit_pointer;

static struct device *vme_root;

#ifdef VME_DMA_EN
/*
 * Wakes up DMA queue.
 */
static u32 fake_DMA_irqhandler(struct fake_driver *bridge,
	int channel_mask)
{
	u32 serviced = 0;

	if (channel_mask & FAKE_LCSR_INTS_DMA0S) {
		wake_up(&bridge->dma_queue[0]);
		serviced |= FAKE_LCSR_INTC_DMA0C;
	}
	if (channel_mask & FAKE_LCSR_INTS_DMA1S) {
		wake_up(&bridge->dma_queue[1]);
		serviced |= FAKE_LCSR_INTC_DMA1C;
	}

	return serviced;
}


/*
 * Top level interrupt handler.  Clears appropriate interrupt status bits and
 * then calls appropriate sub handler(s).
 */
static irqreturn_t fake_irqhandler(int irq, void *ptr)
{
	u32 stat, enable, serviced = 0;
	struct vme_bridge *fake_bridge;
	struct fake_driver *bridge;

	fake_bridge = ptr;

	bridge = fake_bridge->driver_priv;

	/* Determine which interrupts are unmasked and set */
	enable = ioread32be(bridge->base + FAKE_LCSR_INTEO);
	stat = ioread32be(bridge->base + FAKE_LCSR_INTS);

	/* Only look at unmasked interrupts */
	stat &= enable;

	if (unlikely(!stat))
		return IRQ_NONE;

	/* Call subhandlers as appropriate */
	/* DMA irqs */
	if (stat & (FAKE_LCSR_INTS_DMA1S | FAKE_LCSR_INTS_DMA0S))
		serviced |= fake_DMA_irqhandler(bridge, stat);

	/* Location monitor irqs */
	if (stat & (FAKE_LCSR_INTS_LM3S | FAKE_LCSR_INTS_LM2S |
			FAKE_LCSR_INTS_LM1S | FAKE_LCSR_INTS_LM0S))
		serviced |= fake_LM_irqhandler(bridge, stat);

	/* Mail box irqs */
	if (stat & (FAKE_LCSR_INTS_MB3S | FAKE_LCSR_INTS_MB2S |
			FAKE_LCSR_INTS_MB1S | FAKE_LCSR_INTS_MB0S))
		serviced |= fake_MB_irqhandler(fake_bridge, stat);

	/* PCI bus error */
	if (stat & FAKE_LCSR_INTS_PERRS)
		serviced |= fake_PERR_irqhandler(fake_bridge);

	/* VME bus error */
	if (stat & FAKE_LCSR_INTS_VERRS)
		serviced |= fake_VERR_irqhandler(fake_bridge);

	/* IACK irq */
	if (stat & FAKE_LCSR_INTS_IACKS)
		serviced |= fake_IACK_irqhandler(bridge);

	/* VME bus irqs */
	if (stat & (FAKE_LCSR_INTS_IRQ7S | FAKE_LCSR_INTS_IRQ6S |
			FAKE_LCSR_INTS_IRQ5S | FAKE_LCSR_INTS_IRQ4S |
			FAKE_LCSR_INTS_IRQ3S | FAKE_LCSR_INTS_IRQ2S |
			FAKE_LCSR_INTS_IRQ1S))
		serviced |= fake_VIRQ_irqhandler(fake_bridge, stat);

	/* Clear serviced interrupts */
	iowrite32be(serviced, bridge->base + FAKE_LCSR_INTC);

	return IRQ_HANDLED;
}
#endif

/*
 * Calling VME bus interrupt callback if provided.
 */
static void fake_VIRQ_tasklet(unsigned long data)
{
	struct vme_bridge *fake_bridge;
	struct fake_driver *bridge;

	fake_bridge = (struct vme_bridge *) data;
	bridge = fake_bridge->driver_priv;

	vme_irq_handler(fake_bridge, bridge->int_level, bridge->int_statid);
}

/*
 * Check to see if an IACk has been received, return true (1) or false (0).
 */
static int fake_iack_received(struct fake_driver *bridge)
{
	return 0;
}

/*
 * Configure VME interrupt
 */
static void fake_irq_set(struct vme_bridge *fake_bridge, int level,
	int state, int sync)
{
	return;
}

/*
 * Generate a VME bus interrupt at the requested level & vector. Wait for
 * interrupt to be acked.
 */
static int fake_irq_generate(struct vme_bridge *fake_bridge, int level,
	int statid)
{
	struct fake_driver *bridge;

	bridge = fake_bridge->driver_priv;

	mutex_lock(&bridge->vme_int);

	bridge->int_level = level;

	bridge->int_statid = statid;

	/* Schedule tasklet to run VME handler to emulate normal VME interrupt handler behaviour */
	tasklet_schedule(&bridge->int_tasklet);

	mutex_unlock(&bridge->vme_int);

	return 0;
}


/*
 * Initialize a slave window with the requested attributes.
 */
static int fake_slave_set(struct vme_slave_resource *image, int enabled,
	unsigned long long vme_base, unsigned long long size,
	dma_addr_t buf_base, u32 aspace, u32 cycle)
{
	unsigned int i, granularity = 0;
	unsigned long long vme_bound;
	struct vme_bridge *fake_bridge;
	struct fake_driver *bridge;

	fake_bridge = image->parent;
	bridge = fake_bridge->driver_priv;

	i = image->number;

	switch (aspace) {
	case VME_A16:
		granularity = 0x10;
		break;
	case VME_A24:
		granularity = 0x1000;
		break;
	case VME_A32:
		granularity = 0x10000;
		break;
	case VME_A64:
		granularity = 0x10000;
		break;
	case VME_CRCSR:
	case VME_USER1:
	case VME_USER2:
	case VME_USER3:
	case VME_USER4:
	default:
		pr_err("Invalid address space\n");
		return -EINVAL;
		break;
	}

	/*
	 * Bound address is a valid address for the window, adjust
	 * accordingly
	 */
	vme_bound = vme_base + size - granularity;

	if (vme_base & (granularity - 1)) {
		pr_err("Invalid VME base alignment\n");
		return -EINVAL;
	}
	if (vme_bound & (granularity - 1)) {
		pr_err("Invalid VME bound alignment\n");
		return -EINVAL;
	}

	mutex_lock(&image->mtx);

	bridge->slaves[i].enabled = enabled;
	bridge->slaves[i].vme_base = vme_base;
	bridge->slaves[i].size = size;
	bridge->slaves[i].buf_base = buf_base;
	bridge->slaves[i].aspace = aspace;
	bridge->slaves[i].cycle = cycle;

	mutex_unlock(&image->mtx);

	return 0;
}

/*
 * Get slave window configuration.
 */
static int fake_slave_get(struct vme_slave_resource *image, int *enabled,
	unsigned long long *vme_base, unsigned long long *size,
	dma_addr_t *buf_base, u32 *aspace, u32 *cycle)
{
	unsigned int i;
	struct fake_driver *bridge;

	bridge = image->parent->driver_priv;

	i = image->number;

	mutex_lock(&image->mtx);

	*enabled = bridge->slaves[i].enabled;
	*vme_base = bridge->slaves[i].vme_base;
	*size = bridge->slaves[i].size;
	*buf_base = bridge->slaves[i].buf_base;
	*aspace = bridge->slaves[i].aspace;
	*cycle = bridge->slaves[i].cycle;

	mutex_unlock(&image->mtx);

	return 0;
}


/*
 * Set the attributes of an outbound window.
 */
static int fake_master_set(struct vme_master_resource *image, int enabled,
	unsigned long long vme_base, unsigned long long size, u32 aspace,
	u32 cycle, u32 dwidth)
{
	int retval = 0;
	unsigned int i;
	struct vme_bridge *fake_bridge;
	struct fake_driver *bridge;

	fake_bridge = image->parent;

	bridge = fake_bridge->driver_priv;

	/* Verify input data */
	if (vme_base & 0xFFFF) {
		pr_err("Invalid VME Window alignment\n");
		retval = -EINVAL;
		goto err_window;
	}

	if (size & 0xFFFF) {
		spin_unlock(&image->lock);
		pr_err("Invalid size alignment\n");
		retval = -EINVAL;
		goto err_window;
	}

	if ((size == 0) && (enabled != 0)) {
		pr_err("Size must be non-zero for enabled windows\n");
		retval = -EINVAL;
		goto err_window;
	}

	/* Setup data width */
	switch (dwidth) {
	case VME_D8:
	case VME_D16:
	case VME_D32:
		break;
	default:
		spin_unlock(&image->lock);
		pr_err("Invalid data width\n");
		retval = -EINVAL;
		goto err_dwidth;
	}

	/* Setup address space */
	switch (aspace) {
	case VME_A16:
	case VME_A24:
	case VME_A32:
	case VME_A64:
	case VME_CRCSR:
	case VME_USER1:
	case VME_USER2:
	case VME_USER3:
	case VME_USER4:
		break;
	default:
		spin_unlock(&image->lock);
		pr_err("Invalid address space\n");
		retval = -EINVAL;
		goto err_aspace;
		break;
	}

	spin_lock(&image->lock);

	i = image->number;

	bridge->masters[i].enabled = enabled;
	bridge->masters[i].vme_base = vme_base;
	bridge->masters[i].size = size;
	bridge->masters[i].aspace = aspace;
	bridge->masters[i].cycle = cycle;
	bridge->masters[i].dwidth = dwidth;

	spin_unlock(&image->lock);

	return 0;

err_aspace:
err_dwidth:
err_window:
	return retval;

}

/*
 * Set the attributes of an outbound window.
 */
static int __fake_master_get(struct vme_master_resource *image, int *enabled,
	unsigned long long *vme_base, unsigned long long *size, u32 *aspace,
	u32 *cycle, u32 *dwidth)
{
	unsigned int i;
	struct fake_driver *bridge;

	bridge = image->parent->driver_priv;

	i = image->number;

	*enabled = bridge->masters[i].enabled;
	*vme_base = bridge->masters[i].vme_base;
	*size = bridge->masters[i].size;
	*aspace = bridge->masters[i].aspace;
	*cycle = bridge->masters[i].cycle;
	*dwidth = bridge->masters[i].dwidth;

	return 0;
}


static int fake_master_get(struct vme_master_resource *image, int *enabled,
	unsigned long long *vme_base, unsigned long long *size, u32 *aspace,
	u32 *cycle, u32 *dwidth)
{
	int retval;

	spin_lock(&image->lock);

	retval = __fake_master_get(image, enabled, vme_base, size, aspace,
		cycle, dwidth);

	spin_unlock(&image->lock);

	return retval;
}


void fake_lm_check(struct fake_driver *bridge, unsigned long long addr, u32 aspace, u32 cycle)
{
	struct vme_bridge *fake_bridge;
	unsigned long long lm_base;
	u32 lm_aspace, lm_cycle;
	int i;
	struct vme_lm_resource *lm;
	struct list_head *pos = NULL, *n;

	/* Get vme_bridge */
	fake_bridge = bridge->parent;

	/* Loop through each location monitor resource */
	list_for_each_safe(pos, n, &fake_bridge->lm_resources) {
		lm = list_entry(pos, struct vme_lm_resource, list);

		/* If disabled, we're done */
		if (bridge->lm_enabled == 0)
			return;

		lm_base = bridge->lm_base;
		lm_aspace = bridge->lm_aspace;
		lm_cycle = bridge->lm_cycle;

		/* First make sure that the cycle and address space match */
		if ((lm_aspace == aspace) && (lm_cycle == (cycle & 0xf000))) {
			for (i = 0; i < lm->monitors; i++) {
				/* Each location monitor covers 8 bytes */
				if (((lm_base + (8 * i)) <= addr) && ((lm_base + (8 * i) + 8) > addr)) {
					if (bridge->lm_callback[i] != NULL) {
						bridge->lm_callback[i](i);
					}
				}
			}
		}
	}
}


static u8 fake_vmeread8(struct fake_driver *bridge, unsigned long long addr, u32 aspace, u32 cycle)
{
	u8 retval = 0xff;
	int i;
	unsigned long long start, end, offset;
	u8 *loc;

	for (i = 0; i < FAKE_MAX_SLAVE; i++) {
		start = bridge->slaves[i].vme_base;
		end = bridge->slaves[i].vme_base + bridge->slaves[i].size;

		if (aspace != bridge->slaves[i].aspace)
			continue;

		if (cycle != bridge->slaves[i].cycle)
			continue;

		if ((addr >= start) && (addr < end)) {
			offset = addr - bridge->slaves[i].vme_base;
			loc = (u8 *)(bridge->slaves[i].buf_base + offset);
			retval = *loc;

			break;
		}
	}

	fake_lm_check(bridge, addr, aspace, cycle);

	return retval;
}


static u16 fake_vmeread16(struct fake_driver *bridge, unsigned long long addr, u32 aspace, u32 cycle)
{
	u16 retval = 0xffff;
	int i;
	unsigned long long start, end, offset;
	u16 *loc;

	for (i = 0; i < FAKE_MAX_SLAVE; i++) {
		if (aspace != bridge->slaves[i].aspace)
			continue;

		if (cycle != bridge->slaves[i].cycle)
			continue;

		start = bridge->slaves[i].vme_base;
		end = bridge->slaves[i].vme_base + bridge->slaves[i].size;

		if ((addr >= start) && ((addr + 1) < end)) {
			offset = addr - bridge->slaves[i].vme_base;
			loc = (u16 *)(bridge->slaves[i].buf_base + offset);
			retval = *loc;

			break;
		}
	}

	fake_lm_check(bridge, addr, aspace, cycle);

	return retval;
}


static u32 fake_vmeread32(struct fake_driver *bridge, unsigned long long addr, u32 aspace, u32 cycle)
{
	u32 retval = 0xffffffff;
	int i;
	unsigned long long start, end, offset;
	u32 *loc;

	for (i = 0; i < FAKE_MAX_SLAVE; i++) {
		if (aspace != bridge->slaves[i].aspace)
			continue;

		if (cycle != bridge->slaves[i].cycle)
			continue;

		start = bridge->slaves[i].vme_base;
		end = bridge->slaves[i].vme_base + bridge->slaves[i].size;

		if ((addr >= start) && ((addr + 3) < end)) {
			offset = addr - bridge->slaves[i].vme_base;
			loc = (u32 *)(bridge->slaves[i].buf_base + offset);
			retval = *loc;

			break;
		}
	}

	fake_lm_check(bridge, addr, aspace, cycle);

	return retval;
}

static ssize_t fake_master_read(struct vme_master_resource *image, void *buf,
	size_t count, loff_t offset)
{
	int retval;
	u32 aspace, cycle, dwidth;
	struct vme_bridge *fake_bridge;
	struct fake_driver *priv;
	int i;
	unsigned long long addr;
	unsigned int done = 0;
	unsigned int count32;

	fake_bridge = image->parent;

	priv = fake_bridge->driver_priv;

	i = image->number;

	addr = (unsigned long long)priv->masters[i].vme_base + offset;
	aspace = priv->masters[i].aspace;
	cycle = priv->masters[i].cycle;
	dwidth = priv->masters[i].dwidth;

	spin_lock(&image->lock);

	/* The following code handles VME address alignment. We cannot use
	 * memcpy_xxx here because it may cut data transfers in to 8-bit
	 * cycles when D16 or D32 cycles are required on the VME bus.
	 * On the other hand, the bridge itself assures that the maximum data
	 * cycle configured for the transfer is used and splits it
	 * automatically for non-aligned addresses, so we don't want the
	 * overhead of needlessly forcing small transfers for the entire cycle.
	 */
	if (addr & 0x1) {
		*(u8 *)buf = fake_vmeread8(priv, addr, aspace, cycle);
		done += 1;
		if (done == count)
			goto out;
	}
	if ((dwidth == VME_D16) || (dwidth == VME_D32)) {
		if ((addr + done) & 0x2) {
			if ((count - done) < 2) {
				*(u8 *)(buf + done) = fake_vmeread8(priv, addr + done, aspace, cycle);
				done += 1;
				goto out;
			} else {
				*(u16 *)(buf + done) = fake_vmeread16(priv, addr + done, aspace, cycle);
				done += 2;
			}
		}
	}

	if (dwidth == VME_D32) {
		count32 = (count - done) & ~0x3;
		while (done < count32) {
			*(u32 *)(buf + done) = fake_vmeread32(priv, addr + done, aspace, cycle);
			done += 4;
		}
	} else if (dwidth == VME_D16) {
		count32 = (count - done) & ~0x3;
		while (done < count32) {
			*(u16 *)(buf + done) = fake_vmeread16(priv, addr + done, aspace, cycle);
			done += 2;
		}
	} else if (dwidth == VME_D8) {
		count32 = (count - done);
		while (done < count32) {
			*(u8 *)(buf + done) = fake_vmeread8(priv, addr + done, aspace, cycle);
			done += 1;
		}

	}

	if ((dwidth == VME_D16) || (dwidth == VME_D32)) {
		if ((count - done) & 0x2) {
			*(u16 *)(buf + done) = fake_vmeread16(priv, addr + done, aspace, cycle);
			done += 2;
		}
	}
	if ((count - done) & 0x1) {
		*(u8 *)(buf + done) = fake_vmeread8(priv, addr + done, aspace, cycle);
		done += 1;
	}

out:
	retval = count;

	spin_unlock(&image->lock);

	return retval;
}


void fake_vmewrite8(struct fake_driver *bridge, u8 *buf, unsigned long long addr, u32 aspace, u32 cycle)
{
	int i;
	unsigned long long start, end, offset;
	u8 *loc;

	for (i = 0; i < FAKE_MAX_SLAVE; i++) {
		if (aspace != bridge->slaves[i].aspace)
			continue;

		if (cycle != bridge->slaves[i].cycle)
			continue;

		start = bridge->slaves[i].vme_base;
		end = bridge->slaves[i].vme_base + bridge->slaves[i].size;

		if ((addr >= start) && (addr < end)) {
			offset = addr - bridge->slaves[i].vme_base;
			loc = (u8 *)(bridge->slaves[i].buf_base + offset);
			*loc = *buf;

			break;
		}
	}

	fake_lm_check(bridge, addr, aspace, cycle);

}


void fake_vmewrite16(struct fake_driver *bridge, u16 *buf, unsigned long long addr, u32 aspace, u32 cycle)
{
	int i;
	unsigned long long start, end, offset;
	u16 *loc;

	for (i = 0; i < FAKE_MAX_SLAVE; i++) {
		if (aspace != bridge->slaves[i].aspace)
			continue;

		if (cycle != bridge->slaves[i].cycle)
			continue;

		start = bridge->slaves[i].vme_base;
		end = bridge->slaves[i].vme_base + bridge->slaves[i].size;

		if ((addr >= start) && ((addr + 1) < end)) {
			offset = addr - bridge->slaves[i].vme_base;
			loc = (u16 *)(bridge->slaves[i].buf_base + offset);
			*loc = *buf;

			break;
		}
	}

	fake_lm_check(bridge, addr, aspace, cycle);

}


void fake_vmewrite32(struct fake_driver *bridge, u32 *buf, unsigned long long addr, u32 aspace, u32 cycle)
{
	int i;
	unsigned long long start, end, offset;
	u32 *loc;

	for (i = 0; i < FAKE_MAX_SLAVE; i++) {
		if (aspace != bridge->slaves[i].aspace)
			continue;

		if (cycle != bridge->slaves[i].cycle)
			continue;

		start = bridge->slaves[i].vme_base;
		end = bridge->slaves[i].vme_base + bridge->slaves[i].size;

		if ((addr >= start) && ((addr + 3) < end) ) {
			offset = addr - bridge->slaves[i].vme_base;
			loc = (u32 *)(bridge->slaves[i].buf_base + offset);
			*loc = *buf;

			break;
		}
	}

	fake_lm_check(bridge, addr, aspace, cycle);

}


static ssize_t fake_master_write(struct vme_master_resource *image, void *buf,
	size_t count, loff_t offset)
{
	int retval = 0;
	u32 aspace, cycle, dwidth;
	unsigned long long addr;
	int i;
	unsigned int done = 0;
	unsigned int count32;

	struct vme_bridge *fake_bridge;
	struct fake_driver *bridge;

	fake_bridge = image->parent;

	bridge = fake_bridge->driver_priv;

	i = image->number;

	addr = bridge->masters[i].vme_base + offset;
	aspace = bridge->masters[i].aspace;
	cycle = bridge->masters[i].cycle;
	dwidth = bridge->masters[i].dwidth;

	spin_lock(&image->lock);

	/* Here we apply for the same strategy we do in master_read
	 * function in order to assure the correct cycles.
	 */
	if (addr & 0x1) {
		fake_vmewrite8(bridge, (u8 *)buf, addr, aspace, cycle);
		done += 1;
		if (done == count)
			goto out;
	}

	if ((dwidth == VME_D16) || (dwidth == VME_D32)) {
		if ((addr + done) & 0x2) {
			if ((count - done) < 2) {
				fake_vmewrite8(bridge, (u8 *)(buf + done), addr + done, aspace, cycle);
				done += 1;
				goto out;
			} else {
				fake_vmewrite16(bridge, (u16 *)(buf + done), addr + done, aspace, cycle);
				done += 2;
			}
		}
	}

	if (dwidth == VME_D32) {
		count32 = (count - done) & ~0x3;
		while (done < count32) {
			fake_vmewrite32(bridge, (u32 *)(buf + done), addr + done, aspace, cycle);
			done += 4;
		}
	} else if (dwidth == VME_D16) {
		count32 = (count - done) & ~0x3;
		while (done < count32) {
			fake_vmewrite16(bridge, (u16 *)(buf + done), addr + done, aspace, cycle);
			done += 2;
		}
	} else if (dwidth == VME_D8) {
		count32 = (count - done);
		while (done < count32) {
			fake_vmewrite8(bridge, (u8 *)(buf + done), addr + done, aspace, cycle);
			done += 1;
		}

	}

	if ((dwidth == VME_D16) || (dwidth == VME_D32)) {
		if ((count - done) & 0x2) {
			fake_vmewrite16(bridge, (u16 *)(buf + done), addr + done, aspace, cycle);
			done += 2;
		}
	}

	if ((count - done) & 0x1) {
		fake_vmewrite8(bridge, (u8 *)(buf + done), addr + done, aspace, cycle);
		done += 1;
	}

out:
	retval = count;

	spin_unlock(&image->lock);

	return retval;
}

/*
 * Perform an RMW cycle on the VME bus.
 *
 * Requires a previously configured master window, returns final value.
 */
static unsigned int fake_master_rmw(struct vme_master_resource *image,
	unsigned int mask, unsigned int compare, unsigned int swap,
	loff_t offset)
{
	u32 tmp, base;
	u32 aspace, cycle;
	int i;
	struct fake_driver *bridge;

	bridge = image->parent->driver_priv;

	/* Find the PCI address that maps to the desired VME address */
	i = image->number;

	base = bridge->masters[i].vme_base;
	aspace = bridge->masters[i].aspace;
	cycle = bridge->masters[i].cycle;

	/* Lock image */
	spin_lock(&image->lock);

	/* Read existing value */
	tmp = fake_vmeread32(bridge, base + offset, aspace, cycle);

	/* Perform check */
	if ((tmp && mask) == (compare && mask)) {
                tmp = tmp | (mask | swap);
                tmp = tmp & (~mask | swap);

		/* Write back */
		fake_vmewrite32(bridge, &tmp, base + offset, aspace, cycle);
	}

	/* Unlock image */
	spin_unlock(&image->lock);

	return tmp;
}


#ifdef VME_DMA_EN
static int fake_dma_set_vme_src_attributes(struct device *dev, __be32 *attr,
	u32 aspace, u32 cycle, u32 dwidth)
{
	u32 val;

	val = be32_to_cpu(*attr);

	/* Setup 2eSST speeds */
	switch (cycle & (VME_2eSST160 | VME_2eSST267 | VME_2eSST320)) {
	case VME_2eSST160:
		val |= FAKE_LCSR_DSAT_2eSSTM_160;
		break;
	case VME_2eSST267:
		val |= FAKE_LCSR_DSAT_2eSSTM_267;
		break;
	case VME_2eSST320:
		val |= FAKE_LCSR_DSAT_2eSSTM_320;
		break;
	}

	/* Setup cycle types */
	if (cycle & VME_SCT)
		val |= FAKE_LCSR_DSAT_TM_SCT;

	if (cycle & VME_BLT)
		val |= FAKE_LCSR_DSAT_TM_BLT;

	if (cycle & VME_MBLT)
		val |= FAKE_LCSR_DSAT_TM_MBLT;

	if (cycle & VME_2eVME)
		val |= FAKE_LCSR_DSAT_TM_2eVME;

	if (cycle & VME_2eSST)
		val |= FAKE_LCSR_DSAT_TM_2eSST;

	if (cycle & VME_2eSSTB) {
		pr_err("Currently not setting Broadcast Select Registers\n");
		val |= FAKE_LCSR_DSAT_TM_2eSSTB;
	}

	/* Setup data width */
	switch (dwidth) {
	case VME_D16:
		val |= FAKE_LCSR_DSAT_DBW_16;
		break;
	case VME_D32:
		val |= FAKE_LCSR_DSAT_DBW_32;
		break;
	default:
		pr_err("Invalid data width\n");
		return -EINVAL;
	}

	/* Setup address space */
	switch (aspace) {
	case VME_A16:
		val |= FAKE_LCSR_DSAT_AMODE_A16;
		break;
	case VME_A24:
		val |= FAKE_LCSR_DSAT_AMODE_A24;
		break;
	case VME_A32:
		val |= FAKE_LCSR_DSAT_AMODE_A32;
		break;
	case VME_A64:
		val |= FAKE_LCSR_DSAT_AMODE_A64;
		break;
	case VME_CRCSR:
		val |= FAKE_LCSR_DSAT_AMODE_CRCSR;
		break;
	case VME_USER1:
		val |= FAKE_LCSR_DSAT_AMODE_USER1;
		break;
	case VME_USER2:
		val |= FAKE_LCSR_DSAT_AMODE_USER2;
		break;
	case VME_USER3:
		val |= FAKE_LCSR_DSAT_AMODE_USER3;
		break;
	case VME_USER4:
		val |= FAKE_LCSR_DSAT_AMODE_USER4;
		break;
	default:
		pr_err("Invalid address space\n");
		return -EINVAL;
		break;
	}

	if (cycle & VME_SUPER)
		val |= FAKE_LCSR_DSAT_SUP;
	if (cycle & VME_PROG)
		val |= FAKE_LCSR_DSAT_PGM;

	*attr = cpu_to_be32(val);

	return 0;
}

static int fake_dma_set_vme_dest_attributes(struct device *dev, __be32 *attr,
	u32 aspace, u32 cycle, u32 dwidth)
{
	u32 val;

	val = be32_to_cpu(*attr);

	/* Setup 2eSST speeds */
	switch (cycle & (VME_2eSST160 | VME_2eSST267 | VME_2eSST320)) {
	case VME_2eSST160:
		val |= FAKE_LCSR_DDAT_2eSSTM_160;
		break;
	case VME_2eSST267:
		val |= FAKE_LCSR_DDAT_2eSSTM_267;
		break;
	case VME_2eSST320:
		val |= FAKE_LCSR_DDAT_2eSSTM_320;
		break;
	}

	/* Setup cycle types */
	if (cycle & VME_SCT)
		val |= FAKE_LCSR_DDAT_TM_SCT;

	if (cycle & VME_BLT)
		val |= FAKE_LCSR_DDAT_TM_BLT;

	if (cycle & VME_MBLT)
		val |= FAKE_LCSR_DDAT_TM_MBLT;

	if (cycle & VME_2eVME)
		val |= FAKE_LCSR_DDAT_TM_2eVME;

	if (cycle & VME_2eSST)
		val |= FAKE_LCSR_DDAT_TM_2eSST;

	if (cycle & VME_2eSSTB) {
		pr_err("Currently not setting Broadcast Select Registers\n");
		val |= FAKE_LCSR_DDAT_TM_2eSSTB;
	}

	/* Setup data width */
	switch (dwidth) {
	case VME_D16:
		val |= FAKE_LCSR_DDAT_DBW_16;
		break;
	case VME_D32:
		val |= FAKE_LCSR_DDAT_DBW_32;
		break;
	default:
		pr_err("Invalid data width\n");
		return -EINVAL;
	}

	/* Setup address space */
	switch (aspace) {
	case VME_A16:
		val |= FAKE_LCSR_DDAT_AMODE_A16;
		break;
	case VME_A24:
		val |= FAKE_LCSR_DDAT_AMODE_A24;
		break;
	case VME_A32:
		val |= FAKE_LCSR_DDAT_AMODE_A32;
		break;
	case VME_A64:
		val |= FAKE_LCSR_DDAT_AMODE_A64;
		break;
	case VME_CRCSR:
		val |= FAKE_LCSR_DDAT_AMODE_CRCSR;
		break;
	case VME_USER1:
		val |= FAKE_LCSR_DDAT_AMODE_USER1;
		break;
	case VME_USER2:
		val |= FAKE_LCSR_DDAT_AMODE_USER2;
		break;
	case VME_USER3:
		val |= FAKE_LCSR_DDAT_AMODE_USER3;
		break;
	case VME_USER4:
		val |= FAKE_LCSR_DDAT_AMODE_USER4;
		break;
	default:
		pr_err("Invalid address space\n");
		return -EINVAL;
		break;
	}

	if (cycle & VME_SUPER)
		val |= FAKE_LCSR_DDAT_SUP;
	if (cycle & VME_PROG)
		val |= FAKE_LCSR_DDAT_PGM;

	*attr = cpu_to_be32(val);

	return 0;
}

/*
 * Add a link list descriptor to the list
 *
 * Note: DMA engine expects the DMA descriptor to be big endian.
 */
static int fake_dma_list_add(struct vme_dma_list *list,
	struct vme_dma_attr *src, struct vme_dma_attr *dest, size_t count)
{
	struct fake_dma_entry *entry, *prev;
	u32 address_high, address_low, val;
	struct vme_dma_pattern *pattern_attr;
	struct vme_dma_pci *pci_attr;
	struct vme_dma_vme *vme_attr;
	int retval = 0;
	struct vme_bridge *fake_bridge;

	fake_bridge = list->parent->parent;

	/* Descriptor must be aligned on 64-bit boundaries */
	entry = kmalloc(sizeof(struct fake_dma_entry), GFP_KERNEL);
	if (entry == NULL) {
		pr_err("Failed to allocate memory for dma resource structure\n");
		retval = -ENOMEM;
		goto err_mem;
	}

	/* Test descriptor alignment */
	if ((unsigned long)&entry->descriptor & 0x7) {
		pr_err("Descriptor not aligned to 8 byte boundary as required: %p\n",
			&entry->descriptor);
		retval = -EINVAL;
		goto err_align;
	}

	/* Given we are going to fill out the structure, we probably don't
	 * need to zero it, but better safe than sorry for now.
	 */
	memset(&entry->descriptor, 0, sizeof(struct fake_dma_descriptor));

	/* Fill out source part */
	switch (src->type) {
	case VME_DMA_PATTERN:
		pattern_attr = src->private;

		entry->descriptor.dsal = cpu_to_be32(pattern_attr->pattern);

		val = FAKE_LCSR_DSAT_TYP_PAT;

		/* Default behaviour is 32 bit pattern */
		if (pattern_attr->type & VME_DMA_PATTERN_BYTE)
			val |= FAKE_LCSR_DSAT_PSZ;

		/* It seems that the default behaviour is to increment */
		if ((pattern_attr->type & VME_DMA_PATTERN_INCREMENT) == 0)
			val |= FAKE_LCSR_DSAT_NIN;
		entry->descriptor.dsat = cpu_to_be32(val);
		break;
	case VME_DMA_PCI:
		pci_attr = src->private;

		reg_split((unsigned long long)pci_attr->address, &address_high,
			&address_low);
		entry->descriptor.dsau = cpu_to_be32(address_high);
		entry->descriptor.dsal = cpu_to_be32(address_low);
		entry->descriptor.dsat = cpu_to_be32(FAKE_LCSR_DSAT_TYP_PCI);
		break;
	case VME_DMA_VME:
		vme_attr = src->private;

		reg_split((unsigned long long)vme_attr->address, &address_high,
			&address_low);
		entry->descriptor.dsau = cpu_to_be32(address_high);
		entry->descriptor.dsal = cpu_to_be32(address_low);
		entry->descriptor.dsat = cpu_to_be32(FAKE_LCSR_DSAT_TYP_VME);

		retval = fake_dma_set_vme_src_attributes(
			fake_bridge->parent, &entry->descriptor.dsat,
			vme_attr->aspace, vme_attr->cycle, vme_attr->dwidth);
		if (retval < 0)
			goto err_source;
		break;
	default:
		pr_err("Invalid source type\n");
		retval = -EINVAL;
		goto err_source;
		break;
	}

	/* Assume last link - this will be over-written by adding another */
	entry->descriptor.dnlau = cpu_to_be32(0);
	entry->descriptor.dnlal = cpu_to_be32(FAKE_LCSR_DNLAL_LLA);

	/* Fill out destination part */
	switch (dest->type) {
	case VME_DMA_PCI:
		pci_attr = dest->private;

		reg_split((unsigned long long)pci_attr->address, &address_high,
			&address_low);
		entry->descriptor.ddau = cpu_to_be32(address_high);
		entry->descriptor.ddal = cpu_to_be32(address_low);
		entry->descriptor.ddat = cpu_to_be32(FAKE_LCSR_DDAT_TYP_PCI);
		break;
	case VME_DMA_VME:
		vme_attr = dest->private;

		reg_split((unsigned long long)vme_attr->address, &address_high,
			&address_low);
		entry->descriptor.ddau = cpu_to_be32(address_high);
		entry->descriptor.ddal = cpu_to_be32(address_low);
		entry->descriptor.ddat = cpu_to_be32(FAKE_LCSR_DDAT_TYP_VME);

		retval = fake_dma_set_vme_dest_attributes(
			fake_bridge->parent, &entry->descriptor.ddat,
			vme_attr->aspace, vme_attr->cycle, vme_attr->dwidth);
		if (retval < 0)
			goto err_dest;
		break;
	default:
		pr_err("Invalid destination type\n");
		retval = -EINVAL;
		goto err_dest;
		break;
	}

	/* Fill out count */
	entry->descriptor.dcnt = cpu_to_be32((u32)count);

	/* Add to list */
	list_add_tail(&entry->list, &list->entries);

	/* Fill out previous descriptors "Next Address" */
	if (entry->list.prev != &list->entries) {
		prev = list_entry(entry->list.prev, struct fake_dma_entry,
			list);
		/* We need the bus address for the pointer */
		entry->dma_handle = dma_map_single(fake_bridge->parent,
			&entry->descriptor,
			sizeof(struct fake_dma_descriptor), DMA_TO_DEVICE);

		reg_split((unsigned long long)entry->dma_handle, &address_high,
			&address_low);
		entry->descriptor.dnlau = cpu_to_be32(address_high);
		entry->descriptor.dnlal = cpu_to_be32(address_low);

	}

	return 0;

err_dest:
err_source:
err_align:
		kfree(entry);
err_mem:
	return retval;
}

/*
 * Check to see if the provided DMA channel is busy.
 */
static int fake_dma_busy(struct vme_bridge *fake_bridge, int channel)
{
	u32 tmp;
	struct fake_driver *bridge;

	bridge = fake_bridge->driver_priv;

	tmp = ioread32be(bridge->base + FAKE_LCSR_DMA[channel] +
		FAKE_LCSR_OFFSET_DSTA);

	if (tmp & FAKE_LCSR_DSTA_BSY)
		return 0;
	else
		return 1;

}

/*
 * Execute a previously generated link list
 *
 * XXX Need to provide control register configuration.
 */
static int fake_dma_list_exec(struct vme_dma_list *list)
{
	struct vme_dma_resource *ctrlr;
	int channel, retval = 0;
	struct fake_dma_entry *entry;
	u32 bus_addr_high, bus_addr_low;
	u32 val, dctlreg = 0;
	struct vme_bridge *fake_bridge;
	struct fake_driver *bridge;

	ctrlr = list->parent;

	fake_bridge = ctrlr->parent;

	bridge = fake_bridge->driver_priv;

	mutex_lock(&ctrlr->mtx);

	channel = ctrlr->number;

	if (!list_empty(&ctrlr->running)) {
		/*
		 * XXX We have an active DMA transfer and currently haven't
		 *     sorted out the mechanism for "pending" DMA transfers.
		 *     Return busy.
		 */
		/* Need to add to pending here */
		mutex_unlock(&ctrlr->mtx);
		return -EBUSY;
	} else {
		list_add(&list->list, &ctrlr->running);
	}

	/* Get first bus address and write into registers */
	entry = list_first_entry(&list->entries, struct fake_dma_entry,
		list);

	entry->dma_handle = dma_map_single(fake_bridge->parent,
		&entry->descriptor,
		sizeof(struct fake_dma_descriptor), DMA_TO_DEVICE);

	mutex_unlock(&ctrlr->mtx);

	reg_split(entry->dma_handle, &bus_addr_high, &bus_addr_low);

	iowrite32be(bus_addr_high, bridge->base +
		FAKE_LCSR_DMA[channel] + FAKE_LCSR_OFFSET_DNLAU);
	iowrite32be(bus_addr_low, bridge->base +
		FAKE_LCSR_DMA[channel] + FAKE_LCSR_OFFSET_DNLAL);

	dctlreg = ioread32be(bridge->base + FAKE_LCSR_DMA[channel] +
		FAKE_LCSR_OFFSET_DCTL);

	/* Start the operation */
	iowrite32be(dctlreg | FAKE_LCSR_DCTL_DGO, bridge->base +
		FAKE_LCSR_DMA[channel] + FAKE_LCSR_OFFSET_DCTL);

	wait_event_interruptible(bridge->dma_queue[channel],
		fake_dma_busy(ctrlr->parent, channel));

	/*
	 * Read status register, this register is valid until we kick off a
	 * new transfer.
	 */
	val = ioread32be(bridge->base + FAKE_LCSR_DMA[channel] +
		FAKE_LCSR_OFFSET_DSTA);

	if (val & FAKE_LCSR_DSTA_VBE) {
		pr_err("DMA Error. DSTA=%08X\n", val);
		retval = -EIO;
	}

	/* Remove list from running list */
	mutex_lock(&ctrlr->mtx);
	list_del(&list->list);
	mutex_unlock(&ctrlr->mtx);

	return retval;
}

/*
 * Clean up a previously generated link list
 *
 * We have a separate function, don't assume that the chain can't be reused.
 */
static int fake_dma_list_empty(struct vme_dma_list *list)
{
	struct list_head *pos, *temp;
	struct fake_dma_entry *entry;

	struct vme_bridge *fake_bridge = list->parent->parent;

	/* detach and free each entry */
	list_for_each_safe(pos, temp, &list->entries) {
		list_del(pos);
		entry = list_entry(pos, struct fake_dma_entry, list);

		dma_unmap_single(fake_bridge->parent, entry->dma_handle,
			sizeof(struct fake_dma_descriptor), DMA_TO_DEVICE);
		kfree(entry);
	}

	return 0;
}
#endif


/*
 * All 4 location monitors reside at the same base - this is therefore a
 * system wide configuration.
 *
 * This does not enable the LM monitor - that should be done when the first
 * callback is attached and disabled when the last callback is removed.
 */
static int fake_lm_set(struct vme_lm_resource *lm, unsigned long long lm_base,
	u32 aspace, u32 cycle)
{
	int i;
	struct vme_bridge *fake_bridge;
	struct fake_driver *bridge;

	fake_bridge = lm->parent;

	bridge = fake_bridge->driver_priv;

	mutex_lock(&lm->mtx);

	/* If we already have a callback attached, we can't move it! */
	for (i = 0; i < lm->monitors; i++) {
		if (bridge->lm_callback[i] != NULL) {
			mutex_unlock(&lm->mtx);
			pr_err("Location monitor callback attached, can't reset\n");
			return -EBUSY;
		}
	}

	switch (aspace) {
	case VME_A16:
	case VME_A24:
	case VME_A32:
	case VME_A64:
		break;
	default:
		mutex_unlock(&lm->mtx);
		pr_err("Invalid address space\n");
		return -EINVAL;
		break;
	}

	bridge->lm_base = lm_base;
	bridge->lm_aspace = aspace;
	bridge->lm_cycle = cycle;

	mutex_unlock(&lm->mtx);

	return 0;
}

/* Get configuration of the callback monitor and return whether it is enabled
 * or disabled.
 */
static int fake_lm_get(struct vme_lm_resource *lm,
	unsigned long long *lm_base, u32 *aspace, u32 *cycle)
{
	struct fake_driver *bridge;

	bridge = lm->parent->driver_priv;

	mutex_lock(&lm->mtx);

	*lm_base = bridge->lm_base;
	*aspace = bridge->lm_aspace;
	*cycle = bridge->lm_cycle;

	mutex_unlock(&lm->mtx);

	return bridge->lm_enabled;
}

/*
 * Attach a callback to a specific location monitor.
 *
 * Callback will be passed the monitor triggered.
 */
static int fake_lm_attach(struct vme_lm_resource *lm, int monitor,
	void (*callback)(int))
{
	struct vme_bridge *fake_bridge;
	struct fake_driver *bridge;

	fake_bridge = lm->parent;

	bridge = fake_bridge->driver_priv;

	mutex_lock(&lm->mtx);

	/* Ensure that the location monitor is configured - need PGM or DATA */
	if (bridge->lm_cycle == 0) {
		mutex_unlock(&lm->mtx);
		pr_err("Location monitor not properly configured\n");
		return -EINVAL;
	}

	/* Check that a callback isn't already attached */
	if (bridge->lm_callback[monitor] != NULL) {
		mutex_unlock(&lm->mtx);
		pr_err("Existing callback attached\n");
		return -EBUSY;
	}

	/* Attach callback */
	bridge->lm_callback[monitor] = callback;

	/* Ensure that global Location Monitor Enable set */
	bridge->lm_enabled = 1;

	mutex_unlock(&lm->mtx);

	return 0;
}

/*
 * Detach a callback function forn a specific location monitor.
 */
static int fake_lm_detach(struct vme_lm_resource *lm, int monitor)
{
	u32 tmp;
	int i;
	struct fake_driver *bridge;

	bridge = lm->parent->driver_priv;

	mutex_lock(&lm->mtx);

	/* Detach callback */
	bridge->lm_callback[monitor] = NULL;

	/* If all location monitors disabled, disable global Location Monitor */
	tmp = 0;
	for (i = 0; i < lm->monitors; i++) {
		if (bridge->lm_callback[i] != NULL) {
			tmp = 1;
		}
	}

	if (tmp == 0) {
		bridge->lm_enabled = 0;
	}

	mutex_unlock(&lm->mtx);

	return 0;
}

/*
 * Determine Geographical Addressing
 */
static int fake_slot_get(struct vme_bridge *fake_bridge)
{
	return geoid;
}

static void *fake_alloc_consistent(struct device *parent, size_t size,
	dma_addr_t *dma)
{
	void * address;

	address = kzalloc(size, GFP_KERNEL);
	*dma = (dma_addr_t)address;

	return address;
}

static void fake_free_consistent(struct device *parent, size_t size,
	void *vaddr, dma_addr_t dma)
{
	kfree(vaddr);
}

/*
 * Configure CR/CSR space
 *
 * Access to the CR/CSR can be configured at power-up. The location of the
 * CR/CSR registers in the CR/CSR address space is determined by the boards
 * Geographic address.
 *
 * Each board has a 512kB window, with the highest 4kB being used for the
 * boards registers, this means there is a fix length 508kB window which must
 * be mapped onto PCI memory.
 */
static int fake_crcsr_init(struct vme_bridge *fake_bridge)
{
	u32 vstat;
	struct fake_driver *bridge;

	bridge = fake_bridge->driver_priv;

	/* Allocate mem for CR/CSR image */
	bridge->crcsr_kernel = kzalloc(VME_CRCSR_BUF_SIZE, GFP_KERNEL);
	bridge->crcsr_bus = (dma_addr_t)bridge->crcsr_kernel;
	if (bridge->crcsr_kernel == NULL) {
		pr_err("Failed to allocate memory for CR/CSR image\n");
		return -ENOMEM;
	}

	vstat = fake_slot_get(fake_bridge);

	pr_info("CR/CSR Offset: %d\n", vstat);

	return 0;
}

static void fake_crcsr_exit(struct vme_bridge *fake_bridge)
{
	struct fake_driver *bridge;

	bridge = fake_bridge->driver_priv;

	kfree(bridge->crcsr_kernel);
}


static int __init fake_init(void)
{
	int retval, i;
	struct list_head *pos = NULL, *n;
	struct vme_bridge *fake_bridge;
	struct fake_driver *fake_device;
	struct vme_master_resource *master_image;
	struct vme_slave_resource *slave_image;
	struct vme_lm_resource *lm;

	/* We need a fake parent device */
	vme_root = __root_device_register("vme", THIS_MODULE);

	/* If we want to support more than one of each bridge, we need to
	 * dynamically generate this so we get one per device
	 */
	fake_bridge = kzalloc(sizeof(struct vme_bridge), GFP_KERNEL);
	if (fake_bridge == NULL) {
		pr_err("Failed to allocate memory for device structure\n");
		retval = -ENOMEM;
		goto err_struct;
	}

	fake_device = kzalloc(sizeof(struct fake_driver), GFP_KERNEL);
	if (fake_device == NULL) {
		pr_err("Failed to allocate memory for device structure\n");
		retval = -ENOMEM;
		goto err_driver;
	}

	fake_bridge->driver_priv = fake_device;

	fake_bridge->parent = vme_root;

	fake_device->parent = fake_bridge;

	/* Initialize wait queues & mutual exclusion flags */
#ifdef VME_DMA_EN
	init_waitqueue_head(&fake_device->dma_queue[0]);
	init_waitqueue_head(&fake_device->dma_queue[1]);
#endif
	mutex_init(&fake_device->vme_int);
	mutex_init(&fake_bridge->irq_mtx);
	tasklet_init(&fake_device->int_tasklet, fake_VIRQ_tasklet, (unsigned long) fake_bridge);

	strcpy(fake_bridge->name, driver_name);

	/* Add master windows to list */
	INIT_LIST_HEAD(&fake_bridge->master_resources);
	for (i = 0; i < FAKE_MAX_MASTER; i++) {
		master_image = kmalloc(sizeof(struct vme_master_resource),
			GFP_KERNEL);
		if (master_image == NULL) {
			pr_err("Failed to allocate memory for master resource structure\n");
			retval = -ENOMEM;
			goto err_master;
		}
		master_image->parent = fake_bridge;
		spin_lock_init(&master_image->lock);
		master_image->locked = 0;
		master_image->number = i;
		master_image->address_attr = VME_A16 | VME_A24 | VME_A32 |
			VME_A64;
		master_image->cycle_attr = VME_SCT | VME_BLT | VME_MBLT |
			VME_2eVME | VME_2eSST | VME_2eSSTB | VME_2eSST160 |
			VME_2eSST267 | VME_2eSST320 | VME_SUPER | VME_USER |
			VME_PROG | VME_DATA;
		master_image->width_attr = VME_D16 | VME_D32;
		memset(&master_image->bus_resource, 0,
			sizeof(struct resource));
		master_image->kern_base  = NULL;
		list_add_tail(&master_image->list,
			&fake_bridge->master_resources);
	}

	/* Add slave windows to list */
	INIT_LIST_HEAD(&fake_bridge->slave_resources);
	for (i = 0; i < FAKE_MAX_SLAVE; i++) {
		slave_image = kmalloc(sizeof(struct vme_slave_resource),
			GFP_KERNEL);
		if (slave_image == NULL) {
			pr_err("Failed to allocate memory for slave resource structure\n");
			retval = -ENOMEM;
			goto err_slave;
		}
		slave_image->parent = fake_bridge;
		mutex_init(&slave_image->mtx);
		slave_image->locked = 0;
		slave_image->number = i;
		slave_image->address_attr = VME_A16 | VME_A24 | VME_A32 |
			VME_A64 | VME_CRCSR | VME_USER1 | VME_USER2 |
			VME_USER3 | VME_USER4;
		slave_image->cycle_attr = VME_SCT | VME_BLT | VME_MBLT |
			VME_2eVME | VME_2eSST | VME_2eSSTB | VME_2eSST160 |
			VME_2eSST267 | VME_2eSST320 | VME_SUPER | VME_USER |
			VME_PROG | VME_DATA;
		list_add_tail(&slave_image->list,
			&fake_bridge->slave_resources);
	}

#ifdef VME_DMA_EN
	/* Add dma engines to list */
	INIT_LIST_HEAD(&fake_bridge->dma_resources);
	for (i = 0; i < FAKE_MAX_DMA; i++) {
		dma_ctrlr = kmalloc(sizeof(struct vme_dma_resource),
			GFP_KERNEL);
		if (dma_ctrlr == NULL) {
			pr_err("Failed to allocate memory for dma resource structure\n");
			retval = -ENOMEM;
			goto err_dma;
		}
		dma_ctrlr->parent = fake_bridge;
		mutex_init(&dma_ctrlr->mtx);
		dma_ctrlr->locked = 0;
		dma_ctrlr->number = i;
		dma_ctrlr->route_attr = VME_DMA_VME_TO_MEM |
			VME_DMA_MEM_TO_VME | VME_DMA_VME_TO_VME |
			VME_DMA_MEM_TO_MEM | VME_DMA_PATTERN_TO_VME |
			VME_DMA_PATTERN_TO_MEM;
		INIT_LIST_HEAD(&dma_ctrlr->pending);
		INIT_LIST_HEAD(&dma_ctrlr->running);
		list_add_tail(&dma_ctrlr->list,
			&fake_bridge->dma_resources);
	}
#endif

	/* Add location monitor to list */
	INIT_LIST_HEAD(&fake_bridge->lm_resources);
	lm = kmalloc(sizeof(struct vme_lm_resource), GFP_KERNEL);
	if (lm == NULL) {
		pr_err("Failed to allocate memory for location monitor resource structure\n");
		retval = -ENOMEM;
		goto err_lm;
	}
	lm->parent = fake_bridge;
	mutex_init(&lm->mtx);
	lm->locked = 0;
	lm->number = 1;
	lm->monitors = 4;
	list_add_tail(&lm->list, &fake_bridge->lm_resources);

	fake_bridge->slave_get = fake_slave_get;
	fake_bridge->slave_set = fake_slave_set;
	fake_bridge->master_get = fake_master_get;
	fake_bridge->master_set = fake_master_set;
	fake_bridge->master_read = fake_master_read;
	fake_bridge->master_write = fake_master_write;
	fake_bridge->master_rmw = fake_master_rmw;
#ifdef VME_DMA_EN
	fake_bridge->dma_list_add = fake_dma_list_add;
	fake_bridge->dma_list_exec = fake_dma_list_exec;
	fake_bridge->dma_list_empty = fake_dma_list_empty;
#endif
	fake_bridge->irq_set = fake_irq_set;
	fake_bridge->irq_generate = fake_irq_generate;
	fake_bridge->lm_set = fake_lm_set;
	fake_bridge->lm_get = fake_lm_get;
	fake_bridge->lm_attach = fake_lm_attach;
	fake_bridge->lm_detach = fake_lm_detach;
	fake_bridge->slot_get = fake_slot_get;
	fake_bridge->alloc_consistent = fake_alloc_consistent;
	fake_bridge->free_consistent = fake_free_consistent;

	pr_info("Board is%s the VME system controller\n",
		(geoid == 1) ? "" : " not");

	pr_info("VME geographical address is set to %d\n", geoid);

	retval = fake_crcsr_init(fake_bridge);
	if (retval) {
		pr_err("CR/CSR configuration failed.\n");
		goto err_crcsr;
	}

	retval = vme_register_bridge(fake_bridge);
	if (retval != 0) {
		pr_err("Chip Registration failed.\n");
		goto err_reg;
	}

	exit_pointer = fake_bridge;

	return 0;

err_reg:
	fake_crcsr_exit(fake_bridge);
err_crcsr:
err_lm:
	/* resources are stored in link list */
	list_for_each_safe(pos, n, &fake_bridge->lm_resources) {
		lm = list_entry(pos, struct vme_lm_resource, list);
		list_del(pos);
		kfree(lm);
	}
#ifdef VME_DMA_EN
	fake_bridge->dma_list_add = fake_dma_list_add;
err_dma:
	/* resources are stored in link list */
	list_for_each_safe(pos, n, &fake_bridge->dma_resources) {
		dma_ctrlr = list_entry(pos, struct vme_dma_resource, list);
		list_del(pos);
		kfree(dma_ctrlr);
	}
#endif
err_slave:
	/* resources are stored in link list */
	list_for_each_safe(pos, n, &fake_bridge->slave_resources) {
		slave_image = list_entry(pos, struct vme_slave_resource, list);
		list_del(pos);
		kfree(slave_image);
	}
err_master:
	/* resources are stored in link list */
	list_for_each_safe(pos, n, &fake_bridge->master_resources) {
		master_image = list_entry(pos, struct vme_master_resource,
			list);
		list_del(pos);
		kfree(master_image);
	}

	kfree(fake_device);
err_driver:
	kfree(fake_bridge);
err_struct:
	return retval;

}


static void __exit fake_exit(void)
{
	struct list_head *pos = NULL;
	struct list_head *tmplist;
	struct vme_master_resource *master_image;
	struct vme_slave_resource *slave_image;
	int i;
	struct vme_bridge *fake_bridge;
	struct fake_driver *bridge;

	fake_bridge = exit_pointer;

	bridge = fake_bridge->driver_priv;

	pr_debug("Driver is being unloaded.\n");

	/*
	 *  Shutdown all inbound and outbound windows.
	 */
	for (i = 0; i < FAKE_MAX_MASTER; i++)
		bridge->masters[i].enabled = 0;

	for (i = 0; i < FAKE_MAX_SLAVE; i++)
		bridge->slaves[i].enabled = 0;

	/*
	 *  Shutdown Location monitor.
	 */
	bridge->lm_enabled = 0;

	vme_unregister_bridge(fake_bridge);

	fake_crcsr_exit(fake_bridge);
#ifdef VME_DMA_EN
	/* resources are stored in link list */
	list_for_each_safe(pos, tmplist, &fake_bridge->dma_resources) {
		dma_ctrlr = list_entry(pos, struct vme_dma_resource, list);
		list_del(pos);
		kfree(dma_ctrlr);
	}
#endif
	/* resources are stored in link list */
	list_for_each_safe(pos, tmplist, &fake_bridge->slave_resources) {
		slave_image = list_entry(pos, struct vme_slave_resource, list);
		list_del(pos);
		kfree(slave_image);
	}

	/* resources are stored in link list */
	list_for_each_safe(pos, tmplist, &fake_bridge->master_resources) {
		master_image = list_entry(pos, struct vme_master_resource,
			list);
		list_del(pos);
		kfree(master_image);
	}

	kfree(fake_bridge->driver_priv);

	kfree(fake_bridge);

	root_device_unregister(vme_root);
}


MODULE_PARM_DESC(geoid, "Set geographical addressing");
module_param(geoid, int, 0);

MODULE_DESCRIPTION("Fake VME bridge driver");
MODULE_LICENSE("GPL");

module_init(fake_init);
module_exit(fake_exit);
