// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Jongkeun, Choi <jkchoi@nexell.co.kr>
 */

#ifndef _NX_V4L2_H
#define _NX_V4L2_H

#include <linux/atomic.h>
#include <linux/irqreturn.h>

#include <media/media-device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

/**
 * structure for irq sharing
 */
struct nx_v4l2_irq_entry {
	struct list_head entry;
	u32 irqs;
	void *priv;
	irqreturn_t (*handler)(void *);
};

/* macro functions for atomic operations */
#define NX_ATOMIC_SET(V, I) atomic_set(V, I)
#define NX_ATOMIC_SET_MASK(MASK, PTR)  \
	do { \
		int oldval = atomic_read(PTR); \
		int newval = oldval | MASK; \
		atomic_cmpxchg(PTR, oldval, newval); \
	} while (0)
#define NX_ATOMIC_CLEAR_MASK(MASK, PTR) \
	do { \
		int oldval = atomic_read(PTR); \
		int newval = oldval & (~MASK); \
		atomic_cmpxchg(PTR, oldval, newval); \
	} while (0)
#define NX_ATOMIC_READ(PTR)    atomic_read(PTR)
#define NX_ATOMIC_INC(PTR)     atomic_inc(PTR)
#define NX_ATOMIC_DEC(PTR)     atomic_dec(PTR)

struct media_device *nx_v4l2_get_media_device(void);
struct v4l2_device  *nx_v4l2_get_v4l2_device(void);
void *nx_v4l2_get_alloc_ctx(void);
int nx_v4l2_register_subdev(struct v4l2_subdev *sd);
struct v4l2_subdev *nx_v4l2_get_subdev(char *name);

#endif
