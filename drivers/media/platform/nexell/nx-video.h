// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Jongkeun, Choi <jkchoi@nexell.co.kr>
 */

#ifndef _NX_VIDEO_H
#define _NX_VIDEO_H

#define NX_VIDEO_MAX_NAME_SIZE		32
#define NX_VIDEO_MAX_PLANES		3
#define NX_VIDEO_MAX_PADS		2
#define NX_VIDEO_MAX_BUFFERS		16

#define NX_PAD_SINK	0
#define NX_PAD_SOURCE	1

#include <linux/spinlock.h>

struct nx_video;

struct nx_video_format {
	char *name;
	u32   pixelformat;
	u32   mbus_code;
	u32   num_planes;
	u32   num_sw_planes;
	bool  is_separated;
	u32   field;
};

struct nx_video_frame {
	u16 width;
	u16 height;
	u16 stride[NX_VIDEO_MAX_PLANES];
	u32 size[NX_VIDEO_MAX_PLANES];
	struct nx_video_format format;
};

struct nx_video_buffer;
typedef int (*nx_video_buf_done)(struct nx_video_buffer *);

struct nx_video_buffer {
	struct list_head list;
	int consumer_index; /* consumer increment this field after consuming */
	dma_addr_t dma_addr[NX_VIDEO_MAX_PLANES];
	u32 stride[NX_VIDEO_MAX_PLANES];
	void *priv;   /* struct vb2_buffer */
	nx_video_buf_done cb_buf_done;
};

typedef int (*nx_queue_func)(struct nx_video_buffer *, void *);
struct nx_buffer_consumer {
	struct list_head list;
	int index;
	ulong timeout;
	void *priv; /* consumer private data */
	nx_queue_func queue;
	u32 usage_count;
};

struct nx_video_buffer_object {
	struct nx_video *video;
	struct list_head buffer_list;
	spinlock_t slock;
	atomic_t buffer_count;
	struct nx_buffer_consumer *consumer;
};

/* video device type : exclusive */
enum nx_video_type {
	NX_VIDEO_TYPE_CAPTURE = 0,
	NX_VIDEO_TYPE_OUT,
	NX_VIDEO_TYPE_M2M,
	NX_VIDEO_TYPE_MAX,
};

enum nx_buffer_consumer_type {
	/* vq type: V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE */
	NX_BUFFER_CONSUMER_SINK = 0,
	/* vq type: V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE */
	NX_BUFFER_CONSUMER_SOURCE,
	NX_BUFFER_CONSUMER_INVALID
};

struct nx_video {
	char name[NX_VIDEO_MAX_NAME_SIZE];
	u32 type; /* enum nx_video_type */

	struct nx_video_buffer *sink_bufs[NX_VIDEO_MAX_BUFFERS];
	struct nx_video_buffer *source_bufs[NX_VIDEO_MAX_BUFFERS];

	struct v4l2_device *v4l2_dev;
	struct vb2_queue *vbq;

	struct mutex lock; /* for video_device */
	struct video_device vdev;
	/**
	 * pad 0 : sink
	 * pad 1 : source
	 */
	struct media_pad pads[NX_VIDEO_MAX_PADS];

	/* frame[0] : sink, capture */
	/* frame[1] : source, out */
	struct nx_video_frame frame[2];

	/* buffer consumer */
	int (*register_buffer_consumer)(struct nx_video *,
					struct nx_buffer_consumer *,
					enum nx_buffer_consumer_type);
	void (*unregister_buffer_consumer)(struct nx_video *,
					   struct nx_buffer_consumer *,
					   enum nx_buffer_consumer_type);

	/* lock for consumer list */
	spinlock_t lock_consumer;
	/* I'm source */
	struct list_head source_consumer_list;
	int source_consumer_count;
	/* I'm sink */
	struct list_head sink_consumer_list;
	int sink_consumer_count;

	uint32_t open_count;

};

/* macros */
#define vdev_to_nx_video(vdev) container_of(vdev, struct nx_video, video)
#define vbq_to_nx_video(vbq)   container_of(vbq, struct nx_video, vbq)

/* public functions */
struct nx_video *nx_video_create(char *name, u32 type,
		struct v4l2_device *v4l2_dev);
void nx_video_cleanup(struct nx_video *me);
int nx_video_get_buffer_count(struct nx_video_buffer_object *obj);
bool nx_video_done_buffer(struct nx_video_buffer_object *obj);
struct nx_video_buffer *
nx_video_get_next_buffer(struct nx_video_buffer_object *obj, bool remove);
void nx_video_clear_buffer(struct nx_video_buffer_object *obj);
void nx_video_init_vbuf_obj(struct nx_video_buffer_object *obj);
void nx_video_add_buffer(struct nx_video_buffer_object *obj,
			 struct nx_video_buffer *buf);
int nx_video_register_buffer_consumer(struct nx_video_buffer_object *obj,
				      nx_queue_func func,
				      void *data);
void nx_video_unregister_buffer_consumer(struct nx_video_buffer_object *obj);

#endif
