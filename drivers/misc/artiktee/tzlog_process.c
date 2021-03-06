/*********************************************************
 * Copyright (C) 2011 - 2016 Samsung Electronics Co., Ltd All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation version 2 and no later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 *
 *********************************************************/

#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/semaphore.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/namei.h>
#include <linux/dcache.h>
#include <linux/path.h>
#include <linux/vmalloc.h>

#include "circular_buffer.h"
#include "log_level_ree.h"
#include "tzlog_process.h"
#include "tzlog_print.h"
#include "log_system_api_ext.h"

/* #define LOG_RING_BUF_DEBUG */
#ifdef LOG_RING_BUF_DEBUG
#define DPRINTF(X...) \
	tzlog_print(K_INFO, X)
#else
#define DPRINTF(X...)
#endif

#define TZLOG_LOCAL_BUFFERING_SIZE (PAGE_SIZE * 2)
#define TZLOG_TZDAEMON_BUFFERING_SIZE (PAGE_SIZE * 10)
#define TZLOG_COPY_TEMP_SIZE 1024
#define TZLOG_LABEL_SIZE 64

static char buf_label[TZLOG_LABEL_SIZE];

/* Log For Tzdaemon */
static ssize_t tzlog_read(struct file *filp, char __user *buffer, size_t size,
			  loff_t *off);
static unsigned int tzlog_poll(struct file *file, poll_table *wait);
static int tzlog_fasync(int fd, struct file *filp, int on);
static struct fasync_struct *fasync;

typedef struct tzlog_process {
	struct chimera_ring_buffer *ring;
	/* Log For Tzdaemon */
	struct mutex ring_lock;
	struct mutex read_lock;
	wait_queue_head_t empty_wait;
	wait_queue_head_t full_wait;
} s_tzlog_process;

static s_tzlog_process log_data_for_local = {
	.ring = NULL,
};

static s_tzlog_process log_data_for_tzdaemon = {
	.ring = NULL,
	.ring_lock = __MUTEX_INITIALIZER(log_data_for_tzdaemon.ring_lock),
	/* Log For Tzdaemon */
	.read_lock = __MUTEX_INITIALIZER(log_data_for_tzdaemon.read_lock),
	.empty_wait =
	    __WAIT_QUEUE_HEAD_INITIALIZER(log_data_for_tzdaemon.empty_wait),
	.full_wait =
	    __WAIT_QUEUE_HEAD_INITIALIZER(log_data_for_tzdaemon.full_wait),
};

/* Log For Tzdaemon */
static const struct file_operations tzlog_file_operations = {
	.owner = THIS_MODULE,
	.read = tzlog_read,
	.poll = tzlog_poll,
	.fasync = tzlog_fasync,
	.llseek = noop_llseek
};

static struct miscdevice tzlog_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tzlog",
	.fops = &tzlog_file_operations,
	.mode = 0400
};

int init_log_processing_resources(void)
{
	int ret = -1;
	void *buffering_memory_local =
	    vmalloc(TZLOG_LOCAL_BUFFERING_SIZE);
	void *buffering_memory_tzdaemon =
	    vmalloc(TZLOG_TZDAEMON_BUFFERING_SIZE);

	if (buffering_memory_tzdaemon) {
		log_data_for_tzdaemon.ring =
		    chimera_create_ring_buffer_etc(buffering_memory_tzdaemon,
				    TZLOG_TZDAEMON_BUFFERING_SIZE,
				    0, GFP_KERNEL);
		if (log_data_for_tzdaemon.ring != NULL)
			chimera_ring_buffer_clear(log_data_for_tzdaemon.ring);
	} else
		goto exit_processing_resources;

	if (buffering_memory_local) {
		log_data_for_local.ring =
		    chimera_create_ring_buffer_etc(buffering_memory_local,
						   TZLOG_LOCAL_BUFFERING_SIZE,
						   0, GFP_KERNEL);
		if (log_data_for_local.ring != NULL)
			chimera_ring_buffer_clear(log_data_for_local.ring);
	} else
		goto exit_processing_resources;

	ret = misc_register(&tzlog_device);
	return ret;

exit_processing_resources:
	if (buffering_memory_local != NULL)
		vfree(buffering_memory_local);

	if (buffering_memory_tzdaemon != NULL)
		vfree(buffering_memory_tzdaemon);

	return ret;
}

static ssize_t tzlog_read(struct file *filp, char __user *buffer, size_t size,
			  loff_t *off)
{
	int error = mutex_lock_interruptible(&log_data_for_tzdaemon.read_lock);
	ssize_t result;

	if (error < 0)
		return error;

	DPRINTF("TZLOG read %d size = %d, in = %d, out = %d\n",
			(int)size, log_data_for_tzdaemon.ring->size,
			log_data_for_tzdaemon.ring->in,
			log_data_for_tzdaemon.ring->first);

	result =
		chimera_ring_buffer_user_read(log_data_for_tzdaemon.ring,
				(uint8_t *) buffer, size,
				TZLOG_TZDAEMON_BUFFERING_SIZE);

	DPRINTF("TZLOG read result %d size = %d, in = %d, out = %d\n",
			(int)result, log_data_for_tzdaemon.ring->size,
			log_data_for_tzdaemon.ring->in,
			log_data_for_tzdaemon.ring->first);

	if (result > 0)
		wake_up(&log_data_for_tzdaemon.full_wait);

	mutex_unlock(&log_data_for_tzdaemon.read_lock);

	return result;
}

static unsigned int tzlog_poll(struct file *file, poll_table *wait)
{
	unsigned int mask;

	DPRINTF("tzlog_poll call\n");
	poll_wait(file, &log_data_for_tzdaemon.empty_wait, wait);
	DPRINTF("tzlog_poll call end\n");
	mask = 0;

	if (chimera_ring_buffer_readable(log_data_for_tzdaemon.ring))
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static int tzlog_fasync(int fd, struct file *filp, int on)
{
	return fasync_helper(fd, filp, on, &fasync);
}

static int tzlog_copy_buffer(s_tzlog_process *dst_data, int dst_buffer_size,
			     s_tzlog_data *src_data, int src_buffer_size)
{
	struct iovec iov[2];
	int i;
	size_t n_written = 0;
	int nvec =
	    chimera_ring_buffer_get_vecs(src_data->ring, iov, src_buffer_size);
	for (i = 0; i < nvec; ++i) {
		ssize_t written = chimera_ring_buffer_write(dst_data->ring,
							    (const uint8_t *)
							    iov[i].iov_base,
							    iov[i].iov_len,
							    dst_buffer_size);
		/* Occur error */
		if (written < 0)
			break;

		/* Can not write All Log */
		if (written == 0)
			break;

		n_written += written;
	}
	return n_written;
}

void tzlog_transfer_to_tzdaemon(s_tzlog_data *src_data, int src_buffer_size)
{
	size_t ret_write = 0;
	size_t src_size = 0;
	size_t origin_size = 0;
	size_t avail = 0;

	src_size = chimera_ring_buffer_readable(src_data->ring);
	mutex_lock(&log_data_for_tzdaemon.ring_lock);

	/* If we can not completely write, we should be wait for tzdaemon */
	avail = chimera_ring_buffer_writable(log_data_for_tzdaemon.ring);
	if (avail < src_size) {
		wake_up_interruptible(&log_data_for_tzdaemon.empty_wait);
		kill_fasync(&fasync, SIGIO, POLL_IN);

		/* Wait for tzdaemon's read */
		tzlog_print(K_WARNING,
			    "tzdaemon log full. wait for drain(src:%d,avail:%d)\n",
			    src_size, avail);

		mutex_unlock(&log_data_for_tzdaemon.ring_lock);
		wait_event_timeout(log_data_for_tzdaemon.full_wait,
				   chimera_ring_buffer_writable
				   (log_data_for_tzdaemon.ring) >= src_size,
				   5 * HZ);
		mutex_lock(&log_data_for_tzdaemon.ring_lock);
		if (chimera_ring_buffer_writable(log_data_for_tzdaemon.ring) <
		    src_size) {
			chimera_ring_buffer_clear(log_data_for_tzdaemon.ring);
			tzlog_print(K_WARNING,
				    "tzdaemon was cleared necessarily\n");
		}
	}

	/* Copy to tzdaemon-ring-buffer from src_data */
	origin_size = chimera_ring_buffer_readable(log_data_for_tzdaemon.ring);
	ret_write =
	    tzlog_copy_buffer(&log_data_for_tzdaemon,
			      TZLOG_TZDAEMON_BUFFERING_SIZE, src_data,
			      src_buffer_size);
	if (src_size != ret_write || ret_write == 0) {
		int stored_size =
		    chimera_ring_buffer_readable(log_data_for_tzdaemon.ring);
		if (stored_size - origin_size != src_size) {
			tzlog_print(K_WARNING,
				    "log can loss(tzdaemon, src sz : %d, written sz: %d stored sz : %d)\n",
				    src_size, ret_write, stored_size);
		}
	}
	mutex_unlock(&log_data_for_tzdaemon.ring_lock);

	wake_up_interruptible(&log_data_for_tzdaemon.empty_wait);
	kill_fasync(&fasync, SIGIO, POLL_IN);
}

static void tzlog_print_with_header(log_header_type *header, int body_size)
{
	char *read_body = NULL;
	char *temp_read_body = NULL;
	static char keep_read_body[TZLOG_COPY_TEMP_SIZE] = { 0, };

	memset(keep_read_body, 0, sizeof(keep_read_body));

	if (sizeof(keep_read_body) - 1 > body_size) {
		/* If the buffer is sufficient */
		read_body = keep_read_body;
	} else {
		/* If the buffer is not sufficient */
		temp_read_body = (char *)vmalloc(body_size + 1);
		if (temp_read_body != NULL) {
			memset(temp_read_body, 0, body_size + 1);
			read_body = temp_read_body;
		}
	}

	/* output log */
	if (read_body != NULL) {
		chimera_ring_buffer_read(log_data_for_local.ring,
					 (uint8_t *) read_body, body_size,
					 TZLOG_LOCAL_BUFFERING_SIZE);
		if (header != NULL) {
			memset(buf_label, 0, sizeof(buf_label));
			if (header->label_size <= sizeof(buf_label)) {
				memcpy(buf_label, read_body,
				       header->label_size);
			}
			tzlog_print_for_tee(header->level, buf_label,
					    (read_body + header->label_size));
		} else
			tzlog_print_for_tee(NO_HEADER_LOG_LEVEL, NULL, read_body);
	}

	if (temp_read_body != NULL) {
		vfree(temp_read_body);
		temp_read_body = NULL;
	}
}

void tzlog_transfer_to_local(s_tzlog_data *src_data, int src_buffer_size)
{
	size_t ret_write = 0;
	size_t src_size = 0;
	size_t ring_data_origin_size = 0;
	int ring_data_remain_size = 0;
	int is_need_continue = 0;

	ring_data_origin_size =
	    chimera_ring_buffer_readable(log_data_for_local.ring);
	src_size = chimera_ring_buffer_readable(src_data->ring);

	DPRINTF("tzdev receive log(%d) ori(%d)\n", src_size,
		ring_data_origin_size);

	ret_write =
	    tzlog_copy_buffer(&log_data_for_local, TZLOG_LOCAL_BUFFERING_SIZE,
				src_data, src_buffer_size);
	if (src_size != ret_write || ret_write == 0) {
		int stored_size =
		    chimera_ring_buffer_readable(log_data_for_local.ring);
		if (stored_size - ring_data_origin_size != src_size) {
			DPRINTF("log can loss(tzdev, src sz:%d, written sz:%d, stored sz:%d)\n",
					src_size, ret_write, stored_size);
		}
	}

continue_read:

	ring_data_remain_size =
	    chimera_ring_buffer_readable(log_data_for_local.ring);
	if (ring_data_remain_size >= LHDSIZE) {
		log_header_type log_header;
		int head_size = LHDSIZE;
		int body_size = 0;
		char header[LHDSIZE + 1] = { 0, };

		chimera_ring_buffer_peek(log_data_for_local.ring, 0, header,
					 LHDSIZE, TZLOG_LOCAL_BUFFERING_SIZE);

		if (get_log_header(header, &log_header) != 1) {
			int is_find_header = 0;
			int find_cnt = 0;
			int find_max_cnt = ring_data_remain_size - LHDSIZE + 1;

			/* Magic Check */
			DPRINTF("Magic compare (%c %c %c %c)\n",
					header[0], header[1], header[2], header[3]);
			DPRINTF("Header is not compete\n");

			for (; find_cnt < find_max_cnt; find_cnt++) {
				chimera_ring_buffer_peek(log_data_for_local.ring,
						find_cnt,
						header,
						LHDSIZE,
						TZLOG_LOCAL_BUFFERING_SIZE);
				if (get_log_header(header, &log_header) == 1) {
					DPRINTF("Header find\n");
					is_find_header = 1;
					break;
				}
			}

			if (is_find_header == 1) {
				DPRINTF("print msg(without magic-case-1)\n");
				tzlog_print_with_header(NULL, find_cnt);
				goto continue_read;
			} else {
				DPRINTF("print msg(without magic-case-2)\n");
				tzlog_print_with_header(NULL,
						ring_data_remain_size);
			}
		} else {
			if (log_header.body_size == 0
					|| log_header.label_size == 0) {
				DPRINTF("Header is compete(%s)\n", header);
				DPRINTF("body size:%d/ label size:%d/ level:%d/ pid:%d\n",
						log_header.body_size,
						log_header.label_size,
						log_header.level,
						log_header.pid);
			}
			is_need_continue = 1;
			/*
			 * if header ok & body size is enough,
			 * will be processing
			 */
			body_size = log_header.body_size;
			if (chimera_ring_buffer_readable
					(log_data_for_local.ring) >=
					(head_size + body_size)) {
				DPRINTF("Can receive body data\n");
				chimera_ring_buffer_read(log_data_for_local.ring,
						(uint8_t *) header,
						head_size,
						TZLOG_LOCAL_BUFFERING_SIZE);
				tzlog_print_with_header(&log_header, body_size);
			} else {
				DPRINTF("body size is %d / label size is %d / level %d / pid %d\n",
						log_header.body_size,
						log_header.label_size,
						log_header.level,
						log_header.pid);
				DPRINTF("Header + Body size is not enough(%d,%d)\n",
						head_size, body_size);
				is_need_continue = 0;
			}
			if (is_need_continue == 1)
				goto continue_read;
		}
	} else {
		if (ring_data_remain_size > 0) {
			int magic_ret = 0;
			char header[LHDSIZE + 1] = { 0, };

			chimera_ring_buffer_peek(log_data_for_local.ring, 0,
					header, ring_data_remain_size,
					TZLOG_LOCAL_BUFFERING_SIZE);
			magic_ret = get_check_magic(header);
			if (magic_ret == NO_MAGIC) {
				tzlog_print_with_header(NULL,
						ring_data_remain_size);
				DPRINTF("Find NO_MAGIC Data - (size : %d) (%s)\n",
						ring_data_remain_size, (char *)header);
			}
		}
	}
}
